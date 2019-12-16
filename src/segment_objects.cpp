#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <jeff_segment_objects/Cluster.h>
#include <jeff_segment_objects/SegmentObjects.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

/*
TODO: actionlib for asynchronous behaviour
    - goal can be pushed to input queue
    - global result
    - result can be set and returned at the callback level
*/

std::queue<jeff_segment_objects::SegmentObjects::Response*> input_queue;
std::queue<int> output_queue;
float cluster_tolerance;
float non_planar_ratio;

bool segment_objects(jeff_segment_objects::SegmentObjects::Request  &req,
                     jeff_segment_objects::SegmentObjects::Response &res)
{
    std::cout << "service request" << std::endl;
    cluster_tolerance = req.cluster_tolerance;
    non_planar_ratio = req.non_planar_ratio;

    input_queue.push(&res);

    while(output_queue.empty())
        ros::Duration(1).sleep();

    output_queue.pop();
    return true;
}

// map inliers from a cloud filter to their root indices
// kept indices can thus be traced back to the root cloud after applying multiple filters
//
void mapInliersToRootIndices(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices::Ptr inliers, pcl::PointIndices::Ptr root_indices)
{
    int i = 0;
    pcl::PointIndices::Ptr temp_indices (new pcl::PointIndices);
    for(int j = 0; j < cloud->size(); ++j)
    {
        while(inliers->indices[i] < j && i < inliers->indices.size())
            ++i;
        
        if(j != inliers->indices[i])
            temp_indices->indices.push_back(root_indices->indices[j]);
    }
    root_indices->indices = temp_indices->indices;
}

// cloud and image callback always running to reduce overhead of new subscribers
//
void callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg, const sensor_msgs::Image::ConstPtr& image_msg)
{
    std::cout << "callback, input queue is empty: " << input_queue.empty() << std::endl;
    if(!input_queue.empty())
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr nan_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointIndices::Ptr root_indices(new pcl::PointIndices);

        // convert ros cloud_msg input 
        pcl::fromROSMsg(*cloud_msg, *cloud);
        int initial_size = cloud->size();

        std::vector<int> nan_indices;
        pcl::removeNaNFromPointCloud(*cloud, *nan_filtered_cloud, nan_indices);

        std::cout << "cloud size: " << cloud->size() << " inlier size: " << nan_indices.size() << " filtered: " << nan_filtered_cloud->size() << std::endl;

        cloud.swap(nan_filtered_cloud);
        root_indices->indices = nan_indices;

        // plane segmentation
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(1000);
        seg.setDistanceThreshold(0.01);

        // segment planes from cloud
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
        // filter the segmented plane from cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        extract.filter(*filtered_cloud);

        std::cout << "cloud size: " << cloud->size() << " inliers size: " << inliers->indices.size() << " filtered: " << filtered_cloud->size() << std::endl;

        // map to root indices
        mapInliersToRootIndices(cloud, inliers, root_indices);
        cloud.swap(filtered_cloud);

        // extract planes until 80% of the pcl is removed
        while (cloud->size() > initial_size * 0.2)
        {
            // segment planes from cloud
            seg.setInputCloud(cloud);
            seg.segment(*inliers, *coefficients);
            // filter the segmented plane from cloud
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(cloud);
            extract.setIndices(inliers);
            extract.setNegative(true);
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            extract.filter(*filtered_cloud);

            std::cout << "cloud size: " << cloud->size() << " inliers size: " << inliers->indices.size() << " filtered: " << filtered_cloud->size() << std::endl;

            // map to root indices
            mapInliersToRootIndices(cloud, inliers, root_indices);
            cloud.swap(filtered_cloud);
        }
        std::cout << std::endl << std::endl;
        
        // create a tree for cluster extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);

        // cluster extraction
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        std::vector<pcl::PointIndices> cluster_indices;
        ec.setClusterTolerance(0.035);
        ec.setMinClusterSize(100);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        // cloud out and response
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<jeff_segment_objects::Cluster> clusters;

        // for each PointIndices object, add all points to a Cluster vector
        // and append it to the ClusterList
        for (std::vector<pcl::PointIndices>::const_iterator cluster_it = cluster_indices.begin (); cluster_it!= cluster_indices.end (); ++cluster_it)
        {
            jeff_segment_objects::Cluster cluster;
            for (std::vector<int>::const_iterator point_it = cluster_it->indices.begin (); point_it != cluster_it->indices.end (); ++point_it)
            {
                cloud_cluster->points.push_back(cloud->points[*point_it]);
                cluster.indices.push_back(root_indices->indices[*point_it]);
            }
            clusters.push_back(cluster);
        }

        // convert to pcl2
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        sensor_msgs::PointCloud2::Ptr cloud_out (new sensor_msgs::PointCloud2);
        pcl::toROSMsg (*cloud_cluster, *cloud_out);
        cloud_out->header = cloud_msg->header;

        // set response
        jeff_segment_objects::SegmentObjects::Response* res_ptr = input_queue.front();
        res_ptr->header = cloud_msg->header;
        res_ptr->cloud = *cloud_msg;
        res_ptr->image = *image_msg;
        res_ptr->clusters = clusters;
        input_queue.pop();
        output_queue.push(0);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_redirect");
    ros::NodeHandle n;
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(n, "/xtion/depth_registered/points", 1);
    message_filters::Subscriber<sensor_msgs::Image> image_sub(n, "/xtion/rgb/image_rect_color", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(5), cloud_sub, image_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::ServiceServer service = n.advertiseService("segment_objects", segment_objects);
    ros::spin();
}