#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>

#include <jeff_segment_objects/Cluster.h>
#include <jeff_segment_objects/Plane.h>
#include <jeff_segment_objects/SegmentObjects.h>
#include <jeff_segment_objects/RemoveBox.h>

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
#include <pcl/features/normal_3d.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/conditional_removal.h>

// transform listener to project objects onto base_footprint
tf::TransformListener* listener;

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
bool segment_objects(jeff_segment_objects::SegmentObjects::Request &req, jeff_segment_objects::SegmentObjects::Response &res)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr xtion_frame_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr nan_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndices::Ptr root_indices(new pcl::PointIndices);
    std::vector<pcl::ModelCoefficients> plane_coefficients;


    // convert ros cloud_msg input 
    pcl::fromROSMsg(req.cloud_msg, *xtion_frame_cloud);
    pcl_ros::transformPointCloud("/base_footprint", *xtion_frame_cloud, *cloud, *listener);
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
    plane_coefficients.push_back(*coefficients);
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
    while (cloud->size() > initial_size * req.non_planar_ratio)
    {
        // segment planes from cloud
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
        plane_coefficients.push_back(*coefficients);
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
    ec.setClusterTolerance(req.cluster_tolerance);
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    // set response
    // for each PointIndices object, add all points to a Cluster
    // append the Cluster to response's clusters vector
    for (std::vector<pcl::PointIndices>::const_iterator cluster_it = cluster_indices.begin (); cluster_it!= cluster_indices.end (); ++cluster_it)
    {
        jeff_segment_objects::Cluster cluster;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        for (std::vector<int>::const_iterator point_it = cluster_it->indices.begin (); point_it != cluster_it->indices.end (); ++point_it) {
            cluster.indices.push_back(root_indices->indices[*point_it]);
            cluster_cloud->points.push_back(cloud->points[*point_it]);
        }

        // get 3D info about the object (taken from fy16m3aa@leeds.ac.uk)
        // setup cloud for 3D calculations
        cluster_cloud->width = cluster_cloud->points.size();
        cluster_cloud->height = 1;
        cluster_cloud->is_dense = true;

        // calculate object center point
        pcl::PointXYZ object_centroid;
        pcl::computeCentroid(*cluster_cloud, object_centroid);
        cluster.position.header.stamp = req.cloud_msg.header.stamp;
        cluster.position.header.frame_id = "base_footprint";
        cluster.position.point.x = object_centroid.x;
        cluster.position.point.y = object_centroid.y;
        cluster.position.point.z = object_centroid.z;

        // get size of object
        pcl::PointXYZ object_min, object_max;
        pcl::getMinMax3D(*cluster_cloud, object_min, object_max);
        cluster.size.x = std::abs(object_max.x - object_min.x);
        cluster.size.y = std::abs(object_max.y - object_min.y);
        cluster.size.z = std::abs(object_max.z - object_min.z);




        res.clusters.push_back(cluster);
    }
    // do the same for planes
    for (std::vector<pcl::ModelCoefficients>::const_iterator plane_it = plane_coefficients.begin (); plane_it!= plane_coefficients.end (); ++plane_it)
    {
        jeff_segment_objects::Plane plane;
        for (std::vector<float>::const_iterator coefficient_it = plane_it->values.begin (); coefficient_it != plane_it->values.end (); ++coefficient_it)
            plane.coefficients.push_back(*coefficient_it);

        res.planes.push_back(plane);
    }
    
    return true;
}

bool remove_box(jeff_segment_objects::RemoveBox::Request &req, jeff_segment_objects::RemoveBox::Response &res) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr xtion_frame_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr nan_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // convert ros cloud_msg input 
    pcl::fromROSMsg(req.points, *xtion_frame_cloud);
    pcl_ros::transformPointCloud("/base_footprint", *xtion_frame_cloud, *cloud, *listener);

    // remove nans
    std::vector<int> nan_indices;
    pcl::removeNaNFromPointCloud(*cloud, *nan_filtered_cloud, nan_indices);
    cloud.swap(nan_filtered_cloud);

    // set conditions
    pcl::ConditionOr<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionOr<pcl::PointXYZ> ());
    
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
        pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, req.max.x + req.padding)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
        pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::GT, req.max.y + req.padding)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
        pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, req.max.z + req.padding)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
        pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, req.min.x - req.padding)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
        pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::LT, req.min.y - req.padding)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
        pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, req.min.z - req.padding)));

    // build the filter
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    condrem.setCondition (range_cond);
    condrem.setInputCloud (cloud);
    condrem.setKeepOrganized(true);
    condrem.filter (*filtered_cloud);

    cloud.swap(filtered_cloud);

    pcl::toROSMsg(*cloud, res.points);

    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "segment_objects");
    listener = new tf::TransformListener();
    ros::Duration(2).sleep();
    ros::NodeHandle n;
    ros::ServiceServer segment_service = n.advertiseService("segment_objects", segment_objects);
    ros::ServiceServer remove_service = n.advertiseService("remove_box", remove_box);
    ros::spin();
}