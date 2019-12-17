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