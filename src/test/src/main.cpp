#include <iostream>
#include <fstream>

#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#define MEGA

pcl::PointCloud<pcl::PointXYZI>::Ptr input_points(new pcl::PointCloud<pcl::PointXYZI>());
int cnt = 0 ;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle nh("~");
    ros::Publisher pub_points;
    pub_points = nh.advertise<sensor_msgs::PointCloud2>("/lidar_bin", 1);

   // std::fstream load_lidar_data;
    ros::Rate loop_rate(10);

    std::string input_bin_full_path = "/home/dgist/Documents/0914/data/bin/0000.bin";
    std::fstream load_lidar_data(input_bin_full_path,std::ios::binary | std::ios::in);

    if(!load_lidar_data)
    {
        std::cout<<"no data"<<std::endl;
    }

    for (int i=0; load_lidar_data.good() && !load_lidar_data.eof(); i++) {  //convert bin to point cloud
        pcl::PointXYZI points;
        load_lidar_data.read((char*)&points.x, 3*sizeof(float));
        load_lidar_data.read((char*)&points.intensity, sizeof(float));
        input_points->push_back(points);
    }

    load_lidar_data.close();


    while(ros::ok())
    {
        ros::spinOnce();

        input_points->header.frame_id = "velodyne";    //make ros msg
        sensor_msgs::PointCloud2 points_out;
        pcl::toROSMsg(*input_points, points_out);
        pub_points.publish(points_out);

        int wk = cv::waitKey(30);
        if(wk == 'q')
            break;

        loop_rate.sleep();
    }

   return 0;
}
