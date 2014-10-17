#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>


//class Detector{

//private:
//    ros::NodeHandle nh;
//    ros::Publisher pub;
//    ros::Subscriber sub;

//    pcl::PointCloud<pcl::PointXYZRGB> pointCloud;
//    //pcl::MsgFieldMap fieldMap;

//    //ros::Rate loop_rate(10);

//public:

//    Detector(int argc, char **argv){
//        ros::init(argc, argv, "detector");

//        //nh = ros::NodeHandle();

//        //sub = nh.subscribe("/camera/depth_registered/points", 1, pointcloudCallback);
//        //pub = nh.advertise<geometry_msgs::Point>("hand-detector/target", 1);

//        //ros::Rate test_rate(10);

//        //loop_rate(10);

//        return;
//    }

//    void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){

//        //pcl::PCLPointCloud2 pcl_pc;
//        //pcl::fromROSMsg(msg, pointCloud, fieldMap);
//        //pcl::fromROSMsg(msg, pointCloud);

//        return;
//    }

//    //pcl::PointCloud extractObservationArea(const ros::PointCloud& pc){

//        //pcl::PointCloud observedPoints;
//        //return pcl::PointCloudl();

//    //}

//    void printMaxVals(){

//        double left = 0;
//        double righ = 0;
//        double top = 0;
//        double bottom = 0;

//        //int width = pointCloud.width;
//        //int height = pointCloud.height;

////        for(size_t i = 0; i < width; i++){
////            //top = std::max(top, (double) pointCloud.points[i].data_c[1]);
////        }
////        //ROS_INFO("top: %d", top);

////        for(size_t i = 0; i < width; i++){
////            bottom = std::min(bottom, (double) pointCloud.points[((height-1) * width) + i].data_c[1]);
////        }
////        //ROS_INFO("bottom: %d", bottom);

////        for(size_t i = 0; i < height; i++){
////            //left = std::min(left, (double) pointCloud.points[i * width].data_c[0]);
////        }
////        //ROS_INFO("left: %d", left);

////        for(size_t i = 0; i < height; i++){
////            //right = std::max(right, (double) pointCloud.points[(i * width) + width - 1].data_c[0]);
////        }
//        //ROS_INFO("right: %d", right);
//    }

//    void sleep(){

//        //loop_rate.sleep();
//        return;
//    }

//};

    pcl::PointCloud<pcl::PointXYZRGB> pointCloud;

    void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
        pcl::fromROSMsg(*msg, pointCloud);

        return;
    }

int main(int argc, char **argv){

    //Detector detector(argc, argv);

    ros::init(argc, argv, "detector");

    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 1, pointcloudCallback);
    ros::Publisher croppedpub = nh.advertise<sensor_msgs::PointCloud2>("hand_detector/cropped", 1);
    ros::Publisher targetpub = nh.advertise<geometry_msgs::Point>("hand_detector/target", 1);

    pcl::CropBox<pcl::PointXYZRGB> cropBox;

    cropBox.setMin(Eigen::Vector4f(-0.5, -0.2, 0.5, 1.0));
    cropBox.setMax(Eigen::Vector4f(0.5, 0.5, 1.5, 1.0));

    ros::Rate loop_rate(5);

    while(ros::ok()){

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_ptr = pointCloud.makeShared();

        cropBox.setInputCloud(tmp_ptr);

        pcl::PointCloud<pcl::PointXYZRGB> newPointCloud;

        cropBox.filter(newPointCloud);

        pcl::PointXYZ nearestPoint;
        double minDistance = 10;

        for(size_t i = 0; i < newPointCloud.points.size(); i++){
            pcl::PointXYZRGB currPoint = newPointCloud.points[i];
            double dist = std::sqrt(std::pow(currPoint.x, 2) + std::pow(currPoint.y, 2) + std::pow(currPoint.z, 2));

            if(dist < minDistance){
                minDistance = dist;
                nearestPoint.x = currPoint.x;
                nearestPoint.y = currPoint.y;
                nearestPoint.z = currPoint.z;
                ROS_INFO("new min dist: %f", minDistance);
            }
        }

        sensor_msgs::PointCloud2 croppedCloudMsg;
        pcl::toROSMsg(newPointCloud, croppedCloudMsg);
        croppedpub.publish(croppedCloudMsg);

        geometry_msgs::Point targetPoint;
        targetPoint.x = nearestPoint.x;
        targetPoint.y = nearestPoint.y;
        targetPoint.z = nearestPoint.z;
        targetpub.publish(targetPoint);

        ros::spinOnce();

        loop_rate.sleep();

    }
    return 0;
}
