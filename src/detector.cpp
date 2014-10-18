#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>


class Detector{

private:
    ros::NodeHandle nh;

    ros::Subscriber sub;

    ros::Publisher croppedCloudPub; //for debugging
    ros::Publisher targetPub; //sends the detected target point

    pcl::PointCloud<pcl::PointXYZRGB> pointCloud; //the current pointcloud of the environment
    pcl::CropBox<pcl::PointXYZRGB> cropBox; //filter to remove all parts outside a box


    //extract a 3D box from the pointcloud
    pcl::PointCloud<pcl::PointXYZRGB> maskPointCloud(){

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_ptr = pointCloud.makeShared();
        cropBox.setInputCloud(tmp_ptr);

        pcl::PointCloud<pcl::PointXYZRGB> newPointCloud;
        cropBox.filter(newPointCloud);

        return newPointCloud;
    }

    //finds the closest point to the basis
    pcl::PointXYZ findClosestPoint(pcl::PointCloud<pcl::PointXYZRGB> pc){

        pcl::PointXYZ closestPoint;
        double minDistance = 100;

        for(size_t i = 0; i < pc.points.size(); i++){
            pcl::PointXYZRGB currPoint = pc.points[i];
            double dist = std::sqrt(std::pow(currPoint.x, 2) + std::pow(currPoint.y, 2) + std::pow(currPoint.z, 2));

            if(dist < minDistance){
                minDistance = dist;
                closestPoint.x = currPoint.x;
                closestPoint.y = currPoint.y;
                closestPoint.z = currPoint.z;
                //ROS_INFO("new min dist: %f", minDistance);
            }
        }
        return closestPoint;
    }

public:

    Detector(){

        //TODO: change namespace??
        nh = ros::NodeHandle();

        sub = nh.subscribe("/camera/depth_registered/points", 1, &Detector::pointcloudCallback, this);
        croppedCloudPub = nh.advertise<sensor_msgs::PointCloud2>("hand_detector/cropped", 1);
        targetPub = nh.advertise<geometry_msgs::Point>("hand_detector/target", 1);

        cropBox = pcl::CropBox<pcl::PointXYZRGB>();

        cropBox.setMin(Eigen::Vector4f(-0.5, -0.5, 0.0, 1.0));
        cropBox.setMax(Eigen::Vector4f(0.5, 0.5, 10.0, 1.0));

        return;
    }

    //callback function to receive messages
    void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){

        pcl::fromROSMsg(*msg, pointCloud);

        return;
    }


    //the main function that detects the hand (or object) in the pointcloud
    void detectHand(){

        pcl::PointCloud<pcl::PointXYZRGB> croppedPointCloud = maskPointCloud();

        pcl::PointXYZ closestPoint = findClosestPoint(croppedPointCloud);

        sensor_msgs::PointCloud2 croppedCloudMsg;
        pcl::toROSMsg(croppedPointCloud, croppedCloudMsg);
        croppedCloudPub.publish(croppedCloudMsg);

        geometry_msgs::Point targetPoint;
        targetPoint.x = closestPoint.x;
        targetPoint.y = closestPoint.y;
        targetPoint.z = closestPoint.z;
        targetPub.publish(targetPoint);

        return;
    }

};


int main(int argc, char **argv){

    ros::init(argc, argv, "detector");

    Detector detector;

    ros::Rate loop_rate(5);

    while(ros::ok()){

        detector.detectHand();

        ros::spinOnce();

        loop_rate.sleep();

    }
    return 0;
}
