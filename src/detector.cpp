#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>


class Detector{

private:
    ros::NodeHandle nh;

    ros::Subscriber sub;

    ros::Publisher croppedCloudPub; //for debugging
    ros::Publisher targetPub; //sends the detected target point

    pcl::PointCloud<pcl::PointXYZ> pointCloud; //the current pointcloud of the environment
    pcl::CropBox<pcl::PointXYZ> cropBox; //filter to remove all parts outside a box
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> voxelGrid; //downsample data


    //extract a 3D box from the pointcloud
    pcl::PointCloud<pcl::PointXYZ> maskPointCloud(const pcl::PointCloud<pcl::PointXYZ>& pc){

        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_ptr = pc.makeShared();
        cropBox.setInputCloud(tmp_ptr);

        pcl::PointCloud<pcl::PointXYZ> newPointCloud;
        cropBox.filter(newPointCloud);

        return newPointCloud;
    }

    //downsample the given pointcloud
    pcl::PointCloud<pcl::PointXYZ> downSample(const pcl::PointCloud<pcl::PointXYZ>& pc){

        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_ptr = pc.makeShared();
        voxelGrid.setInputCloud(tmp_ptr);

        pcl::PointCloud<pcl::PointXYZ> newPointCloud;
        voxelGrid.filter(newPointCloud);

        return newPointCloud;
    }

    //finds the closest point to the origin
    pcl::PointXYZ findClosestPoint(pcl::PointCloud<pcl::PointXYZ> pc){

        pcl::PointXYZ closestPoint;
        double minDistance = 100;

        for(size_t i = 0; i < pc.points.size(); i++){
            pcl::PointXYZ currPoint = pc.points[i];
            double dist = std::sqrt(std::pow(currPoint.x, 2) + std::pow(currPoint.y, 2) + std::pow(currPoint.z, 2));

            if(dist < minDistance){
                minDistance = dist;
                closestPoint = currPoint;
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

        cropBox = pcl::CropBox<pcl::PointXYZ>();

        cropBox.setMin(Eigen::Vector4f(-0.5, -0.5, 0.0, 1.0));
        cropBox.setMax(Eigen::Vector4f(0.5, 0.5, 3.0, 1.0));

        voxelGrid = pcl::ApproximateVoxelGrid<pcl::PointXYZ>();

        voxelGrid.setLeafSize(0.1, 0.1, 0.05);

        return;
    }

    //callback function to receive messages
    void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){

        pcl::fromROSMsg(*msg, pointCloud);

        return;
    }


    //the main function that detects the hand (or object) in the pointcloud
    void detectHand(){

        pcl::PointCloud<pcl::PointXYZ> croppedPointCloud = maskPointCloud(pointCloud);
        pcl::PointCloud<pcl::PointXYZ> downSampledPointCloud = downSample(croppedPointCloud);

        pcl::PointXYZ closestPoint = findClosestPoint(downSampledPointCloud);

        sensor_msgs::PointCloud2 dsCloudMsg;
        pcl::toROSMsg(downSampledPointCloud, dsCloudMsg);
        croppedCloudPub.publish(dsCloudMsg);

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
