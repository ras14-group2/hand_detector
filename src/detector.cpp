#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/approximate_voxel_grid.h>
//#include <pcl/filters/median_filter.h>
#include <pcl/filters/statistical_outlier_removal.h>

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
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> fineGrid; //downsample data
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> coarseGrid; //downsample data
//    pcl::MedianFilter<pcl::PointXYZ> medianFilter; //median filter to remove outliers
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outlierRemoval; //remove outliers


    //extract a 3D box from the pointcloud
    pcl::PointCloud<pcl::PointXYZ> maskPointCloud(const pcl::PointCloud<pcl::PointXYZ>& pc){

        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_ptr = pc.makeShared();
        cropBox.setInputCloud(tmp_ptr);

        pcl::PointCloud<pcl::PointXYZ> newPointCloud;
        cropBox.filter(newPointCloud);

        return newPointCloud;
    }

    //downsample the given pointcloud with fine grid
    pcl::PointCloud<pcl::PointXYZ> sampleFineGrid(const pcl::PointCloud<pcl::PointXYZ>& pc){

        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_ptr = pc.makeShared();
        fineGrid.setInputCloud(tmp_ptr);

        pcl::PointCloud<pcl::PointXYZ> newPointCloud;
        fineGrid.filter(newPointCloud);

        return newPointCloud;
    }

    //downsample the given pointcloud with coarse grid
    pcl::PointCloud<pcl::PointXYZ> sampleCoarseGrid(const pcl::PointCloud<pcl::PointXYZ>& pc){

        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_ptr = pc.makeShared();
         coarseGrid.setInputCloud(tmp_ptr);

        pcl::PointCloud<pcl::PointXYZ> newPointCloud;
        coarseGrid.filter(newPointCloud);

        return newPointCloud;
    }

    //apply median filter to remove outliers
    pcl::PointCloud<pcl::PointXYZ> removeoutliers(const pcl::PointCloud<pcl::PointXYZ>& pc){

//        ROS_INFO("points given to outlier removal: %d", pc.points.size());

        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_ptr = pc.makeShared();
        //medianFilter.setInputCloud(tmp_ptr);
        outlierRemoval.setInputCloud(tmp_ptr);

        pcl::PointCloud<pcl::PointXYZ> newPointCloud;
        //medianFilter.filter(newPointCloud);
        outlierRemoval.filter(newPointCloud);

        return newPointCloud;
    }

    //finds the closest point to the origin
    pcl::PointXYZ findClosestPoint(const pcl::PointCloud<pcl::PointXYZ>& pc){

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
        cropBox.setMax(Eigen::Vector4f(0.5, 0.5, 2.0, 1.0));
        cropBox.setKeepOrganized(true);

        fineGrid = pcl::ApproximateVoxelGrid<pcl::PointXYZ>();
        fineGrid.setLeafSize(0.05, 0.05, 0.02);

        coarseGrid = pcl::ApproximateVoxelGrid<pcl::PointXYZ>();
        coarseGrid.setLeafSize(0.1, 0.1, 0.02);

//        medianFilter = pcl::MedianFilter<pcl::PointXYZ>();
        outlierRemoval = pcl::StatisticalOutlierRemoval<pcl::PointXYZ>();
        outlierRemoval.setMeanK(50);
        outlierRemoval.setStddevMulThresh(0.5);



        return;
    }

    //callback function to receive messages
    void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){

        pcl::fromROSMsg(*msg, pointCloud);

        return;
    }


    //the main function that detects the hand (or object) in the pointcloud
    void detectHand(){

//        ROS_INFO("inputdata organized: %d", pointCloud.isOrganized());
        pcl::PointCloud<pcl::PointXYZ> croppedPointCloud = maskPointCloud(pointCloud);
//        ROS_INFO("cropped data organized: %d", croppedPointCloud.isOrganized());
        pcl::PointCloud<pcl::PointXYZ> fineGridPointCloud = sampleFineGrid(croppedPointCloud);
//        ROS_INFO("fine grid cloud organized: %d", fineGridPointCloud.isOrganized());
        pcl::PointCloud<pcl::PointXYZ> cleanedPointCloud = removeoutliers(fineGridPointCloud);
//        ROS_INFO("cleaned data organized: %d", cleanedPointCloud.isOrganized());
        pcl::PointCloud<pcl::PointXYZ> coarseGridPointCloud = sampleCoarseGrid(cleanedPointCloud);
//        ROS_INFO("coarse grid cloud organized: %d", coarseGridPointCloud.isOrganized());



        pcl::PointXYZ closestPoint = findClosestPoint(coarseGridPointCloud);

        sensor_msgs::PointCloud2 dsCloudMsg;
        pcl::toROSMsg(coarseGridPointCloud, dsCloudMsg);
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

    ros::Rate loop_rate(10);

    while(ros::ok()){

        detector.detectHand();

        ros::spinOnce();

        loop_rate.sleep();

    }
    return 0;
}
