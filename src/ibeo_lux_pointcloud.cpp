//
// Created by zxkj on 17-12-30.
//
#include <ibeosdk/lux.hpp>
#include <ibeosdk/IpHelper.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <perception_msgs/Object.h>
#include <perception_msgs/Objects.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>

typedef pcl::PointXYZI PointType;
std::string frame_id_;
//using namespace ibeosdk;
const ibeosdk::Version::MajorVersion majorVersion(5);
const ibeosdk::Version::MinorVersion minorVersion(2);
const ibeosdk::Version::Revision revision(2);
const ibeosdk::Version::PatchLevel patchLevel;
const ibeosdk::Version::Build build;
const std::string info = "IbeoSdkLux";
ibeosdk::Version appVersion(majorVersion, minorVersion, revision, patchLevel, build, info);

//IbeoSDK ibeoSDK;
class AllLuxListener : public ibeosdk::DataListener<ibeosdk::ScanLux>,
                       public ibeosdk::DataListener<ibeosdk::ObjectListLux>
{
public:
    ros::NodeHandle nh;
    ros::Publisher pubLuxScan;
    ros::Publisher pubLuxObject;
    //object
    perception_msgs::Object lux_object_;
    perception_msgs::Objects lux_objects_;
    geometry_msgs::Point contour_point;
    //scan
    pcl::PointCloud<PointType> scan_array;
    PointType scan_point;
    sensor_msgs::PointCloud2 scan_array_msg;

    AllLuxListener()
    {
        pubLuxScan = nh.advertise<sensor_msgs::PointCloud2>("/lux_scan", 5);
        pubLuxObject = nh.advertise<perception_msgs::Objects>("/lux_object",5);
    }

    virtual ~AllLuxListener(){}

public:
    void onData(const ibeosdk::ScanLux* const scan)
    {
        float dis,angle;
        scan_array.clear();
        std::vector<ibeosdk::ScanPointLux> scanList = scan->getScanPoints();

        for(int i = 0; i < scanList.size(); ++i)
        {
            if(!(scanList.at(i).getFlags() & (ibeosdk::ScanPointLux::LSPF_Dirt | ibeosdk::ScanPointLux::LSPF_Ground | ibeosdk::ScanPointLux::LSPF_Transparent | ibeosdk::ScanPointLux::LSPF_Rain)))
            {
                dis = scanList.at(i).getDistance() / 100.0;   //米
                angle = scanList.at(i).getHorizontalAngle() / 32.0;   //度
                scan_point.x = dis * sin(3.1415926*angle/180.0);
                scan_point.y = -dis * cos(3.1415926*angle/180.0);
                scan_point.z = 0;
                scan_point.intensity = scanList.at(i).getLayer();
                scan_array.push_back(scan_point);
            }
        }
        pcl::toROSMsg(scan_array, scan_array_msg);
        scan_array_msg.header.frame_id = frame_id_;
        scan_array_msg.header.stamp = ros::Time::now();
        pubLuxScan.publish(scan_array_msg);
    }
    void onData(const ibeosdk::ObjectListLux* const objectList)
    {
        std::vector<ibeosdk::ObjectLux> object_points = objectList->getObjects();
        lux_objects_.objects.clear();

        for(int i = 0; i < object_points.size(); ++i)
        {
            //单位问题：原数据中，涉及到距离都是cm，速度是cm/s，角度都是1/32 deg
            lux_object_.id = object_points.at(i).getObjectId();
            lux_object_.age = object_points.at(i).getObjectAge();
            lux_object_.prediction_age = object_points.at(i).getPredictionAge();

            lux_object_.reference_point.x = object_points.at(i).getReferencePoint().getX() / 100.0;
            lux_object_.reference_point.y = object_points.at(i).getReferencePoint().getY() / 100.0;
            lux_object_.bounding_box_center.x = object_points.at(i).getBoundingBoxCenter().getX() / 100.0;
            lux_object_.bounding_box_center.y = object_points.at(i).getBoundingBoxCenter().getY() / 100.0;
            lux_object_.bounding_box_size.x = object_points.at(i).getBoundingBoxWidth() / 100.0;
            lux_object_.bounding_box_size.y = object_points.at(i).getBoundingBoxLength() / 100.0;
            lux_object_.object_box_center.x = object_points.at(i).getObjectBoxCenter().getX() / 100.0;
            lux_object_.object_box_center.y = object_points.at(i).getObjectBoxCenter().getY() / 100.0;
            lux_object_.object_box_size.x = object_points.at(i).getObjectBoxSizeX() / 100.0;
            lux_object_.object_box_size.y = object_points.at(i).getObjectBoxSizeY() / 100.0;
            lux_object_.object_box_orientation = object_points.at(i).getObjectBoxOrientation() / 32.0;
            //lux_object_.object_box_orientation_absolute
            lux_object_.absolute_velocity.x = object_points.at(i).getAbsoluteVelocity().getX() / 100.0;
            lux_object_.absolute_velocity.y = object_points.at(i).getAbsoluteVelocity().getY() / 100.0;
            lux_object_.absolute_velocity_sigma.x = object_points.at(i).getAbsoluteVelocitySigmaX() / 100.0;
            lux_object_.absolute_velocity_sigma.y = object_points.at(i).getAbsoluteVelocitySigmaY() / 100.0;
            lux_object_.relative_velocity.x = object_points.at(i).getRelativeVelocity().getX() / 100.0;
            lux_object_.relative_velocity.y = object_points.at(i).getRelativeVelocity().getY() / 100.0;
            //lux_object_.absolute_acceleration
            //lux_object_.relative_acceleration
            //lux_object_.absolute_yaw_rate
            //lux_object_.relative_yaw_rate
            lux_object_.classification = object_points.at(i).getClassification();
            lux_object_.classification_age = object_points.at(i).getClassificationAge();
            lux_object_.classification_certainty = object_points.at(i).getClassificationCertainty();

            for(int j = 0; j < object_points.at(i).getNumberOfContourPoints(); ++j)
            {
                contour_point.x = object_points.at(i).getContourPoints().at(j).getX() / 100.0;
                contour_point.y = object_points.at(i).getContourPoints().at(j).getY() / 100.0;
                //lux_object_.contour_points.push_back(contour_point);
            }
            lux_objects_.objects.push_back(lux_object_);
        }
        lux_objects_.second = objectList->getScanStartTimestamp();
        lux_objects_.header.frame_id = frame_id_;
        lux_objects_.header.stamp = ros::Time::now();
        pubLuxObject.publish(lux_objects_);
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ibeo_lux_pointcloud");
    ros::NodeHandle pnh("~");

    std::string device_ip;
    int port;
    pnh.param<std::string>("device_ip", device_ip, "192.168.1.51");
    pnh.param<int>("port", port, 12002);
    pnh.param<std::string>("frame_id", frame_id_, "/lux");

    std::cerr << " using IbeoSDK " << ibeosdk::IbeoSDK::getVersion().toString() << std::endl;

    const off_t magLogFileSize = 1000000;
    ibeosdk::LogFileManager logFileManager;
    ibeosdk::LogFile::setTargetFileSize(magLogFileSize);
    const ibeosdk::LogLevel ll = ibeosdk::logLevelFromString("Debug");
    ibeosdk::LogFile::setLogLevel(ll);
    logFileManager.start();

    AllLuxListener allLuxListener;
    ibeosdk::IbeoLux lux(device_ip, port);
    lux.setLogFileManager(&logFileManager);
    lux.registerListener(&allLuxListener);

    lux.getConnected();
    ros::Rate loop_rate(25);
    while(ros::ok())
    {
        if(!lux.isConnected())
        {
            ROS_INFO("!lux.isConnected\n");
            return -1;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;

}






