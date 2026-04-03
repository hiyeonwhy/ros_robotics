#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>

class ScanToCloud
{
public:
    ScanToCloud()
    {
        scan_sub_ = nh_.subscribe("/scan", 10, &ScanToCloud::scanCallback, this);
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud", 10);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
    {
        sensor_msgs::PointCloud2 cloud_msg;
        projector_.projectLaser(*scan_msg, cloud_msg);
        cloud_pub_.publish(cloud_msg);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    ros::Publisher cloud_pub_;
    laser_geometry::LaserProjection projector_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scan_to_cloud_node");
    ScanToCloud node;
    ros::spin();
    return 0;
}

