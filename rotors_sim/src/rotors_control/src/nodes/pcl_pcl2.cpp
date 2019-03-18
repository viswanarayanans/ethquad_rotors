#include "sensor_msgs/point_cloud_conversion.h"
#include "ros/ros.h"

#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"

class Pcl_Pcl2
{
private:
	ros::NodeHandle n;
	ros::Publisher pub;
	ros::Subscriber sub;

public:
	Pcl_Pcl2()
	{
		pub = n.advertise<sensor_msgs::PointCloud2>("pcl2_cloud",1);
		sub = n.subscribe("/scan", 1000, &Pcl_Pcl2::callback, this);
	}
	void callback(const sensor_msgs::PointCloud& input)
	{
		sensor_msgs::PointCloud2 output;
		sensor_msgs::convertPointCloudToPointCloud2(input, output);
		pub.publish(output);
	}


};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pcl_pcl2");
	ros::NodeHandle n;
	ROS_INFO("Pcl_Pcl2 Initiated");
	Pcl_Pcl2 pcl;
	ros::spin();
}