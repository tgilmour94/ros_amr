#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Imu.h>

ros::Publisher imu_pub;

void imuCallback(const std_msgs::Float32MultiArray& msg){
 ros::Time current_time = ros::Time::now();

 sensor_msgs::Imu imu;
 imu.header.stamp = current_time;
 imu.header.frame_id = "base_link";
 imu.orientation.x = msg.data[0];
 imu.orientation.y = msg.data[1];
 imu.orientation.z = msg.data[2];
 imu.orientation.w = msg.data[3];
 imu.angular_velocity.x = msg.data[4];
 imu.angular_velocity.y = msg.data[5];
 imu.angular_velocity.z = msg.data[6];
 imu.linear_acceleration.x = msg.data[7];
 imu.linear_acceleration.y = msg.data[8];
 imu.linear_acceleration.z = msg.data[9];

 imu_pub.publish(imu);
}

int main(int argc, char** argv){
 ros::init(argc, argv, "odometry_publisher");
 ros::NodeHandle nh; 
 imu_pub = nh.advertise<sensor_msgs::Imu>("imu_data", 50);
 ros::Subscriber imu_sub = nh.subscribe("imupub",10,imuCallback);

 ros::spin();
 return 0;
}
