#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <thread>
#include <std_msgs/Bool.h>
char buffer[100];
ackermann_msgs::AckermannDriveStamped msg2;
int ULTRA_SIGNAL = 1;
int IMU_SIGNAL = 1;
float STEER = 0.0;
float SPEED = 0.0;

void pub(ros::Publisher safety_pub)
{
	while(1)
	{
		if(IMU_SIGNAL == 0)
		{
			msg2.drive.speed  = 0;
			msg2.drive.steering_angle = 0;
			safety_pub.publish(msg2);
		}
		else
		{
			if(ULTRA_SIGNAL == 0)
			{
				msg2.drive.speed = 0;
				msg2.drive.steering_angle = 0;
				safety_pub.publish(msg2);
			}
			else
			{
				if((STEER >-0.2)&&(STEER < 0.2))
				{
				}
				else //STEER != 0
				{
					
					msg2.drive.speed = 0.5;
					msg2.drive.steering_angle = STEER;
					safety_pub.publish(msg2);
					STEER = 0;
					SPEED = 0;

				}


			}

		}


	}


}


void ultraCallback(const std_msgs::Int32 msg){
	sprintf(buffer, "%d", msg.data);
	ROS_INFO(buffer);
	ros::NodeHandle nh;
        ros::Publisher safety_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("mux/ackermann_cmd_mux/input/safety", 2);
	ROS_INFO("Check the gap");
	ULTRA_SIGNAL = msg.data;

}

void naviCallback(const ackermann_msgs::AckermannDriveStamped msg){
	ROS_INFO("GG");
	STEER = msg.drive.steering_angle;
	SPEED = msg.drive.speed;
	ROS_INFO("STEER, SPEED %f, %f", STEER, SPEED);
}

void imuCallback(const std_msgs::Int32 msg){
	ROS_INFO("in imu callback()");
	if(msg.data == 0)
	{
		IMU_SIGNAL = 0;
	}
	ROS_INFO("imu_signal : %d", IMU_SIGNAL);
}

int main(int argc, char **argv){
	int count = 0;
        ros::init(argc, argv, "safety_node");
	ros::NodeHandle nh;
	ros::Publisher safety_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("mux/ackermann_cmd_mux/input/safety", 2);
	ros::Rate looprate(10);
	std::thread p;
	while(ros::ok())
	{
		ros::Subscriber navi_sub = nh.subscribe("mux/ackermann_cmd_mux/input/navigation", 2, naviCallback);
		ros::Subscriber safety_sub = nh.subscribe("safety_topic", 2, ultraCallback);
		ros::Subscriber imu_sub = nh.subscribe("imu_topic", 2, imuCallback);
		p = std::thread(&pub, safety_pub);
		ros::spin();
		looprate.sleep();
	}
        return 0;
}
         
