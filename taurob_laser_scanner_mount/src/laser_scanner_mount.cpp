#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "taurob_laser_scanner_mount/LSMStatusMsgConverter.h"
#include <libtaurob_tools/SocketUDPBinary.h>

#define EMBEDDED_CLOCK_DRIFT_AVG_ELEMENTS 4
#define WATCHDOG_MAX_TIME 400 	// ms

using namespace std_msgs;
typedef boost::posix_time::ptime tTime;
typedef boost::posix_time::time_duration tTimeDuration;


bool latest_command_frame_set = false;
bool first_frame_received = false; 

ros::Publisher pub_status_joint_state;
void Frame_received(unsigned char* ecu_msg_data);

struct timespec receive_time;
double embedded_clock_drift_us;
double cumulated_clock_drift_us;
            
ros::Time embedded_epoch;

std::string host_ip;
int host_port;
std::string laser_joint_name;
std::string joint_state_topic_name;
std::string speed_cmd_topic;

double angular_offset;
double velocity_offset;
int embedded_epoch_error;
int listen_port;
unsigned char latest_received_seq;
tTime latest_watchdog_time;
bool sending_thread_running = false;

uint16_t laser_scanner_speed = 0;


void speed_cmd_callback(const std_msgs::Float64::ConstPtr& msg)
{
	// convert rotation rate from rad to raw value for actor
		
	// rad/sec -> u/sec
	double u_sec = msg->data / (2*M_PI);
	
	// gear ratio is 76; 1 u/sec is 76*60=4560 in raw value
	laser_scanner_speed = (uint16_t)(u_sec * 4560.0);
	
	ROS_INFO("[Laser_scanner_mount] Received new laser rotation speed: %f, which is %d as raw value", msg->data, laser_scanner_speed);
}
	
void watchdog_feed_callback(const std_msgs::Bool::ConstPtr& msg)
{
	latest_watchdog_time = boost::posix_time::microsec_clock::local_time();
}

bool Watchdog_ok()
{
	bool ret = false;
	
	tTimeDuration dt = boost::posix_time::microsec_clock::local_time() - latest_watchdog_time;
	if (dt.total_milliseconds() <= WATCHDOG_MAX_TIME) ret = true;
			
	return ret;
}


void* Sending_thread_fn(void *ptr)
{
	SocketUDPBinary ecu_socket(host_ip, host_port, 0);
	
	ROS_INFO("[Laser_scanner_mount] Starting to send rotation speed data to laser scanner mount");
	ros::Duration sending_interval(0.1); 	// 100ms	
	
	while (ros::ok() && sending_thread_running)
	{
		unsigned char sendbuffer[2];
		sendbuffer[0] = ((laser_scanner_speed & 0xFF00) >> 8);
		sendbuffer[1] =  (laser_scanner_speed & 0x00FF);
		
		if (Watchdog_ok()) 
			ecu_socket.send(sendbuffer, 2);
		sending_interval.sleep();
	}
	
	return 0;
}

void Receive_loop_fn() 
{
	ROS_DEBUG("Setting up socket for receiving");
	
	// implementation with receive-only
	char message[1024];
	int sock;
	struct sockaddr_in name;
	int bytes;
	
	sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (sock < 0)   {
		ROS_ERROR("Unable to open socket -- exiting (if port < 1000, you need admin privileges)");
	}
	else
	{
		ROS_DEBUG("setting receive timeout");
		struct timeval tv;
		tv.tv_sec = 0;
		tv.tv_usec = 100000;
		if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0)
		{
			ROS_ERROR("Unable to set socket timeout -- exiting");
			exit(1);
		}
		
		ROS_DEBUG("binding socket to port %d", listen_port);
		bzero((char *) &name, sizeof(name));
		name.sin_family = AF_INET;
		name.sin_addr.s_addr = htonl(INADDR_ANY);
		name.sin_port = htons(listen_port);

		if (bind(sock, (struct sockaddr *) &name, sizeof(name))) {
			ROS_ERROR("Unable to bind to socket -- exiting (if port < 1000, you need admin privileges)");
			close(sock);
			exit(1);
		}

		ROS_DEBUG("Socket has port number #%d\n", ntohs(name.sin_port));

		ROS_INFO("[Laser_scanner_mount] Ready to receive angular data from laser scanner mount");
		while (ros::ok())
		{
			bytes = read(sock, message, 1024);
			if (bytes >= LSM_STATUS_FRAME_LENGTH)
			{
				unsigned char msg_ary[LSM_STATUS_FRAME_LENGTH];
				for (int i=0; i < LSM_STATUS_FRAME_LENGTH; i++)
				{
					msg_ary[i] = message[i];
				}
				Frame_received(msg_ary);
			}
			ros::spinOnce();
		}

		close(sock);
	}
}


void Get_parameters()
{        
	ros::NodeHandle local_nh = ros::NodeHandle(ros::this_node::getName());
	
	local_nh.param<int>("listen_port", listen_port, 8062);
	ROS_DEBUG("Listen port: %d", listen_port);
	
	local_nh.param<std::string>("host_ip", host_ip, "10.0.0.61");
	ROS_DEBUG("Host IP: %s", host_ip.c_str());
	
	local_nh.param<int>("host_port", host_port, 8064);
	ROS_DEBUG("Host port: %d", host_port);
	
	local_nh.param<double>("angular_offset", angular_offset, 0);
	ROS_DEBUG("Angular offset: %f", angular_offset);
	
	local_nh.param<double>("velocity_offset", velocity_offset, 0);
	ROS_DEBUG("Velocity offset: %f", velocity_offset);
			
	local_nh.param<int>("embedded_epoch_error_us", embedded_epoch_error, 0);
	ROS_DEBUG("Embedded epoch error: %d", embedded_epoch_error);
	
	local_nh.param<std::string>("joint_state_topic", joint_state_topic_name, "front_lidar/joint_state");
	ROS_DEBUG("LSM Status Joint State Topic: %s", joint_state_topic_name.data());           
	
	local_nh.param<std::string>("laser_joint_name", laser_joint_name, "front_spinning_lidar_spin_joint");
	ROS_DEBUG("Laser Joint Name: %s", laser_joint_name.data());
	
	local_nh.param<std::string>("speed_cmd_topic", speed_cmd_topic, "rotation_speed");
	ROS_DEBUG("Rotation speed topic: %s", speed_cmd_topic.c_str());
}


void Frame_received(unsigned char* ecu_msg_data)
{
	struct timespec old_receive_time = receive_time;
	clock_gettime(CLOCK_MONOTONIC, &receive_time);
	
	int second_difference = old_receive_time.tv_sec - receive_time.tv_sec;
	long us_diff = second_difference * (long)1E6 + ((double)(receive_time.tv_nsec - old_receive_time.tv_nsec) / 1E3);
	
	// 20ms (=20000us) is the expected interval. see what the error is.
	us_diff -= 20000;
	if (us_diff < 1000000)
	{
		// calculate moving average for embedded clock drift
		embedded_clock_drift_us -= (embedded_clock_drift_us / (double)EMBEDDED_CLOCK_DRIFT_AVG_ELEMENTS);
		embedded_clock_drift_us += ((double)us_diff / (double)EMBEDDED_CLOCK_DRIFT_AVG_ELEMENTS);
		
		// we need to cumulate the drift to fix the timestamp
		cumulated_clock_drift_us += embedded_clock_drift_us;
		
		ROS_DEBUG("measured current clock drift of %fus. Cumulated drift is %fus.", embedded_clock_drift_us, cumulated_clock_drift_us);
	}
	
	LSMStatusFrame status_frame = LSMStatusMsgConverter::ConvertToLSMStatusMsg(ecu_msg_data, &latest_received_seq);
	
	// received timestamp is in ms, we need sec and ns for ros::duration
	long timestamp_sec = status_frame.timestamp / 1000; 	// get the seconds part, integer division intended
	long timestamp_ns = (status_frame.timestamp % 1000) * (long)1E6; 
	
	if (first_frame_received == false)
	{
		ros::Time now = ros::Time::now();
		ROS_INFO("[Laser_scanner_mount] received first frame, time is %d, %d", now.sec, now.nsec);
		ROS_INFO("[Laser_scanner_mount] received timestamp is %d", status_frame.timestamp);
		ROS_INFO("[Laser_scanner_mount] timestamp_sec: %d, timestamp_ns: %d", (int)timestamp_sec, (int)timestamp_ns);
		first_frame_received = true;
		
		embedded_epoch = now - ros::Duration(timestamp_sec, (timestamp_ns + embedded_epoch_error*1000)); 	// calculate embedded epoch
		ROS_INFO("[Laser_scanner_mount] embedded epoch is %d, %d", embedded_epoch.sec, embedded_epoch.nsec);
	}
	
	// convert timestamp from embedded to ros epoch
	//ros::Time ros_timestamp = embedded_epoch + ros::Duration(timestamp_sec, timestamp_ns);
	ros::Time ros_timestamp = ros::Time::now();
	ros_timestamp -= ros::Duration(0, (embedded_epoch_error*1000));	
	//ROS_INFO("calculated timestamp is %d, %d", ros_timestamp.sec, ros_timestamp.nsec);
	
	// build joint state
	sensor_msgs::JointState jsm;
	
	jsm.header.frame_id = laser_joint_name;
	jsm.header.stamp = ros_timestamp;
	jsm.name.push_back(laser_joint_name);
	jsm.position.push_back(status_frame.current_angle + angular_offset);
	
	// status_frame.current_speed is in U/min of the motor shaft. we need to convert it to rad/s of the laser scanner for ROS.
	double current_speed_si = (double)status_frame.current_speed * ((2 * M_PI) / (76.0 * 60.0));	// 76 = gear ratio
	jsm.velocity.push_back(current_speed_si + velocity_offset);
	
	//ROS_INFO("Received LSM Status Message, angle %.1f", status_frame.current_angle);
	
	pub_status_joint_state.publish(jsm);		
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "taurob_laser_scanner_mount");
	ros::NodeHandle nh;
	Get_parameters();	
	pub_status_joint_state = nh.advertise<sensor_msgs::JointState>(joint_state_topic_name, 1);
	ros::Subscriber sub_watchdog = nh.subscribe("/watchdog_feed", 1, &watchdog_feed_callback);
	ros::Subscriber sub_rotationrate = nh.subscribe(speed_cmd_topic, 1, &speed_cmd_callback);
	
	pthread_t sending_thread;
	sending_thread_running = true;	
	if (pthread_create(&sending_thread, 0, Sending_thread_fn, 0)) 
	{
		fprintf(stderr, "Error creating controller thread\n");
		return 1;
	}

	Receive_loop_fn();
		
	return 0;
}
