#include <stdint.h>
#include <signal.h>
#include <ros/ros.h>
#include <libtaurob_tools/SocketTCP.h>
#include <argo_opmode_publisher/ArgoOpmode.h>
#include <argo_opmode_publisher/RequestOpmode.h>
#include <argo_event_interaction/EventInteractionReport.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <flexbe_msgs/BehaviorLog.h>
#include <flexbe_msgs/OutcomeRequest.h>

using namespace argo_opmode_publisher;

#define DEFAULT_IP_ADDRESS "172.16.20.108"
#define DEFAULT_PORT 9044

#define LOW_VOLTAGE_COUNT_THRESHOLD 40 		// we receive voltage with ~40 Hz, so 40 means ~1sec
#define LOW_VOLTAGE_THRESHOLD_NO_DRIVING 23.0f 	// note that these are higher than in base node because we want the blinking to start early
#define LOW_VOLTAGE_THRESHOLD_WHILE_DRIVING 21.0f

uint8_t opmode = argo_opmode_publisher::ArgoOpmode::SAFE;
std::string leds_host;
int leds_port;

enum Argo_event 	 	// these must be mutually exclusive for now
{ 
	NONE,
	DOCKED, 
	LOW_BATTERY, 
	WAITING_FOR_OPERATOR,
	GPA_DETECTED,
	COMS_LOST
};

Argo_event current_event = NONE;
bool voltage_low = false;
bool robot_docked = false;
bool operator_required = false;
bool currently_driving = false;
bool gpa_detected = false;
bool coms_lost = false;

int low_voltage_count;
SocketTCP *tsocket = NULL;

void update_events()
{
	Argo_event new_event;
	if (gpa_detected)
	{
		new_event = GPA_DETECTED;
	}
	else if (coms_lost)
	{
		new_event = COMS_LOST;
	}
	else if (voltage_low)
	{
		new_event = LOW_BATTERY;
	}
	else if (robot_docked)
	{
		new_event = DOCKED;
	}
	else if (operator_required)
	{
		new_event = WAITING_FOR_OPERATOR;
	}
	else
	{
		new_event = NONE;
	}

	if (current_event != new_event)
	{
		ROS_INFO("New event: %d", new_event);
		current_event = new_event;
	}
}

void twist_callback(const geometry_msgs::TwistConstPtr& msg)
{
	if (msg->linear.x > 0 || msg->angular.z > 0)
	{
		currently_driving = true;
	}
	else
	{
		currently_driving = false;
	}
}

void supply_voltage_callback(const std_msgs::Float32ConstPtr& msg)
{
	float threshold = (currently_driving ? LOW_VOLTAGE_THRESHOLD_WHILE_DRIVING :
											LOW_VOLTAGE_THRESHOLD_NO_DRIVING);
											
	if (msg->data < threshold && low_voltage_count < 2*LOW_VOLTAGE_COUNT_THRESHOLD)
	{
		++low_voltage_count;
	}
	else if (low_voltage_count > 0)
	{
		--low_voltage_count;
	}
	
	if (low_voltage_count > LOW_VOLTAGE_COUNT_THRESHOLD)
	{
		voltage_low = true;
	}
	else
	{
		voltage_low = false;
	}
	
	update_events();
}

void robot_docked_callback(const std_msgs::BoolConstPtr& msg)
{
	if (msg->data) ROS_INFO("Received docked event TRUE");
	else ROS_INFO("Received docked event FALSE");
	robot_docked = msg->data;
	update_events();
}

void robot_opmode_callback(const argo_opmode_publisher::ArgoOpmodeConstPtr& msg)
{
	ROS_INFO("Received new opmode");
	opmode = msg->opmode;
}

void gpa_callback(const std_msgs::BoolConstPtr& msg)
{
	gpa_detected = msg->data;
	update_events();
}

void report_event_callback(const argo_event_interaction::EventInteractionReportConstPtr& msg)
{
	operator_required = true;
	update_events();
}

void event_reaction_callback(const std_msgs::UInt8ConstPtr& msg)
{
	operator_required = false;
	update_events();
}

void coms_lost_callback(const std_msgs::BoolConstPtr& msg)
{
	coms_lost = msg->data;
	update_events();
}
   
void behav_log_callback(const flexbe_msgs::BehaviorLogConstPtr& msg)
{
	if (msg->status_code == flexbe_msgs::BehaviorLog::HINT)
	{
		operator_required = false;
		update_events();
	}
}

void outcome_request_callback(const flexbe_msgs::OutcomeRequestConstPtr& msg)
{
	operator_required = true;
	update_events();
}

void term(int signum)
{
    if (tsocket == NULL)
    {
		tsocket = new SocketTCP(leds_host, leds_port);		
	}
	
	tsocket->send("R:0 G:0 B:0\r\n");
//	tsocket->dispose();
//	delete tsocket;
}

int main(int argc, char** argv)
{
	struct sigaction action;
    memset(&action, 0, sizeof(struct sigaction));
    action.sa_handler = term;
    sigaction(SIGTERM, &action, NULL);
    
	ros::init(argc, argv, "taurob_leds");
	ros::NodeHandle nh("~");
	
	ROS_INFO("starting led node");
	
	// get params and init subscriber	
	nh.param<std::string>("ip_address", leds_host, DEFAULT_IP_ADDRESS);
	nh.param<int>("port", leds_port, DEFAULT_PORT);	
	ros::Subscriber sub_robot_opmode = nh.subscribe("/robot_opmode", 1, robot_opmode_callback);
	ros::Subscriber sub_supply_voltage = nh.subscribe("/supply_voltage", 1, supply_voltage_callback);
	ros::Subscriber sub_docked = nh.subscribe("/robot_docked", 1, robot_docked_callback);
	ros::Subscriber sub_twist = nh.subscribe("/cmd_vel", 1, twist_callback); 	// so we know if we're currently driving
	ros::Subscriber sub_gpa = nh.subscribe("/gpa_alert", 1, gpa_callback);
	ros::Subscriber sub_report_event = nh.subscribe("/report_event", 1, report_event_callback);
	ros::Subscriber sub_event_reaction = nh.subscribe("/event_reaction", 1, event_reaction_callback);		
	ros::Subscriber sub_offline = nh.subscribe("/communication_lost", 1, coms_lost_callback);
	ros::Subscriber sub_outcome_request = nh.subscribe("/flexbe/outcome_request", 1, outcome_request_callback);
	ros::Subscriber sub_behav_log = nh.subscribe("/flexbe/log", 1, behav_log_callback);
	
	ros::ServiceClient client = nh.serviceClient<argo_opmode_publisher::RequestOpmode>("/opmode_publisher/request_opmode");
	argo_opmode_publisher::RequestOpmode srv;
	if (client.call(srv))
	{
		opmode = srv.response.opmode;
	}
	
	ros::Rate loop_rate(4); 	// 4 Hz = 250ms cycle time
	uint count;
	
	tsocket = new SocketTCP(leds_host, leds_port);
	
	while (ros::ok())
	{
		// create packet to send to led strip
		std::stringstream ss;
		int r, g, b;
		
		switch (opmode)
		{
			case ArgoOpmode::SAFE: 	// green
				r = 0;
				g = 255;
				b = 0;
				break;
			
			case ArgoOpmode::MANUAL: 	// blue
				r = 0;
				g = 0;
				b = 255;
				break;
				
			case ArgoOpmode::AUTONOMOUS: 	// yellow
				r = 255;
				g = 192;
				b = 0;
				break;
							
			default: 	// unknown mode -- dark purple
				r = 127;
				g = 0;
				b = 127;
				break;
		}
			
		// if the count is between 0 and 2, and we have an active event, 
		// then we don't show the mode color but the event color instead.
		if (count < 3) 	// eq. 750 ms
		{
			switch (current_event)
			{
				case NONE: 		// keep displaying mode color
					break;
					
				case LOW_BATTERY: 	// red
					r = 255;
					g = 0;
					b = 0;
					break;
					
				case DOCKED:	// white
					r = 255;
					g = 255;
					b = 255;
					break;
				
				case WAITING_FOR_OPERATOR: 	// blue -- note: same color as manual mode - this is intentional
					r = 0;
					g = 0;
					b = 255;
					break;
					
				case GPA_DETECTED: 	// purple
					r = 255;
					g = 0;
					b = 127;
					break;
					
				case COMS_LOST: 	// cyan
					r = 0;
					g = 255;
					b = 192;
					break;
				
				default: 	// unknown event -- dark purple
					r = 127;
					g = 64;
					b = 127;
					break;
			}
		}
		else if (current_event == GPA_DETECTED || current_event == COMS_LOST)
		{
			// use double-flash with black in between to signal GPA detected
			if (count == 3 || count == 4 || count > 6)
			{
				r = 0;
				g = 0;
				b = 0;
			}
		}
				
		ss << "R:" << r << " G:" << g << " B:" << b << "\r\n";
		
		if (tsocket != 0 && tsocket->send(ss.str()) == false) 	// if sending fails, try to re-open the connection
		{
			ROS_WARN("sending to LEDs failed. trying to re-build connection. disposing old socket...");
			tsocket->dispose();
			ROS_INFO("deleting old socket");
			//delete tsocket;
			//tsocket = 0;
			ROS_INFO("instantiating new socket");
			tsocket = new SocketTCP(leds_host, leds_port);
			ROS_INFO("re-initialization completed.");
		}
		
		ros::spinOnce();
		
		loop_rate.sleep();
		++count;
		if (count >= 10) count = 0;
	}

	term(SIGTERM);
	
	return 0;
}
