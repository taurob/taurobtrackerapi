#include <libtaurob_flipper/libtaurob_flipper.h>

Flipper::Flipper(Flipper_config configuration, bool control_enabled_initially) : on_receive_callback(0)
{
	for (int i = 0; i < configuration.joint_ips.size(); i++)
	{
		Arm_segment* seg = new Arm_segment(configuration.joint_names[i], 
										   configuration.joint_ips[i], 
										   configuration.joint_ports[i], 
										   i, 
										   control_enabled_initially,
										   MAX_ANGLE_VALUE,
										   UI_SERVER_BASE_PORT);
		seg->Set_on_received_callback(this);
		segments.push_back(seg);
	}
}

Flipper::~Flipper()
{
	for (int i = 0; i < segments.size(); i++)
	{
		delete segments[i];
	}
}

bool Flipper::Is_uptodate()
{
	bool ret = true;	
	for (int i = 0; i < segments.size(); i++)
	{
		if (segments[i]->Is_uptodate() == false)
		{
			ret = false;
			break;
		}
	} 	
	return ret;
}

bool Flipper::Watchdog_ok()
{
	bool ret = true;	
	for (int i = 0; i < segments.size(); i++)
	{
		if (segments[i]->Watchdog_ok() == false)
		{
			ret = false;
			break;
		}
	}
	return ret;
}

void Flipper::On_segment_receive(int segment_nr)
{
	if (on_receive_callback != 0)
	{
		on_receive_callback(segment_nr);
	}
}

void Flipper::On_friction_clutch_slipped(int segment_nr)
{
	// empty because friction clutch is irrelevant for flippers
}

int Flipper::Get_segment_count()
{
	return segments.size();
}

void Flipper::Run()
{
	for (int i = 0; i < segments.size(); i++)
	{
		segments[i]->Run();
	}
}

void Flipper::Stop()
{
	for (int i = 0; i < segments.size(); i++)
	{
		segments[i]->Stop();
	}
}
   
void Flipper::Feed_watchdog()
{
	for (int i = 0; i < segments.size(); i++)
	{
		segments[i]->Feed_watchdog();
	}
}

void Flipper::Set_on_received_callback(void (*callback)(int segment_nr))
{
	on_receive_callback = callback;
}

void Flipper::Set_watchdog_enabled(bool state)
{
	for (int i = 0; i < segments.size(); i++)
	{
		segments[i]->Set_watchdog_enabled(state);
	}
}

void Flipper::Set_pause_sending(bool pause)
{
	for (int i = 0; i < segments.size(); i++)
	{
		segments[i]->Set_pause_sending(pause);
	}
}

std::string Flipper::Get_segment_name(int segment)
{
	std::string ret;
	if (segment < segments.size())
	{
		ret = segments[segment]->Get_name();
	}
	return ret;
}

// returns rad, range [-pi..+pi]
float Flipper::Get_position(int segment)
{
	float ret = 0;
	if (segment < segments.size())
	{
		ret = segments[segment]->Get_position();
		ret = 2*M_PI - ret; 	// convert turning direction
		ret /= FLIPPER_SETPOINT_FACTOR;
		
		// value should now be in [0..2pi] but if it's not (maybe due to FLIPPER_SETPOINT_FACTOR - see Set_position)
		// add 2*PI to avoid negative values
		if (ret < 0) ret += 2*M_PI;

		ret -= M_PI; 	// convert from range [0..2pi] to [-pi..+pi]
	}
	
	return ret;
}

float Flipper::Get_current(int segment)
{
	float ret = 0;
	if (segment < segments.size())
	{
		ret = segments[segment]->Get_current();
	}
	return ret;
}

float Flipper::Get_temperature(int segment)
{
	float ret = 0;
	if (segment < segments.size())
	{
		ret = segments[segment]->Get_temperature();
	}
	return ret;
}
 
unsigned char Flipper::Get_error_code(int segment)
{
	unsigned char ret = 0;
	if (segment < segments.size())
	{
		ret = segments[segment]->Get_error_code();
	}
	return ret;
}

unsigned char Flipper::Get_bitfield(int segment)
{
	float ret = 0;
	if (segment < segments.size())
	{
		ret = segments[segment]->Get_bitfield();
	}
	return ret;
}

// expects rad [-pi..+pi]
void Flipper::Set_position(int segment, float pos)
{
	if (segment < segments.size())
	{
		pos += M_PI; 	// map from range [-pi..+pi] to [0..2pi]
		pos *= FLIPPER_SETPOINT_FACTOR;
		pos = 2*M_PI - pos; 	// convert turning direction
		
		// take care that the new value is still in range [0..2pi]
		// if it's negative, add (2*PI * FLIPPER_SETPOINT_FACTOR)
		if (pos < 0)
		{
			pos += (2*M_PI * FLIPPER_SETPOINT_FACTOR);
		}
		segments[segment]->Set_position(pos);
	}
}
