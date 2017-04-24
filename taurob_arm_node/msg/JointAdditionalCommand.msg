# This is a message that provides additional commands for each arm_joint
#
# Each joint is uniquely identified by its name.
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.
#
# control_aux_port              if true the aux_values are used - can only be used by one joint per U-ECU!
# aux_five_v_power_enable       switches the 5V power supply output on if true
# aux_twelve_v_power_enable     switches the 12V power supply output on if true
# aux_dig_out_1_enable          switches the digital 1 output on/off
# aux_dig_out_2_enable          switches the digital 2 output on/off
# trigger_u_ecu_reset           allows to trigger a U-ECU reset - only possible once each minute

Header header

string[] name
bool[] control_aux_port
bool[] aux_five_v_power_enable
bool[] aux_twelve_v_power_enable
bool[] aux_dig_out_1_enable
bool[] aux_dig_out_2_enable
bool[] trigger_u_ecu_reset
