# This is a message that holds additional data for each arm_joint
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.

Header header

string[] name
float64[] temperature
float64[] voltage
bool[] dig_in_1_state
bool[] dig_in_2_state
