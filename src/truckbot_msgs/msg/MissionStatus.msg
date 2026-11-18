# mission runner progress, published every mission timer tick
uint8 IDLE=0
uint8 RUNNING=1
uint8 COMPLETE=2

builtin_interfaces/Time stamp
uint8 state
string mission_name
string state_name
bool waiting_for_goal
bool waiting_for_lift
