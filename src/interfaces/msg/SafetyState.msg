# Safety watchdog state message
uint8 FATAL=0
uint8 CRITICAL=1
uint8 WARNING=2
uint8 NORMAL=3

uint8 level
string detail
bool motor_relay_on
float32 battery_voltage
