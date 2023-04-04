# Main Board State

# States of the converters on the rpsm mainboard
# (ADS1015 for 5V, 12V, 15V, extern and LTS4051)
rpsm_msgs/CircuitState[] converter_states

# Bool equals to true if the E-Stop is engaged (pressed)
bool estop_engaged

# Finite state machine state
uint8 fsm_state

# Bool equal to true if the CAN-Power is enabled
bool can_power_enabled
