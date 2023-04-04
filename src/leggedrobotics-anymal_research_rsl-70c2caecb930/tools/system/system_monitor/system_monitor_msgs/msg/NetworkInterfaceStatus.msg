# Network interface status.

# Name.
string name

# Operation status.
NetworkInterfaceOperationStatus operation_status
StatusCheck operation_status_check

# Traffic.
float32 incoming_traffic        # [MB/s]
float32 outgoing_traffic        # [MB/s]
StatusCheck traffic_check

# Other statistics.
int32 mtu                       # [B]
float32 total_received_data     # [MB]
float32 total_transmitted_data  # [MB]
int32 num_collisions            # [-]
int32 num_reception_errors      # [-]
int32 num_transmission_errors   # [-]
