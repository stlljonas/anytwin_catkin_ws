# Chrony status.

# Time stamp of status.
time stamp

# Monitoring status.
MonitoringStatus monitoring_status

# Time sync status.
ChronySyncStatus time_sync_status

# Time offset status.
StatusCheck time_offset_check

# Statistics.
string reference_id
uint32 stratum              # [-]
string reference_time       # UTC
float32 system_time         # [s]
float32 last_offset         # [s]
float32 rms_offset          # [s]
float32 frequency           # [ppm]
float32 residual_frequency  # [ppm]
float32 skew                # [ppm]
float32 root_delay          # [s]
float32 root_dispersion     # [s]
float32 update_interval     # [s]
string leap_status
