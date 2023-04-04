# CPU status.

# Time stamp of status.
time stamp

# Monitoring status.
MonitoringStatus monitoring_status

# Average utilization.
CpuUtilizationStatus utilization
StatusCheck usage_check

# List of CPU cores.
CpuCoreStatus[] cores

# Average loads for the last n minutes.
# (The number of jobs in the run queue (state R) or waiting for disk I/O (state D).)
float32 load_average_1min   # [-]
float32 load_average_5min   # [-]
float32 load_average_15min  # [-]
