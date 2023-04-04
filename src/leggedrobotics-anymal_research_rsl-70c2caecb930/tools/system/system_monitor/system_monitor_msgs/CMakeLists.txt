cmake_minimum_required(VERSION 3.5.1)
project(system_monitor_msgs)

set(CATKIN_PACKAGE_DEPENDENCIES)

find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
    message_generation
)

add_message_files(
  FILES
    ChronyStatus.msg
    ChronySyncStatus.msg
    CpuCoreStatus.msg
    CpuStatus.msg
    CpuUtilizationStatus.msg
    DiskStatus.msg
    MemoryStatus.msg
    MemoryTypeStatus.msg
    MonitoringStatus.msg
    NetworkInterfaceStatus.msg
    NetworkInterfaceOperationStatus.msg
    NetworkStatus.msg
    SingleDiskStatus.msg
    StatusCheck.msg
)

generate_messages(
  DEPENDENCIES
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

catkin_package(
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
    message_runtime
)
