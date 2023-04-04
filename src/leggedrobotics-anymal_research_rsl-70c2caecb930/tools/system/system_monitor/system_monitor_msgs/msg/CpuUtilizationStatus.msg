# CPU core utilization status.

# Statistics.
float32 usage   # [-], Usage [0.0, 1.0].
float32 user    # [-], CPU utilization that occurred while executing at the user level.
float32 nice    # [-], CPU utilization that occurred while executing at the user level with nice priority.
float32 sys     # [-], CPU utilization that occurred while executing at the system level (kernel). Note that this does not include time spent servicing hardware and software interrupts.
float32 iowait  # [-], Fraction of time that the CPU or CPUs were idle during which the system had an outstanding disk I/O request.
float32 irq     # [-], Fraction of time spent by the CPU or CPUs to service hardware interrupts.
float32 soft    # [-], Fraction of time spent by the CPU or CPUs to service software interrupts.
float32 steal   # [-], Fraction of time spent in involuntary wait by the virtual CPU or CPUs while the hypervisor was servicing another virtual processor.
float32 guest   # [-], Fraction of time spent by the CPU or CPUs to run a virtual processor.
float32 gnice   # [-], Fraction of time spent by the CPU or CPUs to run a niced guest.
float32 idle    # [-], Fraction of time that the CPU or CPUs were idle and the system did not have an outstanding disk I/O request.
