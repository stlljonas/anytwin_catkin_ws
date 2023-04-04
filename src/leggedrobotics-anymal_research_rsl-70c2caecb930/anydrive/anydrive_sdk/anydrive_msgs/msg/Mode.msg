# Mode.
uint16 mode
uint16 NA                                  = 0  # Unitialized or not available
uint16 Freeze                              = 1  # Freeze motor
uint16 Disable                             = 2  # Disable motor
uint16 Current                             = 3  # Track current
uint16 MotorPosition                       = 4  # Track motor position
uint16 MotorVelocity                       = 5  # Track motor velocity
uint16 GearPosition                        = 6  # Track gear position
uint16 GearVelocity                        = 7  # Track gear velocity
uint16 JointPosition                       = 8  # Track joint position
uint16 JointVelocity                       = 9  # Track joint velocity
uint16 JointTorque                         = 10 # Track joint torque
uint16 JointPositionVelocity               = 11 # Track joint position with feedforward velocity
uint16 JointPositionVelocityTorque         = 12 # Track joint position with feedforward velocity and torque
uint16 JointPositionVelocityTorquePidGains = 13 # Track joint position with feedforward velocity and torque using custom joint position gains
uint16 DemoChangeGravity                   = 14 # Demo mode: Compensate or change gravity. Do not use this.
uint16 DemoSafeJointVelocity               = 15 # Demo mode: Joint velocity control with torque supervision. Do not use this.
