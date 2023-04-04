### Expected vs. actual behavior
<!-- Tell us what you expected to happen and what happens instead.
     Please make sure to clearly distinguish between observation and interpretation.
     Do not forget any other comments, relevant logs, error output, screenshots, etc. -->


### Reproduction steps
<!-- How can the bug be reproduced?
     Launch file, example code, command sequence, etc. -->

**Happens**:
<!-- On which system does the bug appear?
     Use the following icons:
     YES:     :heavy_check_mark:
     NO:      :heavy_multiplication_x:
     UNKNOWN: :grey_question:          -->
:grey_question: On the robot :robot: <br>
:grey_question: In simulation :computer: <br>
:grey_question: When replaying a dataset :vhs:

**Installation**:
<!-- How did you install the software?
     Use the following icons:
     YES: :heavy_check_mark:
     NO:  :heavy_multiplication_x: -->
:heavy_multiplication_x: From Source - <!-- Insert commit hash here. --> <br>
:heavy_multiplication_x: Debian - <!-- Insert "nightly" or "release" here. -->

<!-- For graphics related issues, insert the output of `apt-cache show xserver-xorg | grep Version && sudo lshw -c video | grep product && apt list --installed | grep nvidia-driver` here. -->

### Checklist
<!-- If relevant, please confirm that you tried the following options:
     Use the following icons:
     YES: :heavy_check_mark:
     NO:  :heavy_multiplication_x: -->
:heavy_multiplication_x: I double-checked the following options:
* I am building with `-DCMAKE_BUILD_TYPE=Release`.
* I used the `--force-cmake` option of `catkin build`.
* I am using the `any` ROS mirror.
* I ran `apt update && apt upgrade`.

<!-- Please provide a time estimate how long it will take to fix the bug. -->
/estimate 0d 0h 0min

<!-- These labels will be added automatically when creating the issue. -->
/label T::Fix
