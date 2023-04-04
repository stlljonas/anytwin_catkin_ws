## Shakeout

Version: 0.3.0

**Usage**

This template contains instructions on how to perform a shakeout test.

* Create a new issue from this template.
* Title: "[shakeout] YYYY-MM-DD / anymal-name"
* Assignees: Shakeout responsibles

If you come across an issue:

* Software: Create a [new GitLab issue](https://git.anybotics.com/anybotics/anybotics/issues/new) (use "Shakeout Bug Report" issue template) resp. comment on an existing issue that it re-appeared and link to this issue.
* Hardware: Create a [new journal entry](https://docs.google.com/forms/d/e/1FAIpQLSdy4ZsThXate4qNz7xMUlB7vnaKZnve5aUaX5ZvZbdfA4epuw/viewform).

### A. Preparations

1. Make yourself familiar with the [known software issues](https://git.anybotics.com/anybotics/anybotics/issues?scope=all&utf8=%E2%9C%93&state=opened&label_name[]=V%3A%3AShakeout). 
   
1. Prepare the OPC:

   1. Install the newest `nightly` Debian packages.
   1. Install the [ANYmal Setup Script](https://git.anybotics.com/snippets/2) if not done yet.

1. Prepare the robot:

   1. Hang the robot on the big crane.
   1. Attach the top shell and the inspection payload (use all screws).
   1. Insert the battery, attach the bottom shell, plug in the charger.
   1. Turn on the robot.
   1. Upgrade the software on all 3 on-board PCs:
      ```
      anymal_setup -a <anymal-name>
      anymal_ssh -c <pc-name>
      sudo apt update && sudo apt upgrade
      ```

1. Configure the software:

   1. Navigate to the configuration directory:
      ```
      anymal_ssh -c lpc
      cd /home/integration/.ros
      ```
   1. Make sure the following files contain the correct parameters (modify if necessary):
      1. `config.yaml`:
         ```
         load_anymal_description:
           custom_parameters:
             enable_inspection_payload: true
         ```
      1. `load_config_input.conf`:
         ```
         DEFAULT_DATA_PACKAGE=anybotics_data
         DEFAULT_WORLD=hagenholz
         ```
   1. Turn the robot off and on again.

1. Start up the user interface:

   1. Turn on the HBC
      1. Check robot status on HBC display. 
         * [ ] The battery level is displayed correctly.
         * [ ] The state estimator is displayed correctly.
         * [ ] The actuator states are displayed correctly.
   1. Plug the 3D mouse into the OPC.
      To initialize properly, make sure it lies on a flat surface while turning on.
   1. Start up the GUI on the OPC.
      ```
      anymal_setup -a <anymal-name> -c
      source /opt/ros/melodic/setup.bash
      roslaunch anymal_c opc.launch
      ```
      * [ ] No error appears.
   1. Switch to the `base` frame in RViz.
      
1. Start recording a video using the camcorder or your phone.
   Make sure the camera follows the robot's movements throughout the shakeout.
   
### B. Basics

1. Test the diagnostics system:

   * [ ] No diagnostics warnings or errors are present (under "Diagnostics").

### C. Locomotion

1. Test sending basic commands using the HBC:

   1. Release the p-stop.
   1. Command `Go Default` and  `Go Rest` (under "Services").
      * [ ] The final configurations look good.
   1. Go `default` and interrupt the controller with the p-stop.
      * [ ] The legs stop moving.
   1. Release the p-stop.

1. Test sending basic commands using the GUI:

   1. Run the `default` operation mode of the `joint_configurations` motion controller (under "Motion Control" / "Manual").
      * [ ] The command is executed.
   1. Lower the robot to the ground.
      * [ ] The state estimator turns green.
   1. Engage and release the p-stop.
      
1. Test the center of mass:

   1. Run the `test_com_with_square_up` operation mode of the `free_gait` motion controller (under "Motion Control" / "Manual").
      **If the robot looses balance, engage the p-stop.**
      * [ ] The robot does not lose balance in the first round.
      * [ ] The robot does not lose balance in the second round.
      * [ ] The robot does not lose balance in the third round.
   1. Release the p-stop if engaged.
   1. Lift the robot, go `default`, lower the robot.
   1. Reset the state estimator at origin (under "Motion Control").
   1. Press and release the p-stop.

1. Release the robot from the crane.
   
1. Test lying down and standing up:

   1. Switch the HBC knob to rest.
      **Engage the p-stop if the inspection payload does not fold.**
      * [ ] The inspection payload folds.
      * [ ] The HAA joints do not move unreasonably in- or outwards.
      * [ ] The touch-down is smooth.
   1. Switch the HBC knob to stand.
      * [ ] The inspection payload unfolds.
   
1. Test torso control:

   1. Enter the `torso_control` motion state (under "Motion Control" / "Smart").
   1. Move the torso using the HBC joystick.
      Start easy, but eventually try to "break" the functionality (ramp up angles & angular velocities, combine axis).
      * [ ] The robot follows motion commands.
      * [ ] The robot behaves well (does not enter singularities, start to vibrate, or switches from X to O configuration).
      
1. Test the free gait demo using the GUI:

   1. Run the `FreeGaitDemo` mission (under "Mission Execution").
      * [ ] The robot switches from XX to OO configuration.
      * [ ] The robot moves the torso w/o self-collision.
      * [ ] The robot switches from OO to XX configuration.

1. Test walking:

   **NOTE: Dangerous, use dedicated handles to push the robot, otherwise keep a safe distance to the robot!**

   1. Switch the HBC knob to walk.
   1. Firmly [push the robot](https://drive.google.com/file/d/1hRC2oZPfkrk7VniiLW56x1OYWhCAQKX0/view) around in all directions.
      * [ ] The robot recovers balance within 5 seconds.
   1. Walk around using the HBC joystick.
      Start easy, but eventually try to make the robot fall (ramp up velocities & accelerations, combine axis).
      * [ ] The robot follows motion commands.
      * [ ] The robot never falls.
   1. Do a full circle in each yawing direction (max forward velocity + max yaw). 
      Ensure enough free space.
      * [ ] The robot never falls.
      * [ ] Both circles look nice and similar.
   1. Attach the robot to the crane.
   1. Lay out wood pieces.
   1. Walk around using the HBC joystick.
      Start easy, but eventually try to make the robot fall (ramp up velocities & accelerations, combine axis).
      * [ ] The robot follows motion commands.
      * [ ] The robot never falls.
   1. Release the robot from the crane.
      
### D. Perception

1. Calibrate the depth camera tilt angles:

   1. Manually check that all depth sensors are mounted rigidly on the robot.
   1. Switch to the `odom` frame in RViz.
   1. Do a `square_up` in the middle of the free space.
   1. In one terminal, log into the NPC:
      ```
      anymal_ssh -c npc
      ```
   1. In a second terminal, log into the LPC and open the configuration file:
      ```
      anymal_ssh -c lpc
      nano /home/integration/.ros/config.yaml
      ```
   1. Repeat these instructions for all camera locations (`front`, `rear`, `left`, `right`):
      1. On NPC, run the calibration:
         ```
         roslaunch depth_sensor_tilt_calibration depth_camera_<location>.launch
         ``` 
      1. On LPC, overwrite the existing calibration values.
   1. On LPC, reload the configuration and restart the software:
      ```
      sudo systemctl restart anymal-load-config.service
      sudo systemctl restart anymal-sw-stack@lpc.service
      ```

1. Test perception sensors:

   1. In RViz, enable the perception sensors one by one: "LIDAR", "Depth Cameras", "Wide Angle Cameras".
      * [ ] LIDAR point cloud is shown.
      * [ ] Depth camera point clouds are shown.
      * [ ] Wide angle camera images are shown.
   1. In RViz, enable "Elevation Map".
   1. Walk around.
      * [ ] The elevation map is updated.
      * [ ] The elevation map is of high quality (no edges on flat terrain, etc.).
   1. Disable the topics again.
   
1. Test localization and mapping:

   1. Switch to the `map` frame in RViz.
   1. Load the `hagenholz.pb` map (under "Localization" / "Localization and Mapping").
      * [ ] Map is visualized in RViz.
   1. Localize the robot:
      1. Drag the interaction marker to robot's location.
      1. Right-click and select "Set localization initial guess".
      1. Enable "Localization".
      * [ ] The robot model appears at the specified location.
   1. Enable "Mapping".
      * [ ] "Active Map Size" increases (under "Advanced").
   1. Disable "Mapping".
   1. Reset the map by reloading it.
   1. Localize the robot again.
      
### E. Autonomy, Navigation & Inspection
   
1. Test single navigation task:

   1. Place robot in the middle of the dynamic testing area.
   1. Move interaction marker on Home position.
   1. Right-click and select "Go Here".
      * [ ] The robot follows the path nicely.
      * [ ] The robot reaches the goal.

1. Test teleoperated inspection:

   1. Check the inspection sensors:
      * [ ] The zoom camera image is displayed.
      * [ ] The thermal camera image is displayed.
   1. In "Start Single Task", select "Start Teleoperated Inspection" (under "Mission Execution").
   1. Move the inspection payload using the HBC joystick.
      * [ ] The movements are smooth.
   1. Move the inspection payload and zoom using the 3D mouse.
      * [ ] The movements are smooth.
      * [ ] The zooming is smooth.
      * [ ] The "reset" button resets pan-tilt angles and camera zoom level.
 
1. Test autonomous mission:
   
   **NOTES:**
   * **During this test the robot will climb the scaffolding stairs.
     On the long staircase the robot needs to be secured with the climbing rope.
     Instructions on how to use the rope are printed on the operator table.
     The robot does not stop automatically before climbing a staircase.
     To attach the rope, you need to pause the mission, and resume it again afterwards.
     On the two short stairs you can either use the rope or hold the robot by its handle facing upwards.
     Try to minimize the force induced on the robot while securing it. 
     Never stand below the robot as it can fall down the stairs.
     Use a helmet and wear closed shoes.**
   
   * **The mission cannot be launched if the robot does not have the docking software installed. 
     Use a robot with a docking socket for this part.**
   
   * **An inspection task is only performed if the robot reached its predefined navigation goal.
     If this is not the case the inspection is skipped and the robot proceeds with the following task.**
   
   1. Place the ramp, pallet, creeping bar and docking station at their designated locations.
   1. Get ready to play the [audio file](https://drive.google.com/file/d/1oiNta0VJIjEynEpWMlvnTextU1ECzQOW/view) at max volume.
   1. Make yourself familiar with the evaluation criteria listed in the final task of this test.
   1. Start "Inspection" mission.
   1. Check that the robot starts walking and follows the path.
   1. When the robot walks towards the OPC table, start the audio file.
   1. After the auditive inspection task, stop the audio file.
   1. Observe the creeping below the bar.
   1. Observe the dial inspection.
   1. Observe the thermal inspection of the room.
   1. Observe the docking and undocking.
   1. Before stair climbing, pause the mission, and attach the robot to the rope.
   1. Resume the mission (by clicking again on "pause").
      **Be ready to prevent the robot from a fall.**
   1. After stair climbing, pause the mission, and release it from the rope.
   1. Resume the mission (by clicking again on "pause").
      **Be ready to prevent the robot from a fall in case of missteps.**
   1. Observe the visual inspection of the analog display.
   1. Observe the stair climbing (2x).
   1. Observe the visual inspection of the digital display.
   1. Observe the robot stepping up onto the ramp.
      **Be ready to prevent the robot from a fall.**
   1. Observe the visual inspection of the valve.
   1. Observe the robot going down the ramp.
   1. Observe the robot going over the pallet using the locomotion planner.
   1. When the mission is done, stop the video recording.
   1. Fill in the evaluation criteria for the autonomous test:
      * [ ] The walking gait looks good (smooth motions, no slipping).
      * [ ] The creeping gait looks good (smooth motions, no slipping).
      * [ ] The locomotion planner gait looks good (smooth motions, no slipping).
      * [ ] The docking looks good.
      * [ ] The stair climbing looks good.
      * [ ] The step climbing looks good.
      * [ ] The robot walking over the ramp looks good (no slipping, reasonable velocity).
      * [ ] The navigation looks good (smooth path following).
      * [ ] The navigation is efficient (reasonable speed, no walking in place, no unnecessary turning).
      * [ ] The mission execution (task scheduling only) works fine.
      * [ ] The mission execution displays the progress of the current task.
      * [ ] The inspection provides visual feedback.
   
### F. Reporting

1. Copy the logs:

   1. Use an Ethernet cable to connect the OPC to the robot.
   1. On OPC, navigate to a partition with sufficient space to copy the logs to.
   1. The following commands will create a new folder structure and copy the logs into it:
   
      Note: If any of the `scp` commands hangs, run it without `sshpass -p ${ANYMAL_VERSION}`.
      ```
      anymal_setup -a <anymal-name>
      export DIR=$(date +'%Y-%m-%d')
      export YYYY=$(date +'%Y')
      export MM=$(date +'%m')
      export DD=$(date +'%d')
      mkdir -p ${DIR}/video ${DIR}/lpc/rosbag ${DIR}/lpc/llc ${DIR}/lpc/hlc ${DIR}/lpc/se ${DIR}/npc ${DIR}/apc ${DIR}/report
      sshpass -p ${ANYMAL_VERSION} scp -r integration@anymal-${ANYMAL_VERSION}-lpc:~/.ros/anymal_lpc_${YYYY}-${MM}-${DD}-* ${DIR}/lpc/rosbag
      sshpass -p ${ANYMAL_VERSION} scp -r integration@anymal-${ANYMAL_VERSION}-lpc:~/.ros/lowLevelController_${YYYY}${MM}${DD}_* ${DIR}/lpc/llc
      sshpass -p ${ANYMAL_VERSION} scp -r integration@anymal-${ANYMAL_VERSION}-lpc:~/.ros/highLevelController_${ANYMAL_VERSION}_${YYYY}${MM}${DD}_* ${DIR}/lpc/hlc
      sshpass -p ${ANYMAL_VERSION} scp -r integration@anymal-${ANYMAL_VERSION}-lpc:~/.ros/stateEstimator_${YYYY}${MM}${DD}_* ${DIR}/lpc/se
      sshpass -p ${ANYMAL_VERSION} scp -r integration@anymal-${ANYMAL_VERSION}-npc:~/.ros/anymal_npc_${YYYY}-${MM}-${DD}-* ${DIR}/npc
      sshpass -p ${ANYMAL_VERSION} scp -r integration@anymal-${ANYMAL_VERSION}-npc:~/.ros/reports/${YYYY}-${MM}-${DD}/DefaultReport/* ${DIR}/report
      sshpass -p ${ANYMAL_VERSION} scp -r integration@anymal-${ANYMAL_VERSION}-apc:~/.ros/anymal_apc_${YYYY}-${MM}-${DD}-* ${DIR}/apc
      ```
   1. Store the recorded video in the `video` subfolder.
   1. Double check the logs:
      * [ ] All folders contain logs.
   1. Upload the log folder to [GDrive](https://drive.google.com/drive/u/1/folders/1XI1alBeAsy-5CxYVZ9q3G_UDP47E-alr).
   
1. Open the report:
   ```
   /usr/bin/google-chrome --allow-file-access-from-files --user-data-dir=/tmp ./${DIR}/report/report.xml
   ```
   * [ ] The alarm can be heard when playing the audio file.
   * [ ] The dial has been analyzed correctly.
   * [ ] The thermal image looks good, and the temperature value looks right.
   * [ ] The analog display can be read by eye.
   * [ ] The digital display can be read by eye.
   * [ ] The valve status can be read by eye.

### G. Final Steps

1. Notify the software team through the GChat that the robot is free.
1. Add a comment to this issue indicating how much time you spent for the shakeout, e.g. `/spend 4h30m`.
1. Double check that you reported all issues.
1. Double check that this issue links to all encountered issues.
1. You're done! You can now close this issue. Thanks :)

/label V::Shakeout
