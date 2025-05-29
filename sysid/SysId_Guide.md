# SysId Guide for Different Mechanisms

## Swerve drive motor gains:
1. Go to CommandSwerveDrivetrain and set m_sysIdRoutineToApply to the SysId translation test.
2. Go to RobotContainer. Make sure you have controller bindings for quasistatic forward, quasistatic backward, dynamic forward and dynamic backward.
3. Make sure to call SignalLogger.start() in robotInit or testInit, and SignalLogger.stop() in testExit.
4. Deploy your robot code.
5. Make sure your robot has as much room as possible to move forward and backward.
6. Enable test mode, run all four tests for as long as you can without driving into anything, and disable test mode.
7. Open Phoenix Tuner X, go to the “Tools” menu, and go to the “Log Extractor” menu.
8. Press connect, and press the up arrow to go up each directory until you reach the root. Then go to .media/sda1/logs”.
9. Find the log file with the exact date and time of your test. It’s the file that doesn’t start with “rio”.
10. Select the downloads folder as your directory and download the file.
11. Click on the file on the right menu, then change the output type to wpilog, then convert.
12. Open up SysId and press “open data log file” to open your converted file.
13. Drag the SysId string from the list of entries to the Test State.
14. Decide on a drive motor (doesn't matter which one), and find the ID from TunerConstants. Open the dropdown for that motor in SysId and drag MotorVoltage to Voltage, Position to Position, and Velocity to Velocity. Set "Analysis Type" to "Simple".
15. Select Rotations as your units and press load.
16. Compare your results to the ones on this page (https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/viewing-diagnostics.html). If your sim velocity r-squared in less than 0.9 or your acceleration R-squared is less than 0.2, you must adjust the plots using the directions from the page. (As a general tip, increasing window size increases acceleration r-squared and increasing the velocity threshold increases sim velocity r-squared.) If you can’t get those values past 0.9 and 0.2 without losing most of your filtered data, you need to run it again. (Keep in mind that SysId translation is one of the hardest tests to get good gains from, especially if you’re using Colson wheels.)
17. Copy the ks and kv to driveGains in TunerConstants, **but not ka**. You can write down ka somewhere else, but it's only useful if you want to find your robot's MOI for PathPlanner.
18. Under Feedback Analysis, choose “CTRE Phoenix 6” gain preset and “Velocity” loop type.
19. Copy kp and kd to driveGains in TunerConstants. kd might be 0.
20. Comment out SignalLogger.stop() in testExit. If you're concerned about memory usage, comment out the SysId routine as well.


## Swerve steer motor gains:
1. Go to CommandSwerveDrivetrain and set m_sysIdRoutineToApply to the SysId steer test.
2. Go to RobotContainer. Make sure you have controller bindings for quasistatic forward, quasistatic backward, dynamic forward and dynamic backward.
3. Make sure to call SignalLogger.start() in robotInit or testInit, and SignalLogger.stop() in testExit.
4. Deploy your robot code.
5. Make sure your robot has room to shake when the steer motors spin.
6. Enable test mode, run all four tests for 5 seconds each. Disable test mode.
7. Open Phoenix Tuner X, go to the “Tools” menu, and go to the “Log Extractor” menu.
8. Press connect, and press the up arrow to go up each directory until you reach the root. Then go to .media/sda1/logs”.
9. Find the log file with the exact date and time of your test. It’s the file that doesn’t start with “rio”.
10. Select the downloads folder as your directory and download the file.
11. Click on the file on the right menu, then change the output type to wpilog, then convert.
12. Open up SysId and press “open data log file” to open your converted file.
13. Drag the SysId string from the list of entries to the Test State.
14. Decide on a steer motor (doesn't matter which one), and find the ID from TunerConstants. Open the dropdown for that motor in SysId and drag MotorVoltage to Voltage, Position to Position, and Velocity to Velocity. Set "Analysis Type" to "Simple".
15. Select Rotations as your units and press load.
16. Compare your results to the ones on this page (https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/viewing-diagnostics.html). If your sim velocity r-squared in less than 0.9 or your acceleration R-squared is less than 0.2, you must adjust the plots using the directions from the page. (As a general tip, increasing window size increases acceleration r-squared and increasing the velocity threshold increases sim velocity r-squared.) If you can’t get those values past 0.9 and 0.2 without losing most of your filtered data, you need to run it again.
17. Copy the ks, kv and ka to steerGains in TunerConstants.
18. Under Feedback Analysis, choose “CTRE Phoenix 6” gain preset and “Position” loop type.
19. Copy kp and kd to steerGains in TunerConstants.
20. Comment out SignalLogger.stop() in testExit. If you're concerned about memory usage, comment out the SysId routine as well.


## Swerve rotation gains (this is only useful if you want to calculate your robot’s exact MOI for PathPlanner. I’m not sure how necessary this is.):
1. Go to CommandSwerveDrivetrain and set m_sysIdRoutineToApply to the SysId rotation test.
2. Go to RobotContainer. Make sure you have controller bindings for quasistatic forward, quasistatic backward, dynamic forward and dynamic backward.
3. Make sure to call SignalLogger.start() in robotInit or testInit, and SignalLogger.stop() in testExit.
4. Deploy your robot code.
5. Make sure your robot has room to rotate.
6. Enable test mode, run all four tests for 10 seconds each. Disable test mode.
7. Open Phoenix Tuner X, go to the “Tools” menu, and go to the “Log Extractor” menu.
8. Press connect, and press the up arrow to go up each directory until you reach the root. Then go to .media/sda1/logs”.
9. Find the log file with the exact date and time of your test. It’s the file that doesn’t start with “rio”.
10. Select the downloads folder as your directory and download the file.
11. Click on the file on the right menu, then change the output type to wpilog, then convert.
12. Open up SysId and press “open data log file” to open your converted file.
13. Drag the SysId string from the list of entries to the Test State.
14. Decide on a drive motor (doesn't matter which one), and find the ID from TunerConstants. Open the dropdown for that motor in SysId and drag MotorVoltage to Voltage. Drag the pigeon 2’s Yaw to Position and AngularVelocityZWorld to Velocity. Set "Analysis Type" to "Simple".
15. Select radians as your units and PI/180 (copy/paste from desmos) as your velocity and position scaling. Press load.
16. Compare your results to the ones on this page (https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/viewing-diagnostics.html). If your sim velocity r-squared in less than 0.9 or your acceleration R-squared is less than 0.2, you must adjust the plots using the directions from the page. (As a general tip, increasing window size increases acceleration r-squared and increasing the velocity threshold increases sim velocity r-squared.) If you can’t get those values past 0.9 and 0.2 without losing most of your filtered data, you need to run it again.
17. Save the ka in a constants file.
18. Comment out SignalLogger.stop() in testExit. If you're concerned about memory usage, comment out the SysId routine as well.


## Swerve drive slip current:
1. Go to CommandSwerveDrivetrain and create a SysId slip current routine similar to [this one](https://github.com/MDHSRobotics/Dive-2025/blob/main/src/main/java/frc/robot/subsystems/drive/CommandSwerveDrivetrain.java#L125). Set m_sysIdRoutineToApply to the routine. You should set the quasistatic ramp rate to something low, like 0.1 volts per second. You will not need to log the state in the routine, because you will view it live in AdvantageScope.
2. Go to RobotContainer. Make sure you have a controller binding for quasistatic forward. Do not use quasistatic backward or dynamic.
4. Deploy your robot code, and open AdvantageScope. Go to "Help", then "Show Preferences", then set "Live Source" to Phoenix Diagnostics.
5. Get the ID of one of the front drive motors in TunerConstants, and find the motor in AdvantageScope.
6. Open up a line graph and plot Velocity and StatorCurrent.
7. Enable test mode and drive the robot up to a wall so the front is fully against the wall.
8. Run quasistatic forward until the wheels stop whistling and start slipping. Then disable test mode.
9. Press the pause button in AdvantageScope, and zoom in on the line graph to find when the velocity starts to rise or drop and the stator current starts to drop. The maximum stator current is your slip current.
10. Save the value to "kSlipCurrent" in TunerConstants.
11. Open AdvantageScope. Go to "Help", then "Show Preferences", then set "Live Source" back to NetworkTables 4.
12. If you're concerned about memory usage, comment out the SysId routine as well.


## Other CTRE gains (like for an elevator):
1. Write a SysId routine and SysId commands for the mechanism. Use this as an example: https://github.com/MDHSRobotics/Dive-2025/blob/experiments/src/main/java/frc/robot/subsystems/elevator/Elevator.java
2. Go to RobotContainer. Make sure you have controller bindings for quasistatic forward, quasistatic backward, dynamic forward and dynamic backward.
3. Make sure to call SignalLogger.start() in robotInit or testInit, and SignalLogger.stop() in testExit.
4. Deploy your robot code.
5. Make sure your robot has room to move as much as it can and for as long as it can. **It is highly encouraged to find your soft limits beforehand.**
6. Enable teleop mode, and run for a split second to see how fast it goes. If it looks like it could damage itself from the speed, disable teleop, lower the voltage, and try again.
7. Redeploy the code to start a new log file. Enable test mode, run all four tests for as long as you can without hitting any limits. Disable test mode.
8. Open Phoenix Tuner X, go to the “Tools” menu, and go to the “Log Extractor” menu.
9. Press connect, and press the up arrow to go up each directory until you reach the root. Then go to .media/sda1/logs”.
10. Find the log file with the exact date and time of your test. It’s the file that doesn’t start with “rio”.
11. Select the downloads folder as your directory and download the file.
12. Click on the file on the right menu, then change the output type to wpilog, then convert.
13. Open up SysId and press “open data log file” to open your converted file.
14. Drag the SysId string from the list of entries to the Test State.
15. Find your motor. Open the dropdown for that motor in SysId and drag MotorVoltage to Voltage, Position to Position, and Velocity to Velocity. Set "Analysis Type" to Arm if you're testing an arm (or Simple if the arm is resting on its side), Elevator if you're testing an elevator, and Simple for everything else.
16. Select Rotations as your units and press load.
17. Compare your results to the ones on this page (https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/viewing-diagnostics.html). If your sim velocity r-squared in less than 0.9 or your acceleration R-squared is less than 0.2, you must adjust the plots using the directions from the page. (As a general tip, increasing window size increases acceleration r-squared and increasing the velocity threshold increases sim velocity r-squared.) If you can’t get those values past 0.9 and 0.2 without losing most of your filtered data, you need to run it again.
18. Copy the ks, kv and ka to a constants file, as well as kg if you have something that falls due to gravity.
19. Under Feedback Analysis, choose “CTRE Phoenix 6” gain preset. Select “Position” loop type if you’re using a position controller, or “Velocity” if you’re using a velocity controller.
20. Copy kp and kd to steerGains in TunerConstants.
21. Comment out SignalLogger.stop() in testExit. If you're concerned about memory usage, comment out the SysId routine as well.


## REV gains:
1. Use the URCL page as a guide if you need more information: https://docs.advantagescope.org/more-features/urcl/
2. Write a SysId routine and SysId commands for the mechanism. Use this as an example: https://github.com/MDHSRobotics/Dive-2025/blob/experiments/src/main/java/frc/robot/subsystems/elevator/Elevator.java
3. Decide what units you want your gains in. I suggest radians and radians per second because ArmFeedforward requires these units. Regardless, set position and velocity conversion factors in your motor configs to convert from rotations and rotations per minute to your desired units.
4. Enable either primaryEncoderPosition/VelocityAlwaysOn or absoluteEncoderPosition/VelocityAlwaysOn, depending on whether you use the built in encoder or a separate absolute encoder.
5. Go to RobotContainer. Make sure you have controller bindings for quasistatic forward, quasistatic backward, dynamic forward and dynamic backward.
6. Make sure to call DataLogManager.start() and URCL.start() in robotInit or testInit, and DataLogManager.stop() in testExit.
7. Deploy your robot code.
8. Make sure your robot has room to move as much as it can and for as long as it can. **It is highly encouraged to find your soft limits beforehand.**
9. Enable teleop mode, and run for a split second to see how fast it goes. If it looks like it could damage itself from the speed, disable teleop, lower the voltage, and try again.
10. If your mechanism falls quickly due to gravity and prevents you from running any backwards tests, feel free to lay the robot on its side.
11. Redeploy the code to start a new log file. Enable test mode, run all four tests for as long as you can without hitting any limits. Disable test mode.
12. Turn off the robot, and take the USB drive out and plug it into your computer.
13. Open AdvantageScope, press “File” and “Open Log(s)” and find your log on the USB drive. It should be the most recent .wpilog.
14. Press “File” and “Export Data”. Set the format to “WPILOG” and the field set to “Include Generated”. Press the save icon.
15. Open up SysId and press “open data log file” to open your file.
16. Drag the SysId string from the list of entries to the Test State.
17. Find your motor. Open the dropdown for that motor in SysId and drag AppliedOutputVoltage to Voltage, primary or absolute position to Position, and primary or absolute velocity to Velocity. Set "Analysis Type" to Arm if you're testing an arm (or Simple if the arm is resting on its side), Elevator if you're testing an elevator, and Simple for everything else.
18. Select whatever units you chose to use and press load.
19. Compare your results to the ones on this page (https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/viewing-diagnostics.html). If your sim velocity r-squared in less than 0.9 or your acceleration R-squared is less than 0.2, you must adjust the plots using the directions from the page. (As a general tip, increasing window size increases acceleration r-squared and increasing the velocity threshold increases sim velocity r-squared.) If you can’t get those values past 0.9 and 0.2 without losing most of your filtered data, you need to run it again.
20. Copy the ks, kv and ka to a constants file, as well as kg if you have something that falls due to gravity.
21. Select “REV Brushless Encoder Port” for Gain Preset. (In the past, I recalculated the measurement delay using the SysId documentation, but this shouldn’t be necessary.)
22. If you chose velocity units in terms of “per seconds” instead of “per minutes,” change Velocity Denominator Units to 1.
23. Select “Position” loop type if you’re using a position controller, or “Velocity” if you’re using a velocity controller.
24. Copy kp and kd to a constants file.
25. Comment out DataLogManager.stop() in testExit. If you're concerned about memory usage, comment out the SysId routine as well.
