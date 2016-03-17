This folder contains a simple working example of non-repeatable behaviour in ROS/Gazebo.

To run use:
$ roslaunch bert2_simulator gripper_repeatability_test.launch

The BERT2 robot loops through the following behaviour ten times:
- Move hand to object
- Grasp object
- Move object
- Release

For demonstration purposes, the robot control and simulation in this example are constructed such that the grasping of the object is not stable.  Sometimes (roughly 20 percent of the time) the robot drops the object or fails to grasp it.  When running the launch file multiple times, there is no consistency as to which tests are fails or the manner in which the fail happens.

As explained by Yoav Hollander (Foretellix Ltd, blog.foretellix.com), for the task of verifying robot code, this makes it difficult to...

"
- ... debug (timing-related bugs, as these may appear only once-per-many-runs). Yes, you can do some record-replay tricks, but then if you want to experiment (“does this change eliminate the bug?”) you can’t use the recording.
- ... check that a new release did not break something (an apparently-new bug may have existed in old version too)
- ... check that a new release indeed fixed some bug (an apparently clean run may be the result of slightly different timing)
- ... reliably measure coverage (run it again and you’ll get different coverage).
- ... demonstrate things with confidence: “Oops – let me run this again – hopefully it will show the effect I mean this time”.
"

Hence, even in robotic platforms that are inherently non-repeatable, repeatability of simulation-based testing is important.
