Robotic arm - Pick & Place project
============

[//]: # (Image References)
[start]: ./readme_images/start.jpg
[dh]: ./readme_images/dh.png
[alpha]: ./readme_images/alpha_i-1.png
[a]: ./readme_images/a_i-1.png
[d]: ./readme_images/d_i.png
[theta]: ./readme_images/theta_i.png
[pi2]: ./readme_images/pi2.png
[-pi2]: ./readme_images/-pi2.png
[theta1]: ./readme_images/theta_1.png
[theta2]: ./readme_images/theta_2.png
[theta3]: ./readme_images/theta_3.png
[theta4]: ./readme_images/theta_4.png
[theta5]: ./readme_images/theta_5.png
[theta6]: ./readme_images/theta_6.png
[transform-single]: ./readme_images/transform_single.png
[transform-simple]: ./readme_images/transform_simple.png
[transform-comb]: ./readme_images/transform_comb.png
[diag-clean]: ./readme_images/diag-clean.png
[diag-detailed]: ./readme_images/diag-detailed.png
[diag-zoom]: ./readme_images/diag-zoom.png
[O_1]: ./readme_images/O_1.png

![Start][start]

In this project, we are working with a simulation of Kuka KR210 to pick up cans from a shelf and then put them in a dropbox.

*Note: For information on setting up and running this project, consult Appendix 1 section below.*

# Forward and Inverse Kinematics

Forward Kinematics (FK) is a set of methods to calculate the final coordinate position and rotation of end effector of a conjoined links (e.g. robotic arms, limbs, etc.), given parameters of each joint between links. In this project, these parameters are angles of each joint, totalling 6 joints (i.e. 6 Degrees of Freedom).

Inverse Kinematics (IK), on the other hand, is the exact opposite of FK, where we calculate the parameters from a given coordinate position and rotation.

# Homogenous Transforms

To calculate FK and IK calculation, we attach reference frames to each link of the manipulator and writing the homogeneous transforms from the fixed base link to link 1, link 1 to link 2, and so forth, all the way to the end effector.

# Denavit-Hartenberg (DH) Parameters

To do FK and IK, we are using a method by Jacques Denavit and Richard Hartenberg which requires only four parameters for each reference frame.

![dh][dh]

Following are DH parameters used specifically in this project:

|ID   |![alpha][alpha] |![a][a] |![d][d] |![theta][theta] |
|:---:|:--------------:|:------:|:------:|:--------------:| 
|    1|              0 |      0 |   0.75 |  ![q1][theta1] |
|    2|  ![-pi2][-pi2] |   0.35 |      0 |  ![q2][theta2] |
|    3|              0 |   1.25 |      0 |  ![q3][theta3] |
|    4|  ![-pi2][-pi2] | -0.054 |   1.50 |  ![q4][theta4] |
|    5|    ![pi2][pi2] |      0 |      0 |  ![q5][theta5] |
|    6|  ![-pi2][-pi2] |      0 |      0 |  ![q6][theta6] |
|   EE|              0 |      0 |  0.303 |              0 |

**Homogenous transforms** are then combined together. Parameters of each transformation are set from **DH parameters**.Each transformation matrix looks like this:

![\begin{bmatrix}cos(\theta_i) &  - sin(\theta_i) & 0 & a \\ sin(\theta_i)cos(\alpha_{i-1}) & cos(\theta_i)cos(\alpha_{i-1}) &  - sin(\alpha_{i-1}) &  - d  *  sin(\alpha_{i-1}) \\ sin(\theta_i)sin(\alpha_{i-1}) & cos(\theta_i)sin(\alpha_{i-1}) & cos(\alpha_{i-1}) & d  *  cos(\alpha_{i-1}) \\ 0 & 0 & 0 & 1 \end{bmatrix}][transform-single]

Simplified as:

![^{0}_1T][transform-simple]

The links go from 0 to 6 and then followed by EE, that is why in the DH parameters above we have 7 rows. To combine transformations, calculate the dot products of all single transformations:

![^{0}_{EE}T=^{0}_{1}T * ^{1}_{2}T * ^{2}_{3}T * ^{3}_{4}T * ^{4}_{5}T * ^{5}_{6}T * ^{6}_{EE}T][transform-comb]


# Basic Solution

From the diagram above, notice that reference frame 4, 5, and 6 intersect at the same coordinate. We treat frame 5 as the **wrist center (WC)**, which then allow us to solve ![theta1][theta1] to ![theta3][theta3] analytically.

Here is a simplified diagram, showing frame 0 (ground) to frame 5 (or WC):

![diag-clean][diag-clean]

![theta1][theta1] is pretty straightforward. We can get it by rotating ![O_1][O_1] about its Z-axis.

![diag-clean][diag-detailed]
![diag-zoom][diag-zoom]

## Joint 1 to 3

## Joint 4 to 6

Our entire 

## Problems with basic solution

# Advanced Solution


# Appendix 1. How to Run

Make sure you are using robo-nd VM or have Ubuntu+ROS installed locally.

### One time Gazebo setup step:
Check the version of gazebo installed on your system using a terminal:
```sh
$ gazebo --version
```
To run projects from this repository you need version 7.7.0+
If your gazebo version is not 7.7.0+, perform the update as follows:
```sh
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install gazebo7
```

Once again check if the correct version was installed:
```sh
$ gazebo --version
```
### For the rest of this setup, catkin_ws is the name of active ROS Workspace, if your workspace name is different, change the commands accordingly

If you do not have an active ROS workspace, you can create one by:
```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

Now that you have a workspace, clone or download this repo into the **src** directory of your workspace:
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/udacity/RoboND-Kinematics-Project.git
```

Now from a terminal window:

```sh
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ sudo chmod +x target_spawn.py
$ sudo chmod +x IK_server.py
$ sudo chmod +x safe_spawner.sh
```
Build the project:
```sh
$ cd ~/catkin_ws
$ catkin_make
```

Add following to your .bashrc file
```
export GAZEBO_MODEL_PATH=~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/models

source ~/catkin_ws/devel/setup.bash
```

For demo mode make sure the **demo** flag is set to _"true"_ in `inverse_kinematics.launch` file under /RoboND-Kinematics-Project/kuka_arm/launch

In addition, you can also control the spawn location of the target object in the shelf. To do this, modify the **spawn_location** argument in `target_description.launch` file under /RoboND-Kinematics-Project/kuka_arm/launch. 0-9 are valid values for spawn_location with 0 being random mode.

You can launch the project by
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ ./safe_spawner.sh
```

If you are running in demo mode, this is all you need. To run your own Inverse Kinematics code change the **demo** flag described above to _"false"_ and run your code (once the project has successfully loaded) by:
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ rosrun kuka_arm IK_server.py
```
Once Gazebo and rviz are up and running, make sure you see following in the gazebo world:

	- Robot
	
	- Shelf
	
	- Blue cylindrical target in one of the shelves
	
	- Dropbox right next to the robot
	

If any of these items are missing, report as an issue.

Once all these items are confirmed, open rviz window, hit Next button.

To view the complete demo keep hitting Next after previous action is completed successfully. 

Since debugging is enabled, you should be able to see diagnostic output on various terminals that have popped up.

The demo ends when the robot arm reaches at the top of the drop location. 

There is no loopback implemented yet, so you need to close all the terminal windows in order to restart.

In case the demo fails, close all three terminal windows and rerun the script.