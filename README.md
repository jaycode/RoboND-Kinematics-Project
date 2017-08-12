Robotic Arm - Pick & Place Project
============

[//]: # (Image References)
[start]: ./readme_images/start.jpg
[dh]: ./readme_images/dh.png
[alpha]: ./readme_images/alpha.png
[alpha_i-1]: ./readme_images/alpha_i-1.png
[a]: ./readme_images/a_i-1.png
[d]: ./readme_images/d_i.png
[theta]: ./readme_images/theta_i.png
[pi2]: ./readme_images/pi2.png
[-pi2]: ./readme_images/-pi2.png
[theta1]: ./readme_images/theta_1.png
[theta2]: ./readme_images/theta_2.png
[theta2-90]: ./readme_images/theta_2-90.png
[theta3]: ./readme_images/theta_3.png
[theta4]: ./readme_images/theta_4.png
[theta5]: ./readme_images/theta_5.png
[theta6]: ./readme_images/theta_6.png
[transform-single]: ./readme_images/transform-single.png
[transform-simple]: ./readme_images/transform-simple.png
[transform-composition1]: ./readme_images/transform-composition1.png
[transform-composition2]: ./readme_images/transform-composition2.png
[A_r_P_A_0]: ./readme_images/A_r_P_A_0.png
[A]: ./readme_images/A.png
[P]: ./readme_images/P.png
[A_0]: ./readme_images/A_0.png
[R_small]: ./readme_images/r.png
[r_11]: ./readme_images/r_11.png
[A_B_R]: ./readme_images/A_B_R.png
[rotation-single]: ./readme_images/rotation-single.png
[transform-comb]: ./readme_images/transform-comb.png
[diag-clean]: ./readme_images/diag-clean.png
[diag-detailed]: ./readme_images/diag-detailed.png
[O_0]: ./readme_images/O_0.png
[O_1]: ./readme_images/O_1.png
[O_2]: ./readme_images/O_2.png
[O_2_1]: ./readme_images/O_2_1.png
[O_EE]: ./readme_images/O_EE.png
[Z_1]: ./readme_images/Z_1.png
[theta_2-calc]: ./readme_images/theta_2-calc.png
[theta_2-zoom]: ./readme_images/theta_2-zoom.png
[delta]: ./readme_images/delta.png
[delta-calc]: ./readme_images/delta-calc.png
[WC]: ./readme_images/WC.png
[WC^1]: ./readme_images/WC^1.png
[theta_3-zoom]: ./readme_images/theta_3-zoom.png
[theta_3-calc]: ./readme_images/theta_3-calc.png
[epsilon]: ./readme_images/epsilon.png
[epsilon-calc]: ./readme_images/epsilon-calc.png
[T]: ./readme_images/T.png
[R]: ./readme_images/R.png
[R-calc]: ./readme_images/R-calc.png
[R_0_6]: ./readme_images/R_0_6.png
[R_3_6]: ./readme_images/R_3_6.png
[R_rpy-calc]: ./readme_images/R_rpy-calc.png
[R_3_6-calc-LHS-1]: ./readme_images/R_3_6-calc-LHS-1.png
[R_3_6-calc-LHS-2]: ./readme_images/R_3_6-calc-LHS-2.png
[y]: ./readme_images/y.png
[P_small]: ./readme_images/p.png

![Start][start]

In this project, we are working with a simulation of Kuka KR210 to pick up cans from a shelf and then put them in a dropbox.

*Note: For information on setting up and running this project, consult Appendix 1 section below.*

# Forward and Inverse Kinematics

Forward Kinematics (FK) is a set of methods to calculate the final coordinate position and rotation of end-effector of a conjoined links (e.g. robotic arms, limbs, etc.), given parameters of each joint between links. In this project, these parameters are angles of each joint, totalling 6 joints (i.e. 6 Degrees of Freedom).

Inverse Kinematics (IK), on the other hand, is the exact opposite of FK, where we calculate the parameters from a given coordinate position and rotation.

# Homogenous Transforms

To calculate FK and IK calculation, we attach reference frames to each link of the manipulator and writing the homogeneous transforms from the fixed base link to link 1, link 1 to link 2, and so forth, all the way to the end-effector.

# Denavit-Hartenberg (DH) Parameters

To do FK and IK, we are using a method by Jacques Denavit and Richard Hartenberg which requires only four parameters for each reference frame.

![dh][dh]

Following are DH parameters used specifically in this project:

|ID   |![alpha][alpha_i-1] |![a][a] |![d][d] |![theta][theta]    |
|:---:|:------------------:|:------:|:------:|:-----------------:| 
|    1|                  0 |      0 |   0.75 |     ![q1][theta1] |
|    2|      ![-pi2][-pi2] |   0.35 |      0 |  ![q2][theta2-90] |
|    3|                  0 |   1.25 |      0 |     ![q3][theta3] |
|    4|      ![-pi2][-pi2] | -0.054 |   1.50 |     ![q4][theta4] |
|    5|        ![pi2][pi2] |      0 |      0 |     ![q5][theta5] |
|    6|      ![-pi2][-pi2] |      0 |      0 |     ![q6][theta6] |
|   EE|                  0 |      0 |  0.303 |                 0 |

**Homogenous transforms** are then combined together. Parameters of each transformation are set from **DH parameters**.Each transformation matrix looks like this:

![^{i-1}_iT=\begin{bmatrix}cos(\theta_i) &  - sin(\theta_i) & 0 & a \\ sin(\theta_i)cos(\alpha_{i-1}) & cos(\theta_i)cos(\alpha_{i-1}) &  - sin(\alpha_{i-1}) &  - d  *  sin(\alpha_{i-1}) \\ sin(\theta_i)sin(\alpha_{i-1}) & cos(\theta_i)sin(\alpha_{i-1}) & cos(\alpha_{i-1}) & d  *  cos(\alpha_{i-1}) \\ 0 & 0 & 0 & 1 \end{bmatrix}][transform-single]

[eq. 1]

Simplified as ![^{0}_1T][transform-simple] which means *tranformation from reference frame A to reference frame B*.

It is important to note (for later when we calculate ![q4][theta4] to ![q6][theta6] in this project) that this transformation matrix is consisted of rotational and translational matrices:

![transform-composition1][transform-composition1]

[eq. 2]

Or simplified as:

![transform-composition1][transform-composition2]

[eq. 3]

They should have come up with a better notation on this one, but ![^{A}r_{P/A_0}][A_r_P_A_0] means *the position of point ![P][P] relative to ![A_0][A_0], expressed in frame ![A][A]*. It is confusing since ![r][R_small] here has a different meaning than in ![r_11][r_11], in which the latter means *element [1,1] of a rotational matrix*.

![^{A}_BR][A_B_R] denotes the rotational matrix from frame A to frame B. In other words, then, ![R][R] block from [T][T] matrix above looks as follows:

![^{i-1}_iR=\begin{bmatrix}cos(\theta_i) &  - sin(\theta_i) & 0 \\ sin(\theta_i)cos(\alpha_{i-1}) & cos(\theta_i)cos(\alpha_{i-1}) &  - sin(\alpha_{i-1}) \\ sin(\theta_i)sin(\alpha_{i-1}) & cos(\theta_i)sin(\alpha_{i-1}) & cos(\alpha_{i-1})  \end{bmatrix}][rotation-single]

[eq. 4]

The links go from 0 to 6 and then followed by EE, that is why in the DH parameters above we have 7 rows. To combine transformations, calculate the dot products of all single transformations:

![^{0}_{EE}T=^{0}_{1}T * ^{1}_{2}T * ^{2}_{3}T * ^{3}_{4}T * ^{4}_{5}T * ^{5}_{6}T * ^{6}_{EE}T][transform-comb]

[eq. 5]


# Basic Solution

From the diagram above, notice that reference frame 4, 5, and 6 intersect at the same coordinate. We treat frame 5 as the **wrist center (WC)**, which then allow us to solve ![theta1][theta1] to ![theta3][theta3] analytically. ![theta4][theta4] to ![theta6][theta6] can then be found by solving 

## Joint 1 to 3

Here is a simplified diagram that shows frame 0 (ground) to frame 5 (or WC):

![diag-clean][diag-clean]

![theta1][theta1] is pretty straightforward. We can get it by rotating ![O_1][O_1] about its Z-axis (![Z_1][Z_1]). The diagram below shows the analytical method used to find ![q2][theta2] and ![q3][theta3].

![diag-clean][diag-detailed]

![q2][theta2] is the angle distance of ![O_2][O_2] and ![O_2_1][O_2_1], thus from the diagram above we see that it can be calculated with the following formula:

![\theta_2 = \pi/2 - \alpha - \delta][theta_2-calc]

[eq. 6]

![theta_2-zoom][theta_2-zoom]

Where ![delta][delta] was calculated from WC length and width relative to ![O_2][O_2]:

![\delta = atan2(WC_z - 0.75, \sqrt{WC_x^2 + WC_y^2} - 0.35)][delta-calc]

[eq. 7]

Note that ![alpha][alpha] here is different from ![\alpha_{i-1}][alpha_i-1] in the DH parameters above.

![\theta_3][theta3] can be calculated in a similar fashion:

![theta_3-zoom][theta_3-zoom]

![\theta_3 = \pi/2 - \beta - \epsilon][theta_3-calc]

[eq. 8]

And to calculate ![epsilon][epsilon](epsilon):

![\epsilon = atan2(d, \sqrt{a^2-d^2})][epsilon-calc]

[eq. 9]

## Joint 4 to 6

From equation 5 above, using only the rotation parts ![R][R] from ![T][T], and solve for reference frame 3 to 6, we have:

![^3_6R=inv(^0_3R) * ^0_6R][R-calc]

[eq. 10]

We then analyze the left and right hand sides of the equation above independently and solve for ![q4][theta4], ![q5][theta5], and ![q6][theta6].

### Left-Hand-Side of the equation

![^3_6R][R_3_6] can also be solved by calculating dot products of its components as described above in equation 5, or in other words, multiplying the matrix in equation 4 three times for ![q4][theta4], ![q5][theta5], and ![q6][theta6]:

![^3_6R=^3_4R*^4_5R*^5_6R][R_3_6-calc-LHS-1]

![^3_6R=\begin{bmatrix} - s(\theta_4)c(\theta_6) + c(\theta_4)c(\theta_5)c(\theta_6) &  - s(\theta_4)c(\theta_6) - s(\theta_6)c(\theta_4)c(\theta_5) & -s(\theta_5)c(\theta_4) \\ s(\theta_5)c(\theta_6) & - s(\theta_5)s(\theta_6) &  c(\theta_5) \\ - s(\theta_4)c(\theta_5)c(\theta_6) - s(\theta_6)c(\theta_4) & s(\theta_4)s(\theta_6)c(\theta_5) - c(\theta_4)c(\theta_6) & s(\theta_4)s(\theta_5)  \end{bmatrix}][R_3_6-calc-LHS-2]

[eq. 11]

### Right-Hand-Side of the equation

![^0_6R][R_0_6] can be found by evaluating the product of all rotations from ![O_0][O_0] to ![O_{EE}][O_EE]:

![^0_6R=R_{rpy}][R_rpy-calc]

[eq. 12]

![r][R_small] (another r!), ![p][P_small], and ![y][y] in this context stands for the roll, pitch, and yaw of the end-effector, which are known in this project.

## Result and problems with basic solution

The result can be seen in this [youtube video](https://youtu.be/THQ01RUHObg). As you see, the arm moves in a very ineffective way.

# Advanced Solution

*Will work on advanced solution after the course is passed.*


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
	
Once all these items are confirmed, open rviz window, hit Next button.

To view the complete demo keep hitting Next after previous action is completed successfully. 

Since debugging is enabled, you should be able to see diagnostic output on various terminals that have popped up.

The demo ends when the robot arm reaches at the top of the drop location. 

There is no loopback implemented yet, so you need to close all the terminal windows in order to restart.

In case the demo fails, close all three terminal windows and rerun the script.

## IK debug

To see how the algorithm was implemented, run `IK_debug.py` script.