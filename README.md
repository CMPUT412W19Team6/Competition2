# Competition2: Run, Robot Run

**Description:** In this competition, the robot is supposed to follow a line and count objects at certain points.

## 1. Purpose

For this competition, we want the robot to follow a pre-defined course and do different vision tasks. 

### 1.1 The Course Map

- The course map in simulation:
![map link](https://github.com/CMPUT412W19Team6/Competition2/blob/master/course_sim.png?s=200)

- The actual course's picture:
![pick link](https://github.com/CMPUT412W19Team6/Competition2/blob/master/course_pic.png?s=200)

- Explanation:
     - The robot is supposed to follow the white lines on the floor, and stop for a bit when reaching a red mark on the floor.
     - There are 3 locations where a vision task is required:
        
        1. Location 1: Detect how many cylinders are on the left , (Maximum 3), and signal the result with LED lights and sound. Example picture: ![location 1 picture](https://github.com/CMPUT412W19Team6/Competition2/blob/master/location1.png?s=200)
        2. Location 2: Detect how many geometric shapes are there (Maximum 3), signal the result with LED lights and sound,and recognize what the green shape is. Example picture: ![location 2 picture](https://github.com/CMPUT412W19Team6/Competition2/blob/master/location2.png?s=200)
        3. Location 3: Recognize the red shapes on the left one by one, and signal with a sound when finding one that's matching the green shape discovered at Location 2. Example picture: ![location 3 picture](https://github.com/CMPUT412W19Team6/Competition2/blob/master/location3.png?s=200)


## 2. Pre-requisites

## 2.1 Hardware requirement

- A kobuki turtlebot base
- An Asus Xtion Pro
- A controller (prefered logitach)
- A device with minimum 3 usb ports

### 2.2 Software requirement

- ROS kinetic ([Guide here](http://wiki.ros.org/kinetic/Installation/Ubuntu))

- Turtlebot packages ([Guide here](http://wiki.ros.org/action/show/Robots/TurtleBot?action=show&redirect=TurtleBot))

  ```bash
  sudo apt-get install ros-kinetic-turtlebot
  sudo apt-get install ros-kinetic-turtlebot-apps
  sudo apt-get install ros-kinetic-turtlebot-interactions
  sudo apt-get install ros-kinetic-turtlebot-simulator
  ```

- Kobuki ROS packages ([Guide here](https://wiki.ros.org/kobuki/Tutorials/Installation))

  ```bash
  sudo apt-get install ros-kinetic-kobuki
  sudo apt-get install ros-kinetic-kobuki-core
  ```

- Upgrade camera packages

  ```bash
  sudo apt-get install ros-kinetic-openni2-camera
  sudo apt-get install ros-kinetic-openni2-launch
  ```

- ROS smach ([Guide here](http://wiki.ros.org/smach))

## 3. Execution

### 3.1 Quickstart

1. Clone this repo into the source directory of your catkin workspace (e.g. catkin_ws/src)

   ```bash
   # under catkin_ws/src folder
   mkdir comp2
   git clone https://github.com/CMPUT412W19Team6/Competition2.git
   ```

2. Run catkin_make and source the setup.bash

   ```bash
   cd ..
   catkin_make
   source ./devel/setup.bash
   ```

3. Connect your your kobuki base, Asus Xtion Pro and controller.

4. Power up the kobuki base and put it on the start position

5. Start the library

   ```bash
   roslaunch comp2 start.launch
   ```

6. Start the turtlebot by pressing A on the controller


## 4. Concepts & Code

### Overview

- state machine:
  ![statemachine](https://github.com/CMPUT412W19Team6/Competition1/blob/master/statemachine.png?s=200)

### Evade

_Concept_:

    1. Move straight until the camera found anything that's within a 1.1 meters.

    2. Turn for 3 seconds. If range is less than 0.7 meter, turn with a higher anguler speed.
       Then check if the anything's winth `range`.
      > if yes, go to step 2
      > if no, go to step 1

    3. In case of a bump:
      3.1 move back 0.15 meter
      3.2 turn left
      3.3 move 0.4 meter
      3.4
        > if no more bump, turn back and go to step 1
        > if another bump, go to step 3.1


_Some code explanation_:

- How we calculated the minimum range from the laserScan message: minimum non-NAN number in the ranges list
  ```python
   def scan_callback(self, msg):
     validList = [x for x in msg.ranges if not math.isnan(x)]
     validList.append(float('Inf'))
     # g range ahead will be the minimal range
     self.g_range_ahead = min(validList)
  ```

### Following the white line

_Concept_:

    1. Filter out all the colors of the video stream except white and convert the images to be binary.

    2. Keep 20 rows of the images from the video stream starting at 3/4th the height of the images and discard all the other rows

    3. Calculate the moments and find the center of the white line. Since the images from the video have been converted to be binary, the white pixels of the white line can be considered mass whose zeroth moment represents the total mass and the first moment divided by the zeroth moment (total mass) is the center of the white line (center of mass).

    4. Adjust robot's angular velocity so that if the centroid is not at center, change robot's angular velocity to turn towards the centroid.
      
