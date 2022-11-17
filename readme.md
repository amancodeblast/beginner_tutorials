[![GitHub license](https://badgen.net/github/license/Naereen/Strapdown.js)](LICENSE.md)
# Beginers Tutorial 
## A simple Service Client with Publisher and Subscriber

### Dependencies
* Installed on Linux (Ubuntu 22.04)
* Ros 2 Humble installed 
* And a workspace created for ros2. If not then follow the instructions below:
<details>
<summary>How to reate a ros Workspace?</summary>

```xml
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

</details>

### Building this package
These simple steps are to be followed to replicate the work of this repository from scratch

<details>
<summary>Clone this package </summary>
Inside your ros_workspace/src clone the following package

```xml 
git clone https://github.com/amancodeblast/beginner_tutorials.git
```
make sure you have selected the tag "ros_pub_sub_Release"
</details>



<details>
<summary>Compile the package </summary>

Complile the package using the command 
```xml
colcon build --packages-select beginner_tutorials
``` 
</details>
<details>
<summary>Source the workspace and run the nodes </summary>

Open a new terminal, navigate to ros2_ws, and source the setup files:

```xml
. install/setup.bash
```

Now running the talker or publisher node

```ros2 run beginner_tutorials talker```

Now running the listener or subscriber on another terminal 
**Note**: Don't forget to source the workspace in every terminal you want to use ros commands 

```ros2 run beginner_tutorials listener```
</details>
### How this package was build
<details>
<summary>Create a Ros Package from scratch</summary>

```ros2 pkg create --build-type ament_cmake beginner_tutorials```
</details>

<details>
<summary>Write Publisher Subscriber node </summary>

Download the scripts in src folder
For subscriber 
```xml
 wget -O subscriber_member_function.cpp https://raw.githubusercontent.com/ros2/examples/humble/rclcpp/topics/minimal_subscriber/member_function.cpp 
 ```
For publisher 
```xml 
wget -O publisher_member_function.cpp https://raw.githubusercontent.com/ros2/examples/humble/rclcpp/topics/minimal_publisher/member_function.cpp
```

The publisher and the subscriber is slightly changed for this repo for a custom message. You can compare the original file from the one in the repo.   

</details>

<details>
<summary>Change the Cmakelist.txt or package.xml</summary>

Check the link here to change the Cmakelist.txt and the package.xml for publisher and subscriber code. [Link](http://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html#add-dependencies)
</details>
## Running Linterand checker
<details>
<summary>Execute the following scripts for Linter and CHecker output </summary>

```xml 
sh cpplint.sh
sh cppcheck.sh
```

</details>