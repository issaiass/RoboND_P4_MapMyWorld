# RoboND_P4_MapMyWorld


<details open>
<summary> <b>Brief Review<b></summary>

This project includes all necessary files reproduce a simulation of the the robotics nanodegree project 4 (Map My World).  We will create a 2D occupancy grid and a 3D octomap for a simulated environment using this dummy robot with the RTAB-Map Package.

RTAB-Map (Real-Time Appearance-Based Mapping) is a popular solution for SLAM to develop robots that can map environments in 3D. 

RTAB-Map has good speed and memory management, and it provides custom developed tools for information analysis. Most importantly, the quality of the documentation on [RTAB-Map ROS Wiki](http://wiki.ros.org/rtabmap_ros) is very high. Being able to leverage RTAB-Map with your own robots will lead to a solid foundation for mapping and localization.

For this project we will be using the rtabmap_ros package, which is a ROS wrapper (API) for interacting with RTAB-Map. Keep this in mind when looking at the relative documentation.


Below an example of the outcome

<p align="center">
<img src = "doc/gif/rtabmap.GIF?raw=true" center=true width="85%"/>

</p>

The project tree:

<p align="center">
<img src = "doc/img/rtabmap-2.PNG?raw=true" center=true width="85%"/>
<img src = "doc/img/tree.PNG?raw=true" width="55%"/>
</p>

The project is now divided in several folders and now you can easily excecute effectively each file.

</details>

<details open>
<summary> <b>Using The Package <b></summary>

NOTE:  *If you experience gazebo craching or rviz crashing, remember to add the ros gazebo models from the source repository of open robotics to .gazebo/models and also add my models from the RoboND_P1_Gazebo/model folder*.

- Follow the next steps to replicate the outcome
~~~
    cd ~
    mkdir -p catkin_ws/src
    cd catkin_ws
~~~
- Clone this repo in the `~/catkin_ws/src` folder by typing:
~~~ 
    cd ~/catkin_ws/src
    git clone https://github.com/issaiass/RoboND_P4_MapMyWorld.git
    cd ..
~~~
- Next compile and source the repository
~~~
    catkin_make
    source ~/catkin_ws/devel/setup.bash
~~~
- First, launch gazebo world and rviz, spawn the robot in the environment
~~~
    roslaunch my_robot world.launch
~~~
- Next, run the teleop node
~~~
    roslaunch rtabmap_pkg teleop.launch
~~~
- Finally, launch your mapping node.
~~~
    roslaunch rtabmap_pkg mapping.launch
~~~
- Note:  If you desire to perform localization using the map you created, there are only a few changes you need to make to the mapping.launch file. 

  - First, we remove the args="--delete_db_on_start" from your node launcher since you will need your database to localize too.
  - Next, we removed  Mem/NotLinkedNodesKept parameter
  - Finally, added the Mem/IncrementalMemory parameter of type string and set it to false to finalize the changes needed to put the robot into localization mode.

  Now you could launch the localization file.

~~~
    roslaunch rtabmap_pkg localization.launch
~~~


Navigate your robot in the simulation to create map for the environment! When you are all set, terminate the node and you could find your map db file in the place you specified in the launch file. If you did not modify the argument, it will be located in the .ros/ folder, usually in the root folder (~).

####Best Practices
You could start by lower velocity. Getting 3 loop closures will be sufficient for mapping the entire environment. You can maximize your loop closures by going over similar paths two or three times. This allows for the maximization of feature detection, facilitating faster loop closures! When you are done mapping, be sure to copy or move your database before moving on to map a new environment. Remember, relaunching the mapping node deletes any database in place on launch start up!

<details open>
<summary> <b>Results<b></summary>

We will made a youtube video later

<p align="center"> </p>
</details>



<details open>
<summary> <b>Issues<b></summary>

- None

</details>

<details open>
<summary> <b>Future Work<b></summary>

- Implement RTAB-Map in other robots

</details>

<details open>
<summary> <b>Contributing<b></summary>

Your contributions are always welcome! Please feel free to fork and modify the content but remember to finally do a pull request.

</details>

<details open>
<summary> :iphone: <b>Having Problems?<b></summary>

<p align = "center">

[<img src="https://img.shields.io/badge/linkedin-%230077B5.svg?&style=for-the-badge&logo=linkedin&logoColor=white" />](https://www.linkedin.com/in/riawa)
[<img src="https://img.shields.io/badge/telegram-2CA5E0?style=for-the-badge&logo=telegram&logoColor=white"/>](https://t.me/issaiass)
[<img src="https://img.shields.io/badge/instagram-%23E4405F.svg?&style=for-the-badge&logo=instagram&logoColor=white">](https://www.instagram.com/daqsyspty/)
[<img src="https://img.shields.io/badge/twitter-%231DA1F2.svg?&style=for-the-badge&logo=twitter&logoColor=white" />](https://twitter.com/daqsyspty) 
[<img src ="https://img.shields.io/badge/facebook-%233b5998.svg?&style=for-the-badge&logo=facebook&logoColor=white%22">](https://www.facebook.com/daqsyspty)
[<img src="https://img.shields.io/badge/linkedin-%230077B5.svg?&style=for-the-badge&logo=linkedin&logoColor=white" />](https://www.linkedin.com/in/riawe)
[<img src="https://img.shields.io/badge/tiktok-%23000000.svg?&style=for-the-badge&logo=tiktok&logoColor=white" />](https://www.linkedin.com/in/riawe)
[<img src="https://img.shields.io/badge/whatsapp-%23075e54.svg?&style=for-the-badge&logo=whatsapp&logoColor=white" />](https://wa.me/50766168542?text=Hello%20Rangel)
[<img src="https://img.shields.io/badge/hotmail-%23ffbb00.svg?&style=for-the-badge&logo=hotmail&logoColor=white" />](mailto:issaiass@hotmail.com)
[<img src="https://img.shields.io/badge/gmail-%23D14836.svg?&style=for-the-badge&logo=gmail&logoColor=white" />](mailto:riawalles@gmail.com)

</p

</details>

<details open>
<summary> <b>License<b></summary>
<p align = "center">
<img src= "https://mirrors.creativecommons.org/presskit/buttons/88x31/svg/by-sa.svg" />
</p>
</details>
