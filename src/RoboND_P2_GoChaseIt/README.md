# RoboND_P2_GoChaseIt


<details open>
<summary> <b>Brief Review<b></summary>

This project includes all necessary files reproduce a simulation of the the robotics nanodegree project 2 (Go Chase It) using rviz and gazebo to visualize the camera, control and navigate with the differential drive robot.


Below an example of the outcome

<p align="center">
<img src = "doc/gif/GoChaseIt.gif?raw=true" center="true" width="55%"/>
</p>

The project tree:

<p align="center">
<img src = "doc/img/tree.PNG?raw=true" width="65%"/>
</p>

The project is now divided in several folders and now you can easily excecute effectively each file.

</details>

<details open>
<summary> <b>Using The Package <b></summary>

NOTE:  You must include the models to your .gazebo/models folder or initialize .gazebo/gui.ini file with the mathe where are the models to reference to the map and the ball.

- Put *my_ball* folder from *RoboND_P2_GoChaseIt/my_robot/model* folder inside *~/.gazebo/models* folder to load also the white ball.
- See prerequisites on the *package.xml* of each package.
- Create a ROS ros workspace and compile an empty package:
~~~
    cd ~
    mkdir -p catkin_ws/src
    cd catkin_ws
    catkin_make
~~~
- Open the `.bashrc` with nano:
~~~
    nano ~/.bashrc
~~~    
- Insert this line at the end of the `~/.bashrc` file for sourcing your workspace:
~~~
    source ~/catkin_ws/devel/setup.bash
~~~
- Clone this repo in the `~/catkin_ws/src` folder by typing:
~~~ 
    cd ~/catkin_ws/src
    git clone https://github.com/issaiass/RoboND_P2_GoChaseIt.git
    cd ..
~~~
- Go to the root folder `~/catkin_ws` and make the folder running `catkin_make` to ensure the application compiles.
- Now you can test the package
- Launch the robot inside your world
~~~
    roslaunch my_robot world.launch
~~~
- Run the *drive_bot* and *process_image* nodes
- You could enable more parameters if tab is pressed
~~~
    roslaunch ball_chaser ball_chaser.launch
~~~
- Visualize the camera
~~~
    rosrun rqt_image_view rtq_image_view
~~~

<details open>
<summary> <b>Results<b></summary>

We will made a youtube video later

<p align="center"> </p>
</details>



<details open>
<summary> <b>Issues<b></summary>

- None, but you should note that i put angular and linear velocities to 0.5

</details>

<details open>
<summary> <b>Future Work<b></summary>

- Plan to use the lidar
- Plan to make it better from navigate

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