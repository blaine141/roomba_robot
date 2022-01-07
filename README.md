# roomba_robot

ROS packages to add advanced navigation to my modified roomba. Requires custom hardware to be added for control. 

## Install

#### Prerequisites

* Internet connection
* [ROS](http://wiki.ros.org/ROS/Installation) _Kinetic_ or _Melodic_
* Ubuntu packages: `python-rosdep`, `python-catkin-tools`

``` bash
$ sudo apt-get install python-rosdep python-catkin-tools
```

#### Compiling

1. Create a catkin workspace  
    ``` bash
    $ cd ~
    $ mkdir -p roomba_ws/src  
    $ cd roomba_ws  
    $ catkin init  
    ```

2. Clone [libcreate](https://github.com/AutonomyLab/libcreate) into the workspace so it can be referenced

3. Clone [this repo](https://github.com/blaine141/roomba_robot) into the workspace
  
3. Install dependencies  
    ``` bash
    $ rosdep update  
    $ rosdep install --from-paths src -i  
    ```

4. Build  
    ``` bash
    $ catkin build
    ```
#### I/O Permissions
5. In order to connect to Create over serial, ensure your user is in the dialout group
    ``` bash
    $ sudo usermod -a -G dialout $USER
    ```

6. Logout and login for permission to take effect


## Sources

The driver code and install instructions were copied from [create_robot](https://github.com/AutonomyLab/create_robot) and modified. You can reference this repo for usage. I have not contributed to that project. 