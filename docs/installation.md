# Installation

## Dependencies
- **`ROS noetic:`** full desktop version is preferred. Can install other variant dependant on your use case.
- **`rosdep:`** command-line tool for installing system dependencies. Follow these install [instructions](http://wiki.ros.org/rosdep).
- **`(optional) Docker:`** allow system flexibility. Viable option for no OS wide ROS install, or for working with multiple ROS versions. See [Working with Docker](#working-with-docker) for details. 

## ROS workspace
```bash
# create a catkin workspace
mkdir -p ~/MiRco/ws/src \
&& cd ~/MiRco/ws/src

# clone mir_robot, robotiq_2f85 and this repository into the catkin workspace
git clone -b noetic https://github.com/JanJericevic/mir_robot.git \
&& git clone -b noetic-devel https://github.com/JanJericevic/robotiq_2f85.git \
&& git clone -b main https://github.com/JanJericevic/MiRco_robot.git

# update and install packages
sudo apt update \
&& sudo apt upgrade -y \
&& sudo apt install -y --no-install-recommends python3-catkin-lint python3-catkin-tools ros-noetic-moveit ros-noetic-ur-robot-driver ros-noetic-ur-calibration ros-noetic-ur-calibration-dbgsym

# use rosdep to install all dependencies
cd ~/MiRco/ws/ \
&& rosdep update \
&& rosdep install --from-paths src -i -y --rosdistro noetic 

# build all the packages in the catkin workspace
source /opt/ros/noetic/setup.bash \
&& cd ~/MiRco/ws/src \
&& catkin_init_workspace \
&& cd ~/MiRco/ws/ \
&& catkin_make -DCMAKE_BUILD_TYPE=RelWithDebugInfo

# source the workspace and add it to the .bashrc
source ~/MiRco/ws/devel/setup.bash \
&& echo "source ~/MiRco/ws/devel/setup.bash" >> ~/.bashrc
```

## Working with Docker
Docker containers allow you flexibility in your setup:
- reproducibility of environments/projects
- no need for OS wide ROS install
- possibility of switching between ROS versions 

### Install Docker engine
Full install instructions are available at Dockers [official website](https://docs.docker.com/engine/install/ubuntu/).
Make sure old versions od Docker engine are [uninstalled](https://docs.docker.com/engine/install/ubuntu/#uninstall-docker-engine).

#### Optional Docker post installation steps
All optional installation steps are available at Dockers [official website](https://docs.docker.com/engine/install/linux-postinstall/).  
Useful post installation step is to manage Docker as a non sudo user. This allow the omission of `sudo` in front of docker commands. When allowing this make sure to be aware of how this [impacts security of your system](https://docs.docker.com/engine/security/#docker-daemon-attack-surface).

### Building the image
If all you want is to connect to the MiR100 `roscore` for monitoring all you need is a ROS Docker image.  
We will build a custom ROS Docker image, complete with the same ROS packages as a local install, so you have a choice of running the project locally or using Docker containers.

```bash
# download the desired packages to the src of your workspace
mkdir -p ~/MiRco/ws/src \
&& cd ~/MiRco/ws/src

git clone -b noetic https://github.com/JanJericevic/mir_robot.git \
&& git clone -b noetic-devel https://github.com/JanJericevic/robotiq_2f85.git \
&& git clone -b main https://github.com/JanJericevic/MiRco_robot.git

# move the Dockerfile and docker-compose.yaml in MiRco_robot folder to the src folder
mv ~/MiRco/ws/src/MiRco_robot/Dockerfile ~/MiRco/ws/src \
&& mv ~/MiRco/ws/src/MiRco_robot/docker-compose.yaml ~/MiRco/ws/src/docker-compose.yaml

# build the Docker image
cd ~/MiRco/ws/src
docker build -t mirco_docker --build-arg MYUID=$(id -u) --build-arg MYGID=$(id -g) --build-arg MYUSER=$(id -nu) --build-arg MYGROUP=$(id -ng) .

# list your built Docker images
# verify that your <image-name> is among the listed images
docker images

```
!!! note
    If you're on a machine with no OS wide ROS install and don't have a `catkin ws` the steps remain the same. Move the `Dockerfile` to the root directory of your packages, then build the Docker image in that root directory. The build commands remain the same.

To avoid permissions issues with shared files between the host computer and the image container, we create a user with `sudo` permissions inside the image (this is especially relevant during [development](#volume-mounting)). User profile can be changed when building the image (the `build-arg` listed above) and inside the `Dockerfile`.  
The current profile settings are: 

> ***username***: same as the host username that built the image  
> ***password***: same as the username

The `Dockerfile` creates a `catkin workspace` at `/home/<your-user>/ws` inside the image. The workspace is also set as the work directory of the image so it will be the starting point of every new container.

Depending on your use case you will use the Docker image during development (you plan to regularly modify your codebase) or you will only use it for deployment. For deployment you only need to copy your files once, which is what we have done until now. For a development setup see the [volume mounting](#volume-mounting) section.

### Running the image
You can start a container with the MiRco image using the `docker-compose.yaml` file.

```bash
# start the docker container in detached mode
cd ~/MiRco/ws/src \
&& docker compose up -d \

# open a terminal in the container
docker exec -it mirco_container bash
```

This Docker compose configuration enables:

- GUI usage inside the container (X11 forwarding)
- NVIDIA GPU utilization (host needs NVIDIA Container Toolkit)
- device usage inside the container (for joysticks)
- mounts the src folder of your workspace directory as a volume for persistent memory during development  

Check the `docker-compose.yaml` file for details how each function is enabled.  
If these functions are not needed, configure the `docker-compose.yaml` to your needs.

#### GUI applications
If you have problems using GUI applications inside the container try to change access permissions to the X server. The easiest is to grant access to everyone, or you can [specify](https://manpages.ubuntu.com/manpages/lunar/en/man1/xhost.1.html) a specific user.

```bash
# disables access control
xhost +

# grants access to specific user
xhost +local:<user-name>
# or
xhost +SI:local:<user-name>
```

Changes to the X server access only persist until the next logout/login, but it is best practice to enable back the access control once you're finished working with the container.

```bash
# enables access control
xhost -
```

#### NVIDIA
Users of NVIDIA GPUs can download the [NVIDIA Container Toolkit](https://github.com/NVIDIA/nvidia-container-toolkit), which allows them to build GPU accelerated containers or in some cases solves display issues if the above mentioned setup is not working. The "Graphics inside Docker Containers" paragraph of this [ ROS&Docker guide](https://roboticseabass.com/2021/04/21/docker-and-ros/) describes working with such images.

#### Volume mounting
For development we want persistent files that are shared between the host machine and the containers, files that can be changed both from the host side and from inside the container, in a setup which does not require us to rebuild the image every time we modify our code. We achieve this using [volumes](https://docs.docker.com/storage/volumes/). To achieve the necessary permissions we create a `sudo` user inside the image (see "[Building the image](#building-the-image)" section).

!!! note
    Mounted files are available only at runtime. Any files needed for building the image should be copied before then.

#### Input devices
Some input devices require you to change their permissions so that they become accessible to your application (e.g. ["Configuring the Joystick"](http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick) section of this ROS tutorial). To avoid repetative changing of permissions every time you plug them in, you can create a `udev` rule.

```bash
# with the desired input device unplugged
# list the input devices on your host
$ ls /dev/input
by-id    event0  event10  event3  event5  event7  event9  mouse0  mouse2
by-path  event1  event11  event2   event4  event6  event8  mice    mouse1

# plug in the desired input device
# again list the input devices to see your device name
$ ls /dev/input
by-id    event1   event20  event5  event8  mice    mouse2
by-path  event10  event19  event3   event6  event9  mouse0
event0   event11  event2   event4   event7  js0     mouse1
```

Using the above method, we find that our device name is `js0`. Now we create a `udev` rule. Add a file `/etc/udev/rules.d/99-userdev-input.rules` with:

```bash
KERNEL=="js0", SUBSYSTEM=="input", ACTION=="add", RUN+="/usr/bin/setfacl -m o:rw $env{DEVNAME}"
```

This udev rule uses `ACL` to set the read-write permissions of the input device `js0` for the `others` group. You can modify the rule using `ACL` commands.