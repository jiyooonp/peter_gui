FROM nvidia/cuda:12.1.0-devel-ubuntu20.04
CMD nvidia-smi


# Minimal setup
RUN apt-get update \
 && apt-get install -y locales lsb-release
ARG DEBIAN_FRONTEND=noninteractive
RUN dpkg-reconfigure locales
 
# Install ROS Noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update \
 && apt-get install -y --no-install-recommends ros-noetic-desktop-full
RUN apt-get install -y --no-install-recommends python3-rosdep
RUN rosdep init \
 && rosdep fix-permissions \
 && rosdep update
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y git && apt update
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# to have same alias as in local laptop
RUN echo "# alias " >> ~/.bashrc
RUN echo "alias sb='source ~/.bashrc' " >> ~/.bashrc
RUN echo "alias eb='gedit ~/.bashrc' " >> ~/.bashrc
RUN echo "alias cm='cd ~/catkin_ws/ && catkin_make' " >> ~/.bashrc
RUN echo "alias cb='cd ~/catkin_ws/ && catkin build' " >> ~/.bashrc
RUN echo "alias u='sudo apt update && sudo apt upgrade'" >> ~/.bashrc
RUN echo "alias cw='cd ~/catkin_ws/ '" >> ~/.bashrc
RUN echo "alias c='clear'" >> ~/.bashrc
RUN source ~/.bashrc

RUN sudo apt-get -y update && sudo apt-get -y upgrade 
# for ease of use 
RUN apt-get install -y gedit 
# cv related 

RUN apt-get -y install python3
RUN apt-get -y install python3-pip
RUN pip3 install torchvision 
RUN pip3 install matplotlib==3.7.0
RUN pip3 install opencv-python==4.8.1.78
RUN pip3 install numpy==1.20.0
RUN pip3 install scipy==1.10.0
RUN pip3 install scikit-image==0.20.0
RUN pip3 install scikit-learn==1.3.0
RUN pip3 install pandas==2.0.0
RUN apt-get install -y python3-tk
RUN pip3 install shapely==2.0.0
RUN pip install ultralytics==8.0.199
RUN pip install pyrealsense2==2.54.2.5684
RUN pip install termcolor==2.3.0
RUN pip install ipdb

# for xarm 
RUN apt-get -y install ros-noetic-moveit-visual-tools
RUN apt install ros-noetic-combined-robot-hw

RUN mkdir -p ~/catkin_ws/src 
RUN cd ~/catkin_ws 
RUN . /opt/ros/noetic/setup.sh && cd ~/catkin_ws/ && catkin_make

RUN sudo apt-get install -y ros-noetic-realsense2-camera
# RUN git clone -b pepper-people-devel https://github.com/GetAGrip-dot-Ai/realsense-ros.git &&\
# cd realsense-ros/ &&\
# git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1` &&\
# cd ..

RUN pip3 install conan==1.59 && conan config set general.revisions_enabled=1 && conan profile new default --detect > /dev/null && conan profile update settings.compiler.libcxx=libstdc++11 default 
# RUN cd ~/catkin_ws/src && git clone https://github.com/GetAGrip-dot-Ai/fvd_ws.git
# RUN cd ~/catkin_ws && rosdep install --from-paths src --ignore-src -y 

RUN echo "source ~/catkin_ws/devel/setup.bash " >> ~/.bashrc &&source ~/.bashrc


# BUILD source 
RUN source /opt/ros/noetic/setup.bash

# Installing MoveIt
RUN sudo apt-get update
RUN sudo apt-get install -y ros-noetic-moveit

RUN apt-get -y update && apt-get -y upgrade && apt-get -y dist-upgrade
RUN apt-get install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at
RUN apt-get install ros-noetic-moveit ros-noetic-moveit-plugins ros-noetic-moveit-planners
RUN apt-get install -y ros-noetic-moveit-servo

RUN apt-get install -y ros-noetic-joy

RUN source /opt/ros/noetic/setup.bash

RUN cw && git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
RUN cw && git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench.git
RUN cw && git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git

RUN cw && git clone git clone https://github.com/xArm-Developer/xArm-Python-SDK.git && cd xArm-Python-SDK/ && python3 setup.py install 

CMD cm
