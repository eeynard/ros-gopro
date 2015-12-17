#!/usr/bin/env bash

sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt-key adv --keyserver hkp://pool.sks-keyservers.net:80 --recv-key 0xB01FA116
apt-get update

apt-get install -y libgl1-mesa-dev-lts-utopic

apt-get install -y ros-jade-desktop-full
rosdep init
rosdep update

echo "source /opt/ros/jade/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo -u vagrant echo "source /opt/ros/jade/setup.bash" >> /home/vagrant/.profile

apt-get install -y python-rosinstall

wget https://bootstrap.pypa.io/get-pip.py
python get-pip.py

pip install requests