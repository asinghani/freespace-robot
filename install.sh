#!/usr/bin/env bash

sudo -v

export ROS_DISTRO=kinetic

wstool init src
wstool merge -t src src/deps.rosinstall
wstool update -t src

sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y -r

