# MINIRYS RQT
Plugins for managing Mini Rys robot in RQT


## INSTALL
Tested on:  
* **UBUNTU 22.04**
* **ROS2 HUMBLE**


Before run install
colcon
```
sudo apt install python3-colcon-common-extensions   
```
git
```
sudo apt install git-all
```
pip3
```
sudo apt install python3-pip   
```
setuptools
```
pip3 install setuptools==58.2.0  
```

Then  
```
# create directory
mkdir -p rys-rqt/src
# go into directory
cd rys-rqt/src
# clone repo
git clone https://github.com/GroupOfRobots/MiniRysRQT
# go into repository
cd MiniRysRQT
# init submodule    
git submodule init   
git submodule update  
# go back to workspace
cd ../..
# add source
source /opt/ros/humble/setup.bash
# build
colcon build
# add new source  
source install/setup.bash  
# run rqt  
rqt --force-discover  
```
