# MINIRYS RQT
Plugins for managing Mini Rys robot in RQT


## INSTALL
Before run install
```
sudo apt install python3-colcon-common-extensions   
sudo apt install python3-pip   
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
source source /opt/ros/humble/setup.bash
# build
colcon build
rqt --force-discover  
```
