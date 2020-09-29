# SEURoboCup2020

东南大学 Robocup Kidsize 2020年校赛代码

遇到问题请先行使用搜索引擎，或查阅 [FAQ](https://gitee.com/robocup/SEURoboCup2020/wikis/)

## 环境配置

- ROS2
- Webots R2020b：[下载链接](https://github.com/cyberbotics/webots/releases/download/R2020b-rev1/webots_2020b-rev1_amd64.deb)（国内下载较慢，可以选择从群内下载）

ROS2 安装教程：
- [Windows 10](docs/windows.md)
- [Ubuntu 18.04/20.04](docs/ubuntu.md)

Webots 安装：
```Shell
sudo dpkg -i path/to/webots.deb # 请自行修改安装文件路径
sudo apt install -f
sudo apt install ros-$ROS_DISTRO-webots-ros2
```

## 使用方法

### 编译

```Shell
git clone https://gitee.com/robocup/SEURoboCup2020
cd SEURoboCup2020
colcon build
```
编译一般在2min以内。

### 初始化

```Shell
sudo rosdep init
# 若报错则参考该教程：https://blog.csdn.net/u013039705/article/details/106651161
rosdep update
echo "export WEBOTS_HOME=/usr/local/webots" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${WEBOTS_HOME}/lib/controller" >> ~/.bashrc
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
ws=`pwd`
echo "source $ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 运行

+ 启动仿真相关的节点  
    新建一个终端，输入：
    ```Shell
    ros2 launch start start_launch.py
    ```

+ 启动比赛控制器  
    新建一个终端，输入：
    ```Shell
    ros2 run gamectrl gamectrl
    ```

+ 启动机器人的控制节点  
    新建一个终端，输入：
    ```Shell
    ros2 launch player player_launch.py color:=red
    # red 为机器人颜色，亦可使用 blue，即
    # ros2 launch player player_launch.py color:=blue
    ```

+ 若修改过 [player.cpp](src/player/src/player.cpp)，则需要重新编译：
    ```Shell
    colcon build
    ```
