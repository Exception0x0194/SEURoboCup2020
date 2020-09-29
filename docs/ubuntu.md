# Ubuntu 下安装教程

以下步骤均在 Ubuntu 20.04.1 测试通过，18.04 下可能存在少量问题。

Ubuntu 安装教程请参考 [Ubuntu20.04的安装教程](./os_install.md)

## ROS2 安装

请主要参考[官方教程](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/)，
但请记得更换源为清华源，即：将`http://packages.ros.org/ros2/ubuntu`替换为`http://mirror.tuna.tsinghua.edu.cn/ros2/ubuntu`

以下是详细步骤：

### 设置软件源

**此处为了便于国内访问，修改了原教程中的PGP公钥地址**

```Shell
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://gitee.com/Sciroccogti/SEURoboCup2020/raw/master/docs/ros.asc | sudo apt-key add -
```

**此处为了便于国内访问，修改了原教程中的软件源地址**
```Shell
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://mirror.tuna.tsinghua.edu.cn/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
```

### ROS2 安装

```Shell
sudo apt update
sudo apt install ros-foxy-desktop
```

若配置正确，数十分钟即可安装完成。

### 工具安装

```Shell
sudo apt update && sudo apt install -y   build-essential   cmake   git   python3-colcon-common-extensions   python3-pip   python3-rosdep   python3-vcstool   wget
pip3 install -U argcomplete
```
