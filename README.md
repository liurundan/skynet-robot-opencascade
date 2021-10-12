# skynet-robot-opencascade
simulation platform for robot use opencascade
1、开源仿真项目opencascade

下载地址：Release more advanched example · grotius-cnc/skynet_robot_control_rtos_ethercat；
或者是：https://github.com/grotius-cnc/Skynet_Robot_Control_Rtos_Ethercat；

2、实时操作系统Debian+rtpreempt+ighEthercat
下载地址：https://github.com/grotius-cnc/linux_rtos；
下载安装系统后，如果安装过程联网且设定正确的软件源，则会联网安装若干库，若没有则需按如下步骤安装依赖库：

2.1 设定系统的软件源
sudo vi /etc/apt/sources.list，
备份原有文件，后删除全部，并添加如下内容：

deb http://mirrors.aliyun.com/debian/ buster main non-free contrib

deb-src http://mirrors.aliyun.com/debian/ buster main non-free contrib

deb http://mirrors.aliyun.com/debian-security buster/updates main

deb-src http://mirrors.aliyun.com/debian-security buster/updates main

deb http://mirrors.aliyun.com/debian/ buster-updates main non-free contrib

deb-src http://mirrors.aliyun.com/debian/ buster-updates main non-free contrib

deb http://mirrors.aliyun.com/debian/ buster-backports main non-free contrib

deb-src http://mirrors.aliyun.com/debian/ buster-backports main non-free contrib
使用软件源安装软件，可以安装对应的依赖库。如果使用安装包，不一定能成功；

2.2 安装依赖库
使用sudo apt-get install 安装如下最新的基础库
Cmake， Eigen3， Doxygen，Xmu

sudo apt-get install cmake

sudo apt-get install libeigen3-dev

sudo apt-get install libxmu-dev
并下载 tk8.6.11.1-src.tar.gz 和 tcl8.6.11-src.tar.gz，进行编译安装；
下载网站：http://www.tcl-lang.org/software/tcltk/download.html
