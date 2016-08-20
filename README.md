# tracker-dskcf

应用深度相机的Ds-kcf追踪算法，不需要依赖ros环境

Ds-kcf论文链接：http://www.cs.bris.ac.uk/Publications/Papers/2001755.pdf

# 开发环境：

Ubuntu 14.04

# 依赖：

## openni2 

下载官方openni2库,链接地址：http://structure.io/openni
![image](http://git.aiiage.com:9000/tian.y/tracker-dskcf/uploads/3948514d29e2aa98e0da79da2b7b2666/Screenshot_from_2016-08-20_11_37_43.png)
根据系统选择所需要的版本，这里我选择了 **OpenNI 2.2.0.33 Beta (x64)**.

下载完成后解压到任意目录，打开文件夹，运行：
```
sudo ./install.sh
```
将生成的环境变量文件（OpenNIDevEnvironment）内的环境变量加入到系统环境变量：
```
gedit ~/.bashrc
```
将文件OpenNIDevEnvironment内的以下内容拷贝到.bashrc文件内：

export OPENNI2_INCLUDE=/home/exbot/Downloads/OpenNI-Linux-x64-2.2/Include

export OPENNI2_REDIST=/home/exbot/Downloads/OpenNI-Linux-x64-2.2/Redist

生效环境变量：
```
source ~/.bashrc
```
## opencv

opencv安装过程详见：http://git.aiiage.com:9000/algorithm/dev-doc/blob/master/OpenCV.md

# 传感器：

深度相机（Xtion pro live）

# 实验平台：

Aiibot

# 运行

## 运行前为了通过串口向移动平台发送速度，需要修改串口权限:
```
sudo chmod 666 /dev/ttyUSB0
```
## 编译：
```
mkdir bin
mkdir build
cd build
cmake ..
make
```
## 运行程序：
```
./../bin/main
```
## 使用程序：

程序启动后，在图像窗口内鼠标左键框选所要跟踪的目标.

## 速度规划：

|参数| 数值（单位m）|
|:----:| -------------|
|Min_distance | 1.5|
|Max_distance | 5.0|
|Min_linear_speed | 0.4|
|Max_linear_speed | 0.6|
|Min_rotation_speed | 0|
|Max_rotation_speed | 0.75|

目标距离相机1.5m时开始跟踪，初始速度0.4m/s，速度随着距离的增大而增加，
最大距离5m时速度增加到0.6m/s，超过5m速度恒定为0.6m/s。旋转速度初始为0, 随着目标偏移相机中心点的角度的增大而增加，
最大叫速度为0.75rad/s。速度参数可以在程序主文件main.cpp内修改。

# 针对kcf算法的改进：

Ds-kcf算法框图：
![image](http://git.aiiage.com:9000/tian.y/tracker-dskcf/uploads/9dc0b9f171ef269136f40d8b3bc01352/Screenshot_from_2016-08-20_17_52_47.png)

### 1.基于深度图像计算目标框内目标的深度信息。

原论文应用kmean聚类算法分割目标区域并统计目标深度，本程序应用区间分布原理，
判断目标框内大部分像素点的深度分布并计算其深度值。

代码详见kcftracker.cpp - getDepth（）；

### 2.根据深度信息计算目标框的大小比例。

根据目标距离相机的深度与其大小成反比：

scale = 上一帧目标深度 / 当前帧目标深度。

代码详见kcftracker.cpp - update（）；

### 2.根据深度信息判断遮挡问题的触发条件以及恢复策略。

这部分内容仍在完善中。。。
