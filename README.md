# Px4+MAVROS键盘控制

实现键盘控制基于px4的无人机飞行

# 一、启动方法

**启动px4**

在PX4的目录下，运行px4+gazebo的联合仿真，输入以下命令：

**make px4\_sitl gazebo**

**连接MavROS和PX4**

打卡新的终端输入：

**roslaunch mavros px4.launch fcu\_url:="udp://:14540@127.0.0.1:14557"**

连接PX4和mavros，此时输入rostopic list可以看到mavros的话题，当前mavros的话题结果是反应当前PX4无人机的状态。

![image](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/res/MeYVOLGkxkEAqpz2/img/e6581763-defd-44e3-89a9-79067748e595.png)

**打开px4键盘控制节点**

1.  首先新建一个工作空间并编译：
    

**mkdir -p Px4Control\_ws/src**

**cd Px4Control\_ws**

**catkin\_make**

2.  将附件中的功能包放入 Px4Control\_ws/src 目录下再对该功能包进行编译
    

**catkin\_make**

3.  运行
    

1.  进入root权限，由于当前是键盘控制节点，需要读取外设，因此需要进入root权限，输入命令：
    

**sudo su**

2.  运行键盘控制节点
    

**source ./devel/setup.bash**

**rosrun px4\_keyboard\_control px4\_keyboard\_control\_node**

如果没有报错应该可以正常运行

# 二、操作说明

该部分主要是对无人机控制进行说明，也顺便对键位控制的代码简单讲解。

Q ———— 无人机上升

该键位主要控制增加无人机z方向上的线加速度，从而实现上升的效果。

![image](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/res/MeYVOLGkxkEAqpz2/img/8463f3d5-fdf9-4e08-9071-4dba231daf77.png)

A ———— 无人机停止升降

该键位主要将无人机z方向上的线速度归零，以达到无人机停止上升的效果。

![image](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/res/MeYVOLGkxkEAqpz2/img/96bae86a-f031-4577-a93f-cd321e95b2fd.png)

Z—————无人机下降

原理同Q

![image](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/res/MeYVOLGkxkEAqpz2/img/2129333b-b179-4ae1-a7d4-6e4bcb72a67d.png)

J ——————无人机向左运动

![image](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/res/MeYVOLGkxkEAqpz2/img/b04908d8-68a0-469f-9838-c4578bb041cf.png)

L——————无人机向右运动

![image](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/res/MeYVOLGkxkEAqpz2/img/f3714758-66cb-4cbc-b079-796aa9221cfe.png)

I——————无人机向前运动

![image](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/res/MeYVOLGkxkEAqpz2/img/2f457b5a-10f3-47f1-a4c7-4b63e7de23cd.png)

K——————无人机停止运动

![image](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/res/MeYVOLGkxkEAqpz2/img/43f56651-c516-438b-a2d3-314b1e4ef3d6.png)

，——————无人机向后运动

![image](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/res/MeYVOLGkxkEAqpz2/img/62ae883c-e8b6-47de-ac6f-2515f9664776.png)

U——————顺时针转向

主要改变角加速度的航向角

![image](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/res/MeYVOLGkxkEAqpz2/img/996fb7e5-1d4e-41f2-85a8-15fbdb126302.png)

O——————逆时针转向

![image](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/res/MeYVOLGkxkEAqpz2/img/f7711b06-1278-4df1-850c-2739d367533e.png)

H——————线加速度和角加速度归零

![image](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/res/MeYVOLGkxkEAqpz2/img/905bb0c4-653d-40fb-8af9-c7c6e723946f.png)

N———————控制姿态

![image](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/res/MeYVOLGkxkEAqpz2/img/2e6a51eb-be75-4278-82d0-99d5bd9ec3c5.png)

Y———————从控制cmd\_vel模式转换到控制pose模式

![image](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/res/MeYVOLGkxkEAqpz2/img/164b1854-da94-44f5-a425-4288be50476f.png)

T————————外部控制启用

![image](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/res/MeYVOLGkxkEAqpz2/img/02a11363-6ac8-47ff-954d-34158de0040e.png)

G————————无人机解锁

![image](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/res/MeYVOLGkxkEAqpz2/img/5a311eac-54c3-4408-b8d6-f07c4bd0a054.png)

B————————无人机锁定

![image](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/res/MeYVOLGkxkEAqpz2/img/4aca388a-18f1-49a6-ae5a-7aa4f2016d9d.png)

注意：在控制px4仿真的无人机时，需要先按键 T启动外部控制，然后按键G解锁无人机后才能正常控制其飞行。

