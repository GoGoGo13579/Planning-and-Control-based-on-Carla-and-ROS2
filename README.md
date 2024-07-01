# 简介
使用Carla和ros2使用C++复现了B站up主忠厚老实的老王的规控教程，供朋友们参考。
initial_version是初代版本,之后的规控算法在plus_version里更新
# 软件配置
Ubuntu20.04（用的WSL2）   
ros2-foxy   
Carla-0.9.13  
osqp0.6.3  
osqp-eigen0.8.0   
matplot++最新版就可以,这个和其他是独立的，没啥依赖关系  
这些配置都是安装Carla-ros-bridge时候官方要求的配置，我这里是推荐大家直接使用这一套的，不然会出现各种各样的问题，很浪费时间
# 算法实现
控制算法：纵向串级pid，横向lqr  
规划算法：参考线平滑，路径dp+qp，速度dp+qp
# 使用方法  
我写的源代码在my_planning_and_control这个文件夹里，如果想对算法做出修改，也是改这个文件夹。    
运行案列我是借助的carla_ad_demo(这是carla-ros-bridge里自带的案例)，在它的场景里运行自己规控算法，并没有自己写场景。  
运行指令为    
colcon build  
. install/setup.bash  
ros2 launch carla_ad_demo carla_ad_demo.launch.py  
# 视频展示
https://www.bilibili.com/video/BV1bT421e7CM/?share_source=copy_web&vd_source=c6672cbc9ac3f70466950e7ef1e8855a
