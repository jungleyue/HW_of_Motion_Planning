# 使用方法
创建一个工作空间，在src文件中放入这些文件，执行cm,然后执行 roslaunch waypoint_trajectory_generator test.launch 即可，
在rviz界面中选择3D工具(快捷键G)选择路径点，即可出现相应效果展示,绿色轨迹为默认将选择点以线段连接，红色为用闭式求解算法求出来的最优轨迹。
默认起点为原点，终止选点时选择z轴值为负的点即可。

![效果展示](~/a.gif)
