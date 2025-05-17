添加了sim2real的test工程
创建了一个test_sim2real的功能包，生成3个节点
分别为
节点1：发布关节数据话题，暂以只发送左臂数据为测试，分别发送6个关节角度数据，单位为弧度
节点2：订阅发布关节数据话题，收到信息后解析并输出到终端
节点3：通过调用mercury的python函数，将关节数据写入实体机器人

使用方法：
一.sim仿真
1.在Isaac_sim中打开USD文件，注意检查是否开启ROS2的桥接
2.新建终端，进入工作空间colcon build 编译后，source /install/setup.bash
3.执行命令ros2 run test_sim2real publish    ，isaacsim中会正常显示机器人按照接受的关节角运动

二.实体机器人
打开实体机器人，远程连接到机器人ubuntu系统终端，通过python命令行将机器人左臂关节上电，
同时将2关节移动到0度，3关节移动到-90度，防止运行程序时出现关节限位。
1.进入工作空间colcon build 编译后，source /install/setup.bash
2.执行命令ros2 run test_sim2real publish 
3.另外开一个终端，source test_sim2real的/install/setup.bash，执行命令 ros2 run test_sim2real subscriber 
检查是否正常接受关节信息
4.信息正常后按Ctrl+C终止接收，执行命令 ros2 run test_sim2real conrol ,机器人即可订阅关节发布话题运动
