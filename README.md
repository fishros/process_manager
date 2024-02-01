# process_manager

这是一个基于ROS 2的进程管理开源库,是小鱼从实际工程中拆出来的一个小工具，小鱼觉得很不错，就开源提供给大家使用。

这个工具主要用于实际的生产当中进行程序运行的管理，提供:

1. 进程状态监测，运行中还是已经停止。
2. 当你的程序挂掉后会自动帮你重启
3. 可以通过参数动态查询管理的进程，动态进行启动和停止
4. 通过脚本进行日志的生成

对拥有多个模块的系统特别好用哦，利用ROS 2的局域网通信，某些节点可以跑到服务器上，想要调试的时候用这个工具直接把服务器上的节点关掉，运行本地的节点，就可以无缝桥接到系统进行调试。调试效率++++！

具体怎么用呢？

第一步下载工程，直接下载或者克隆就好

第二步修改ProcessList.txt，格式是这样的，每行一个进程，以逗号分割，第一个是程序名字，第二个是运行的命令，小鱼在后面加的一堆是生成日志相关的，后续有时间再优化！

```
talker,ros2 run demo_nodes_cpp talker > $LOGNAME/talker_`date "+%Y%m%d_%H%M%S"`.log 2>&1
listener,ros2 run demo_nodes_cpp listener > $LOGNAME/listener_`date "+%Y%m%d_%H%M%S"`.log 2>&1
```

接着运行 startup.sh 

```
bash  startup.sh 
---
[INFO] [1706788430.207438513] [process_manager]: Started process: talker command=ros2 run demo_nodes_cpp talker > $LOGNAME/talker_`date "+%Y%m%d_%H%M%S"`.log 2>&1 With ID:362449
[INFO] [1706788430.208981086] [process_manager]: Started process: listener command=ros2 run demo_nodes_cpp listener > $LOGNAME/listener_`date "+%Y%m%d_%H%M%S"`.log 2>&1 With ID:362452
```

就可以看到进程启动了

此时查看该节点的参数

```
ros2 param dump /process_manager
---
/process_manager:
  ros__parameters:
    current_status: '{''talker'': ''run'', ''listener'': ''run''}'
    listener: run
    talker: run
    use_sim_time: false
```
可以看到listener和talker都在运行,用命令行停止运行

```
ros2 param set /process_manager talker stop
---
Set parameter successful
```

可以看到进程已经停下来了

```
[INFO] [1706789134.236143928] [process_manager]: Stop talker: PID:362449
```

然后查无此程

```
ps -a | grep talker
```

使用 ros2 param set /process_manager talker run 就可以让进程重新运行起来。

如果是手动杀死这个进程，就会自动重新启动，程序自动退出也会重新启动。

小鱼后面准备写个C++版本，并增加一些功能，但都会尽量保证程序小巧，相关的开源库很多，但太大太难上手，欢迎各位小伙伴测试。