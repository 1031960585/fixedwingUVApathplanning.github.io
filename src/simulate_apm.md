## APM仿真：

启动仿真程序：

```
cd ~/ardupilot/ArduPlane
../Tools/autotest/sim_vehicle.py --console --map -L TEST
```

启动mavros

```
roslaunch mavros apm.launch fcu_url:=udp://127.0.0.1:14551@14555
```

混合A*算法

```
roslaunch hybrid_a_star run_hybrid_a_star.launch
```
输入
```
roslaunch hybrid_a_star init_pose.launch
```
输出
```
roslaunch hybrid_a_star path_hd.launch firmware:=apm
```
## 起飞：
```
mode TAKEOFF
arm throttle
```

## APM实际飞行：
启动mavros

```
roslaunch mavros apm.launch fcu_url:="/dev/ttyUSB0:57600"
```

混合A*算法

```
roslaunch hybrid_a_star run_hybrid_a_star.launch
```
输入
```
roslaunch hybrid_a_star init_pose.launch
```
输出
```
roslaunch hybrid_a_star path_hd.launch firmware:=apm
```

