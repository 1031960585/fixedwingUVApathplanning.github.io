## 地图的原点为二教西北角
- 经纬度 (117.38500，39.54239)
- UTM (533081.8341,4377039.7674)
- map左下角到原点的偏移量 (346.91,194.61)
- gazebo飞机位置到世界原点的偏移量 (-312，-229)
- gazebo世界原点的UTM (533393.8341,4377268.7674)
- gazebo世界原点的GPS (117.38864，39.54444)

# 启动仿真程序：

```
roslaunch px4 tianyuan_plane_0.launch 
```

启动通信脚本：0表示0号固定翼

```
cd ~/XTDrone/communication/ 
python plane_id_communication.py 0
```

启动键盘控制脚本：1表示一架固定翼

```
cd ~/XTDrone/control/keyboard
python plane_keyboard_control.py 1
```

混合A*算法

```
roslaunch hybrid_a_star run_hybrid_a_star.launch
```
```
roslaunch hybrid_a_star init_pose.launch namespace:=/plane_0
```
```
roslaunch hybrid_a_star path_hd.launch namespace:=/plane_0
```

# 没有命名空间仿真：

```
roslaunch px4 tianyuan.launch
```

启动通信脚本：0表示0号固定翼

```
cd ~/XTDrone/communication/ 
python plane_communication.py 0
```

启动键盘控制脚本：1表示一架固定翼

```
cd ~/XTDrone/control/keyboard
python plane_keyboard_control.py 1
```
混合A*算法

```
roslaunch hybrid_a_star run_hybrid_a_star.launch
```
```
roslaunch hybrid_a_star init_pose.launch
```
```
roslaunch hybrid_a_star path_hd.launch
```

固定翼需要用takeoff模式起飞。如果不希望有影子，可以吧Gazebo scene里的shadows关掉.

![image.png](https://cdn.nlark.com/yuque/0/2020/png/985678/1589461687393-e81b3d92-6ff3-48c0-b22c-b89a1f6b614f.png?x-oss-process=image%2Fresize%2Cw_750%2Climit_0)

固定翼的offboard模式，通过设置航点位置，飞机会先飞到航点(setpoint)附近，然后盘旋。

注意：1.切Offboard之前先s，这样才会飞到航点盘旋，如果是按k，飞机会无动力滑翔。2.设置航点(setpoint)高度尽量超过20m，否则有可能撞地面。

**推荐起飞流程：**v起飞->t解锁->先设置航点位置然后o载入航点->s盘旋->b切换offboard模式



