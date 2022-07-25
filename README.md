###  navigation 组成及作用

-  amcl：amcl的作用是根据机器人自身的里程数值以及地图特征，利用粒子滤波修正机器人在已知的地图内的位置

-   base_local_planner: 局部路径规划的工具和基本功能模块。 所实现的功能就是给定需要跟踪的轨迹和代价地图的情况下，根据当前的机器人和环境状态，计算用于控制机器人运动的速度指令。

-   dwa_local_planner:使用动态窗口法，DWAPlanner并不具体实现局部路径规划算法。真正的DWA算法在包base_local_planner中实现。 DWAPlannerROS提供了接入move_base框架的接口，DWAPlanner定义了评价轨迹优劣的准则对象。

-   carrot_planner: 很简单的全局路径规划器，生成的路径为目标点到机器人当前点的连线上的点

-   clear_costmap_recovery: 无法规划路径的恢复算法

-   costmap_2d: 代价地图实现

-   fake_localization: 主要用来做定位仿真

-   global_planner:全局路径规划算法包

-   map_server: 提供代价地图的管理服务

-   move_base: 机器人移动导航框架（导航最主要的逻辑框架）

-   move_slow_and_clear:也是一种恢复策略

- nav_core:提供接口，能够实现插件更换算法的主要包

  ​      --base_local_planner

  ​      --base_global_planner

  ​      --recover_behavior

-   nav_fn:全局路径规划算法

-   robot_pose_ekf:综合里程计、GPS、imu数据，通过拓展卡尔曼滤波进行位置估计

-   rotate_recovery:旋转恢复策略实现包

-   voxel_grid:三维代价地图

###  详解

### base_local_planner

[base_local_planner](http://wiki.ros.org/base_local_planner)提供了进行DWA(Dynamic Window Approach)的局部路径规划的工具和基本功能模块。 所实现的功能就是给定需要跟踪的轨迹和代价地图的情况下，根据当前的机器人和环境状态，计算用于控制机器人运动的速度指令。

DWA算法的基本思想是，在机器人的控制空间(线速度、角速度指令)中离散地采样，对于每个样本指令，在机器人当前状态和地图信息的基础上，推演未来很短的一段时间内的运动轨迹。 根据发生碰撞的可能性、目标点的接近程度、全局轨迹的跟踪近似度、速度限制等多方面的指标评价这些轨迹并打分，选取得分最高的轨迹，将其对应的指令下发给底盘，控制机器人运动。

base_local_planner提供了用于生成样本轨迹的SimpleTrajectoryGenerator，评估轨迹优势的各种准则类，比如ObstacleCostFunction等，以及实际进行规划的SimpleScoredSamplingPlanner。 把这些功能模块组合起来，就可以得到一个DWA算法的实现。包[dwa_local_planner](https://gaoyichao.com/Xiaotu/?book=turtlebot&title=dwa_local_planner)中的DWAPlanner就是这样一个示例。

### move_base与插件

move_base要运行起来，需要选择好插件，包括三种插件：base_local_planner、base_global_planner和recovery_behavior，这三种插件都得指定，否则系统会指定默认值。

### base_local_planner插件：

base_local_planner/TrajectoryPlannerROS: 实现了Trajectory Rollout和DWA两种局部规划算法

dwa_local_planner: 实现了DWA局部规划算法，可以看作是base_local_planner的改进版本

### base_global_planner插件：

parrot_planner: 实现了较简单的全局规划算法

navfn/NavfnROS: 实现了Dijkstra和A*全局规划算法

global_planner: 重新实现了Dijkstra和A*全局规划算法,可以看作navfn的改进版

### recovery_behavior插件：

clear_costmap_recovery: 实现了清除代价地图的恢复行为

rotate_recovery: 实现了旋转的恢复行为

move_slow_and_clear: 实现了缓慢移动的恢复行为