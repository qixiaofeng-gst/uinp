2020-5-14：
- [Done] 配合从识别到决策到执行策略的联调。（小车已经可以跟着人跑了。）
- [Done] 配合刘超处理时间同步的问题。
  - 传输、识别的延时导致小车目标位置不准确。
  - 第一个解决方案是由动作识别模块将时间随着处理结果发出，但由于 ROS tf 模块与 python3 版本不兼容的问题导致该方案流产。
  - 第二个解决方案是由路径规划模块提供一个 ROS service，在动作识别模块每次识别开始时调用该 service 同步时间到路径规划模块，成功解决问题。
- 使 motion_imitation 项目使用 minicheetah 的 URDF 进行训练和测试。
  1. [Done] 简单整理 motion_imitation 项目，放到 gitlab。
  2. 用 minicheetah 替换 laikago。
     - [Done] 替换掉了 URDF、代码中的关节相关信息。
     - [In-progress] 初始化时，狗身朝向不对。
- 记一些奇奇怪怪的问题（吐槽之魂无法压抑，但因代码太多暂不准备改进任何问题）：
  - 安装 pybullet 时有些附带 pybullet_data/pybullet_envs 诸如此类，感觉只是作为演示目的加入的，却被依赖了（比如 motion_imitation 就依赖 pybullet_data 里面的 URDF）。
  - motion_imitation 的机器人基类居然是 minitaur，laikago 继承了 minitaur，这就很奇幻了……

```
motion-imitation run.py （共 189 行）调用链：
env = envs.env_builder.build_imitation_env --[use]-> motion_file
model = learning.ppo_imitation.PPOImitation --[use]-> model_file, env, learning.imitation_policies
1. train --[use]-> model.learn
2. test --[use]-> model.predict, env.step

envs/env_builder.py build_imitation_env （共 77 行）调用链：
sensors = envs.sensors.sensor_wrappers.HistoricSensorWrapper
task = envs.env_wrappers.imitation_task.ImitationTask
env = envs.locomotion_gym_env.LocomotionGymEnv --[use]-> robot_class = laikago.Laikago, sensors, task
env = envs.env_wrappers.observation_dictionary_to_array_wrapper
env = envs.env_wrappers.trajectory_generator_wrapper_env
env = envs.env_wrappers.imitation_wrapper_env

envs/env_wrappers/imitation_task.py （共 1238 行）关键方法：
reset, update, done, reward --[call by]-> \__call__

envs/locomotion_gym_env.py （共 460 行）关键方法：
reset, step, render, \_terminate

envs/env_wrappers/observation_dictionary_to_array_wrapper.py （共 74 行）关键方法：
reset, step, render

envs/env_wrappers/trajectory_generator_wrapper_env.py （共 92 行）关键方法：
reset, step

envs/env_wrappers/imitation_wrapper_env （共 153 行）关键方法：
step, reset
```

_______ _______
quick-memo:
- to-har -> har-rsync

2020-4-28 会：
1. 电机驱动器的问题，芯片：温度传感器、位置传感器、电流回馈，有损去噪，是用 raw 数据作为 RL 输入还是用去噪过的？
   张钰：不用，没有差异，刘博士赞同
2. 八字腿是 reward 设计的问题。
3. 输入维度增加可能有用，但目前用处未知。

warning: ‘hachi::PyWrapperingMotionController’
 declared with greater visibility than the type of its field
  ‘hachi::PyWrapperingMotionController::controllerInPy’
see: https://pybind11.readthedocs.io/en/stable/faq.html

连接 mini-cheetah，接网线，PC 关 wifi，手动配置 IP 为 10.0.0.2，子网：255.255.255.0，DNS：10.0.0.1。
然后 ssh user@10.0.0.36，密码 i-b。`cd hachidog/build` 然后 `/bin/bash ./run_mc.sh ./mit_ctrl`。

1. 刘博士提议：数值可视化监控的工具，主要针对狗狗的状态（关节位置、扭矩等）数值。

_______ _______
1 配置文件切换模式，硬件时遥控器切换模式。
  AI 这边的可执行内容在 so 文件里。
2 调试时用 rosbag 录制的所有消息。
  CMake 都保证存在冲突的库依赖放到相互独立的地方。

PD(proportional-derivative) controller: TeX[[ f = k_p (x_{ref}-x) + k_d (v_{ref}-v), k_p = \omega_n^2, k_d = 2 \xi \omega_n ]]

Useful information:
1. pybind11 is the bridge between c++ and python
2. check Qt installation: qtdiag
3. URDF: Unified Robot Description Format; SDF: Simulation Description Format; MJCF: Multi-Joint dynamics with Contact Format
4. Eigen: C++ template library for matrix/verctor/numerical solver, pure header files without binary library
5. Executables for Qt might start correctly with -qt=qt5 as argument

这两天对需求和代码粗浅的理解之后，大概弄了个分层设计（没什么高深的，分个层方便描述、理解和沟通，预期每一层都至少是一个独立工程）：
1. 描述格式适配（输入层）
   - 输入：URDF，SDF，MJCF 等格式
   - 输出：单一格式的机器人描述，比如 URDF，或者我们自定
   - 对不同目标（模拟器或机器人）可能要进行数值调整，因此输出可能针对不同目标生成独立的版本
   - 可能是个工具集合，针对不同格式和目标有不同工具
2. 机器人控制模型训练（训练层，核心工程）
   - 输入：机器人描述，环境描述，训练过的控制模型
   - 输出：训练过的控制模型
   - 预期是 C++ 和 Python 混合的工程
   - 与输入层类似，可能针对不同目标需要不同的训练环境
   - 本层的输入输出在概念上是有不同目标之分的，但是代码逻辑上是不用区分不同目标的
   - 本层的输入输出的内容格式和训练逻辑是稳定的、一致的
   - 模型的训练预期是要依赖某种模拟器的，最好能选一个开源的模拟器
3. 机器人控制模型导出（转译/输出层）
   - 输入：训练过的控制模型
   - 输出：用于控制机器人或模拟器的可执行程序
   - 主要需要将训练过的控制模型针对不同目标进行处理
   - 可能是个工具集合，针对不同目标有不同工具
