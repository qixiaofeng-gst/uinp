2020-6-16：
- [In-progress] motion-runner 和真机联调。先把狗子在模拟器中的行动调试正确。
  - 通过 motion-runner 与模拟器连接运行时，狗子一直原地打转。
  - 刘博士之前是用的 dacong 嵌入的 python-interpreter 直接载入脚本执行。
  - motion-runner 是基于 LCM 与模拟器通信。
  - 刘博士切换到 motion-runner 调试后发现，通过 LCM 接收到的关节角和关节角速度全都是 1。
  - 唐彬确认了是模拟器没有发送 spi_data，我们一起给模拟器打了发送 spi_data 的临时补丁。
  - 打完补丁后发现，狗子可以前进了，但是步频很高，且动作幅度很小，腿像是在抖动，而不是步行。
- [Note] 使用 LCM 方式运行脚本时（在 motion-runner 中运行），狗子步频非常高，而且腿的动作幅度很小。
  - 淼神、唐彬、刘博士和我简单碰了一下，淼神总结了三个可能的原因。
  - 一，LCM 通信延时导致了问题。这一条需要 LCM 消息里携带时间戳，模拟器和 AI 控制器两方都检查一下具体延时是多少。
    - 在 motion-runner 端的现象：数据接收的延时很小，在微秒级；但是控制器脚本中取用数据的时候，延时会达到 10 毫秒级别。
    - 接收数据的频率接近 500Hz（2.095～2.130 秒接收 1000 条数据），因此接收间隔是 2 毫秒。
    - 控制器脚本每个 step 耗时在 1~6 毫秒。
    - 由于接收数据是在单独的线程中，为了消除线程间读写变量对延时的影响，使用了读写锁，但对最终效果并没有太多影响。
    - 取用数据的延时有明显的周期性，会从微秒级增长到毫秒级，达到 10 毫秒级时会跳变回微秒级，然后再增长，如此反复。
  - 二，模拟器收到控制器的指令之后对指令的处理方式导致了问题。
    - 模拟器控制频率是 500Hz，AI 控制器的发送频率是 100Hz。
    - 要查一下模拟器接收指令后，在没收到下一条指令前，具体的处理逻辑。
  - 三，模拟器发送的 IMU 数据有误。
    - 之前狗子在模拟器中原地转圈的问题，是由于模拟器没有发送 spi_data 导致的。
    - 同理，IMU 也可能没有被发送。（本条已排除，唐彬已确认 IMU 数据是有发送的）

corpus：语料库
triplet loss：三重损失
acoustic：声学
utterrance：发声
phoneme：音素
Mandarin：普通话
End-to-end describes a process that takes a system or service from beginning to end
  and delivers a complete functional solution,
  usually without needing to obtain anything from a third party.
Speaker diarisation is the process of partitioning an input audio stream
  into homogeneous segments according to the speaker identity.
et al.：means 'and others'.
affine：仿射


epoch, batch, iteration, episode:

epoch: pass of entire training dataset.
batch: sliced training dataset.
iteration: pass of batch.
episode: one game. episodes are completely independent of one another.

_______ _______
new nuc: hachi@192.168.100.201, normal

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
