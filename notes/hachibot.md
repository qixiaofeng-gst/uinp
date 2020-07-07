2020-7-7：
- [In-progress] 比对 dacong 和 raisim 两边的 observation。
  - 我这边的结果记录到了 [wiki](http://wiki.corp.hachibot.com/pages/viewpage.action?pageId=21790761)。
  - 唐彬会帮助对仿真过程的细节进行检查。
  - 淼神建议可以在 raisim 把 action 录制一下，然后在 dacong 里面播放，对比下效果。
  - 第一帧的高度有问题，身体的速度有问题（应该是 0，但是实际有值），第一帧网络输出 abad 值都有问题。
    - 身体速度的问题已经解决。数据的接收线程启动后仿真延迟了 1 秒启动，这样保证一定接收了 observation 数据。
    - 有关 abad 值不对的问题，待修改数值后重新训练，再在两边做对比。
  - 训练的初始值（mean value）有问题（0，-0.5, 1.0），站得太高，不是一个合理的 mean value。
  - 訓練之前的修改：
    - 機身高度 0.29。
    - mean value 修改成为 dacong 中的初始状态。
    - 仿真频率改成 500 Hz。
- [Done] 去除对 raisim 自定义的 PPO2 的依赖，转而依赖 stable_baselines 中原始的 PPO2。
  - 原本 raisimGym 中的 ppo2.py 和 policies.py 都干掉了。
  - 失去了训练时弹出图形界面显示狗子运动表现的能力，因为可以直接使用 launcher.test 模块播放最近保存的 pkl，因此没有该能力影响不大。 
- [Planned] 绘制完整的基于模仿学习的动作控制的流程图，并发起讨论。
  - 起点是 AI4Animation 的方案。
  - 起点是路径规划的方案。

=======
博士：
我看到 motion-runner 里面之前你做的三个 Controller 脚本，有几个地方想和你确认下：
1. action 需不需要乘 -1？（我看你之前做 PPOStand/WalkController 的时候有乘，但到了 PPOMinicheetahController 没有乘）
2. action mean/std 和 obs mean/std 的值在 motion-runner 中和训练时所用的值有没有哪处不一样？
3. 在把 raisim 中的训练结果迁移到 motion-runner 中时，还有哪些东西（数值、计算方法）需要调整？


gpu1-vnc: 123.206.60.196:12301

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
Grafana: admin, normal

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
