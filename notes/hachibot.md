2020-7-7：
- [In-progress] 调查 dacong 和 raisim 间的差异的根源。
  - 第一帧的 action 打印，两边。
  - 发现了 gait fr/fl/rr/rl 的问题，在 motion-runner 中 fl/rr 的值交换了。
  - 修复了 fl/rr 的差异之后，两个模拟器中狗子的动作看起来已经相差不大了。
  - 下面对齐一下更新频率，把 raisim 中的 action 的更新频率限制到 100Hz。
  - 使用 raisim 仿真（打开渲染）时，step 的执行频率达不到 100Hz，通常在 71-77Hz 间浮动（需要 1.3-1.4 秒执行 100 个 step）。
  - 通过降低 raisim 的渲染频率，使 env.step 的频率达到了 100Hz。
- [Planned] 绘制完整的基于模仿学习的动作控制的流程图，并发起讨论。
  - 起点是 AI4Animation 的方案。
  - 起点是路径规划的方案。

Half-weekly Report：   
把 raisim 扭矩控制模式的训练的结果迁移到了 dacong，排除了大部分故障。   
目前 raisim 和 dacong 之间，在姿态、动作频率、动作幅度方面肉眼看起来差异非常小了。   
剩下的一个比较显著的差异是在 raisim 里面狗子的移动距离要远大于 dacong，dacong 里面不管动得多欢，始终像是原地踏步。   
唐彬猜测和两者之间使用的模型有关系，他会找时间把 dacong 的模型回退到和 raisim 一致。然后我们再继续试验。

TODO：   
motion-runner 和 hachy 中的代码，有没有同事有修改需要合并的？今天能合并的就今天合并，如果没有今天能合并的，我就要开始下面的工作了：   
简化 raisim 到 dacong 的迁移过程：
   - 常量、魔法数字全部抽取到配置文件中，使得 hachy 和 motion-runner 可以共用配置。
   - 把所有重复代码抽取出来，作成公用模块。
   - 目标是让训练好的模型只需要指定配置文件和 pkl 文件即完成迁移。

更下一步的计划：
   - 把模仿学习和 dacong 接起来。
   - 前提是 张钰/梦婷/陈晨 到时候还没有成功对接。
   - 如果成功对接了，我会简化对接过程。

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
