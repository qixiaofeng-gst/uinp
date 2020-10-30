RL 团队量化周报 —— 仿真环境的性能评价（草稿 v0.1）

各位 RL 团队的同仁，我又来了。淼神之前回复希望第一次周报做这个 “仿真环境的性能评价” 的问题，我就这个问题写了一版草稿。   
各位帮忙审一审，欢迎反驳、提供新视角、提供更多细节，单纯地打脸也是欢迎的，唯一怕的是石沉大海。   
任何文本都可以审，哪里不对请都指出，尤其是小唐唐和韩哥，多用你们迥异的、专业的视角帮助我，辛苦了。   
分割线后是周报正文的草稿，所有数值尚未填充，因此全是占位的 0 值。在大家反馈意见后我会再更新草稿，有一个相对稳定的版本之后再收集真实数值。

-------
我们通常说起一个仿真环境 A ”行“ 或是 B ”不行“，说的是针对 RL 的某个训练任务，在 A 环境中需要的时间长度远小于 B 环境，因此行不行是比较得出的结论。   
这可以提供一个我们设计打分制度的思路：
- 设置一个环境 R 作为参照，将它的各项指标设为 1 分。
- 待评价的环境用 X 代指，X 各项指标对 R 的相应指标的比值即 X 该项指标的分值。

评价的一些限制：
- 不同硬件配置的机器上，得到的评分是不一样的，我们就某一套硬件配置对仿真环境进行评估的结果，用在其他硬件配置上是不准确的。
因此这个周报中得出的评分只针对我做评价时的硬件配置有效。
- 仿真环境内核的编程语言通常是 C/C++，在 RL 使用时通常外面会用 python 封装一层便于使用，因此我们这里评价的都是带有 python 封装层的仿真环境，
不会拿 python 封装过的和仅有 C/C++ 实现的仿真环境去对比。

在这里我们要量化 ”行“（性能好）这个性质，从下面几个方面来分别对比：
- 时间花费：
  - 单一环境执行一个 step 的时间花费，使用多次（比如 10000 次） step 的统计的平均值。
  - 每增加一个并行执行的环境，平均的 step 的时间花费的变化。
- 空间花费：
  - 单一环境运行期间（比如 10000 次 step 运行中）的内存占用的平均值，多次运行（比如 100 次）的平均值的平均值。
  - 每增加一个并行执行的环境，平均的内存占用的变化。

以上是单纯陈述量化方式，下面用我们用过的两个环境 pybullet 和 raisim 来作为例子。   
将 pybullet 当作参照，用来评估 raisim 的性能。
pybullet：
- 时间花费：
  - 单环境 step 平均：0.0ns。
  - 并行增量：0.0ns。
- 空间花费：
  - 单环境平均：0.0MB。
  - 并行增量：0.0MB。

raisim：
- 时间花费：
  - 单环境 step 平均：0.0ns。0分。
  - 并行增量：0.0ns。0分。
- 空间花费：
  - 单环境平均：0.0MB。0分。
  - 并行增量：0.0MB。0分

-------
其它方面的量化（以后的量化周报备选）：
- 仿真环境的“真实性”衡量，真实性依赖于对真实环境的建模，对于不同方面有不同模型，比如地面摩擦、空气摩擦、运动学、动力学等等。
- RL 算法针对不同目标的训练效率评估。
- 评估不同 RL 算法的上限，通过模型能够达到的控制效果来评估。
- 团队开发的工程化程度评估。从需求管理、任务追踪、代码开发、代码库管理、发布库管理、测试系统这些方面抽取更多细节，评估权重，设计数计算方式。

-------
地形 height map generator:
https://fralonra.github.io/zatlas/

-------
https://github.com/jw1401/PPO-Tensorflow-2.0
https://github.com/fomorians/ppo

Proposal: 将分支都合并起来？

macOS SMB connecting: finder -> menu 'Go' -> last item 'connect to server'
_______
webrtcvad: audio segmentation
给正常语音添加噪音 https://zhuanlan.zhihu.com/p/98960359

_______
播放科大讯飞6麦克风 SDK 录制的原始音频：ffplay -autoexit -f s32le -ar 16000 -ac 8 mic_demo_vvui_ori.pcm
去噪过的：ffplay -autoexit -f s16le -ar 16000 mic_demo_vvui_deno.pcm

black box of miao.yang: 100.50

PID: Proportional Integral Derivative
MPC: Model Predictive Control
RPC: Rate Predictive Control. It is model free. Adaptive to change, less oscillation.

_______
- [Planned] 绘制完整的基于模仿学习的动作控制的流程图，并发起讨论。
  - 起点是 AI4Animation 的方案。
  - 起点是路径规划的方案。

=======
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
