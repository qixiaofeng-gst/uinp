RL 团队量化周报 —— 量化 AI 控制的稳定性（草稿 v0.1）

各位 RL 团队的同仁，我这里写了个量化周报的草稿，各位审一审，欢迎反驳、提供新视角、提供更多细节，单纯地打脸也是欢迎的，唯一怕的是石沉大海。
任何文本都可以审，哪里不对请都指出，尤其是小唐唐和韩哥，多用你们迥异的、专业的视角帮助我，辛苦了。
分割线后是周报正文。

-------
首先限制一下这个问题的讨论范围：
* 仅限于讨论基于强化学习的模仿学习，模仿目标限制在动作捕捉产生的参考动作（学习对象）。
  * 模仿目标还有其他获取办法，比如程序生成、人工绘制、从视频流人工标注、从视频流 AI 识别等，这些获取办法生成的模仿目标不在目前的讨论范围。
* 仅限于讨论稳定性，稳定性只是 AI 控制的质量的较为具体一个方面。
  * 评价 AI 控制的质量还有其他方面，比如与被学习对象相似程度、自然程度、能耗、对不同环境的适应能力等，这些方面不在讨论范围。

先泛泛地定义一下稳定性：控制💹时间（从开始控制到失控的时间）长，控制期间产生的抖动（抖动包括肢体周期性颤动、肢体运动速度突变）小。
稳定性应该是我们评价一个 AI 模型的控制效果的优先级比较高的指标。
既然尝试量化，我们应该首先确定一个打分制度，这里暂定单项满分 100，汇总满分也是 100，后面讨论的每一单项都会给出：
* 满分标准
* 从 0 到满分的评价曲线，暂不设负分
* 汇总时的权重，整型数，权重最小者为 1

下面开始尝试量化。

评价指标：
* 控制时长。依据模仿目标“是否可循环”这个特征分为两个部分：
  1. 可循环的模仿目标
     * 满分标准：有效的（没失控的，比如行走就是没摔倒的）控制的循环次数达到 100。
     * 评价曲线：循环次数达到 20，分数达到 80，其间者线性相关；循环次数达到 100 则评分 100，区间线性相关。
     * 汇总权重：2
  2. 单程的模仿目标
     * 满分标准：从等待动作（标准的站立动作）开始执行一轮控制到恢复到等待动作，整个过程中没有失控。
     * 评价曲线：依据失控时的进度百分比打分，线性相关。比如失控的时候，动作做到了 50%，则为 50 分。
     * 汇总权重：2
* 位置偏差
  * 满分标准：在控制期间，每一步控制的预期位置和实际位置的距离的标准差为 0。
  * 评价曲线：标准差为 1 时 0 分，线性相关。
  * 汇总权重：1
* 速度偏差
  * 满分标准：在控制期间，每一步控制的预期速度（标量 m/s 的速度，无方向）和实际速度的差值的标准差为 0。
  * 评价曲线：标准差为 1 时 0 分，线性相关。
  * 汇总权重：1
* 机身朝向偏差
  * 满分标准：在控制期间，每一步控制的预期朝向（四元组）和实际朝向的差值（四元组减法得出的差值，描述的是两个旋转之间的旋转）的模的标准差为 0。
  * 评价曲线：标准差为 1 时 0 分，线性相关。
  * 汇总权重：1

-------
其它方面的量化（以后的量化周报备选）：
- 仿真环境的性能（计算资源占用，资源包括计算能力、时间、空间）评估。
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
