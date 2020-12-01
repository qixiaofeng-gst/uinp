* 每次 ROS 节点启动时，要重新抽取注册集数据，避免注册集和测试集抽取时所用的模型不一样导致识别率接近 0。
* 真机再次测试 ivector 方案的识别性能。
  * 讯飞的板子连续工作 20-30 分钟之间宕机了 2 次。

_______
Kaldi - 继续改进方案
- Shell 脚本去重，提取和抽象重复脚本。
- 工具类的 python 脚本归拢到一处。
- 目录结构整理，提取公共部分。

声纹识别 ROS 节点 - 继续改进方案
- 声音采集和识别整合到 C++ 节点中。
- C++ 节点直接依赖 Kaldi，不用每次发起识别都要重启进程。重启进程有两个比较大的代价（耗时）：
  - 重新载入模型。
  - 重新载入注册集。
- 声音采集完直接就在内存中处理掉，有调试的需要时才存储。

_______
本周量化周报 - 理解声纹

声纹是什么？声纹是指声音的特征，音色、音调，音量是不能作为一个特征的，音色和音调对应到频率和分布。
声音在计算机里的本质是时序采样的振幅值序列。

我们基于传统声纹算法

_______
以后的量化周报备选：
- RL 算法针对不同目标的训练效率评估。
- 仿真环境的“真实性”衡量，真实性依赖于对真实环境的建模，对于不同方面有不同模型，比如地面摩擦、空气摩擦、运动学、动力学等等。
- 评估不同 RL 算法的上限，通过模型能够达到的控制效果来评估。
- 将仿真性能、仿真真实性、RL 算法效率、RL 算法效果这四个方面进行综合考虑，对训练项目做总体的量化评估。
- 团队开发的工程化程度评估。从需求管理、任务追踪、代码开发、代码库管理、发布库管理、测试系统这些方面抽取更多细节，评估权重，设计数值计算方式。

_______
2020/12/10 分享，声纹识别
AI 团队采用方案的沿革：
声音采集方案：
- Kinect 麦克风阵列 + ODAS
- 指向性麦克风
- 讯飞 6 麦克风阵列（当前方案）
识别方案：
- speaker-recognition
- deep-speaker
- ge2e
- kaldi（当前方案）

选用讯飞 6 麦克风阵列的主要原因是它的完成度比较高：API 完整，性能稳定，并且提供了相当准确的声源定位。
所有历史的识别方案被抛弃的根本原因都是命中率太低。

遇到的问题（皆针对当前方案）：
* 真机上的安静模式（只有风扇噪音和环境噪音）的命中率非常不稳定，从 90% 到 0% 之间任意波动。   
  初步找到了影响命中率的要素，识别率在一定情况下（距离、噪声受控的情况下）可控了。
* 踏步噪声带来的两个问题：唤醒词的命中率下降严重，声纹识别的命中率下降严重。   
  之前前尝试了去噪和添加噪声训练的方案，对声纹识别的命中率没有提升。   
  目前我们在麦克板子和机身之间添加了海绵，通过这种方式变相提升了麦克采集到的声音的信噪比。   
  在当前的方案下，使用配置合理的注册集（1、3、5 米安静和噪声都有注册语音），命中率可达 0.93。   
  唤醒词的命中率目前还没有靠谱的解决方案。    
  目前的唤醒词功能是讯飞的板子提供的，我们目前探索的主要方向是脱离对讯飞板子的依赖，使用其它更可控的方式实现唤醒功能。

（背景知识）
任何对声音的处理都会影响声纹识别的命中率。
解决的方法：
（背景知识）

-------
MeshLab 中模型简化的通常操作：
Filters 菜单：
Remeshing, Simplification and Reconstruction：
Simplification: Quadric Edge Collapse Decimation
指定目标面数，选定 Preserve Normal 等选项，然后 Apply。

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
