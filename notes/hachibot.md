2020-7-11：
- [In-progress] 和梦婷/张钰/陈晨一起把模仿学习对接到 dacong，最后对接到真机做验证。
  - 在 motion-runner 中添加了取得狗子运动模式（AI 控制、传统恢复站立的控制）的功能。[MR](http://gitlab.corp.hachibot.com/ai/motion-runner/-/merge_requests/12)
  - 梦婷现在可以直接在 dacong 中进行自动 finetune 训练了。
- [Done] raisim 的摩擦系数可以从 0.8 修改成为 0.5 然后再训练试一试。
  - raisimLib 提供的 raisim::World 类有个 API 是 setDefaultMaterial，定义了 friction，restitution 和 restitution threshold。
  - 在 envrionment 初始化时，使用 setDefaultMaterial 方法将 friction 设置成了 0.5，另外两个值使用了 raisimLib 中找到的默认值。
  - 重新训练后，在 dacong 里面跑起来了，不再原地踏步。
  - 目前在 raisim 中和 dacong 中的区别就是，dacong 中转圈跑的圈子小一些，raisim 里面转的圈大一些。
  - 唐彬的建议是，最后这点差异，可以通过在训练时添加噪声，让狗子适应不同的速度来解决。

_______
- 工程化：
  - 目前 reward_function 我做成了可以用配置直接切换的程度。
  - 下一步希望把更多的环节，比如 environment 初始化，step，observation 更新，
  都做成可以写好 c++ 代码之后，在 python 和 yaml 中可以动态配置拼接的状态。
  - 希望达到的理想状态是，c++ 代码可以稳定在一个版本后不用再变更，所有训练都可以通过使用 yaml 和 python 来配置完成。
  - 大家对此有没有什么更好的想法？或者补充更多需要抽取成可热插拔的环节或模块？
  - 淼神建议我可以做一些工具辅助博士开发。

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
