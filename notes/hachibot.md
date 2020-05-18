2020-5-16：
1. 实现单独播放 reference 行动的能力。
2. 配合真机部署。
3. 非吐槽，认真记录一下代码中遇见的不妥之处，作为将来我们建立自己的设计參考（代码审查的检查点）的反面教材：
   - 被使用时只接受一个元素的数组参数。
     - 这一点我能够理解为什么产生：当你开始“设计”一件事情时，会抱着它能够用于更广泛的场景的预期。
     - 当前我建议慎用此类“更广泛场景”的预期或假设，较为妥当的设计原则可以是“够用刚好”。
     - 只需要传一个参数就适用的情况，参数换成数组类型，对读者会造成两种干扰：一是时时要追溯这个数组被使用的方式，二是并不能准确理解使用数组参数的意图。
     - 一直以来我认为代码写出来主要两类读者是人和机器，人是重要读者之一，并且每一个读者都是潜在的维护者，大家都想着读者读到这里的感受，对软件整体的质量是有帮助的。
   - 伪面向对象。
     - 以 motion_data.py 中诸如 get_frame_root_pos/get_frame_root_rot 此类方法为例，这是把本应属于某类的方法，做成了工具方法放在了另外的类中。
     - Python 是挺好用的脚本语言，设计一个类在写代码上的代价很小，在设计上也许会需要动一些脑，但这是长久增益。
     - 一个类的方法列表太长的时候，就已经反映问题了；一个类的代码行数太长的时候，同样也反映问题了。
     - 设计建议：模块的粒度小一些，类的粒度小一些，方法的粒度小一些。除非迫不得已（不要给自己借口迫不得已），不要写行数超过 50 行的方法，不要写方法数超过 10 的类。
     - 尽力让类被设计成它应该有的样子，它应该有的样子会使代码更容易理解、组织。
   - 本条非记录，是对这个记录列表的一点解释。这个列表（以及后续类似列表）希望能先把目前接触的小问题搜集起来：
     - 是一些从长久的可维护性的角度的一些考虑审视，不是标准或规矩。
     - 是对我经验的部分梳理，不一定与当前流行的最佳实践和谐一致。
     - 没有任何洁癖，看到不妥的我会想怎么弄好一点，但并非不考虑成本，也会尊重他人的编码习惯。
   - TBC

调用栈查找的记录：
- [1.1.1.1] `def calc_blend_idx(self, time)`
- [1.1.1] motion_data.py, `def calc_frame(self, time)`
- [1.1.2] ?
- [1.1] imitation_task.py, `def _calc_ref_pose_warmup(self)`
- [1] imitation_task.py, `def _calc_ref_pose(self, time, apply_origin_offset=True)`

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
