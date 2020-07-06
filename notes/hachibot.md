2020-7-4：
- [Done] 将扭矩控制模式下的 RaisimEnv 中的训练结果迁移到 dacong 仿真器。
  - 使用与训练环境一致的 mean/std。
  - 在 dacong 仿真器中的效果不行，见[wiki](http://wiki.corp.hachibot.com/pages/viewpage.action?pageId=21790761)。
- [Done] 在 dacong 和 raisim 两边实现了简单的蹲起运动作为对照。
  - 蹲起运动的姿态是一致的。
  - 导致 PPO2 在两边表现不一致的原因可能是 observation 不一致。
- [Done] 讨论并记录[对 raisim 的 action/obs 的 mean/std 的理解](http://wiki.corp.hachibot.com/pages/viewpage.action?pageId=6914287)。
- [Planned] 比对 dacong 和 raisim 两边的 observation。
- [Planned] 去除对 raisim 自定义的 PPO2 的依赖，转而依赖 stable_baselines 中原始的 PPO2。
- [Planned] 绘制完整的基于模仿学习的动作控制的流程图，并发起讨论。
  - 起点是 AI4Animation 的方案。
  - 起点是路径规划的方案。

今天下午，淼神、唐彬、梦婷、陈晨和我一起交流了一下对 RaisimEnv 中 action/obs 的 mean/std 的理解。
先解释一下下面会用到的符号：
- o：observation，是 policy net 的输入，env 的输出。
- r：reward，是 policy net *训练时的* 输入，env 的输出。
- a：action，是 policy net 的输出，env 的输入。
- o_m：observation mean，在下面提及时都指常量。
- o_s：observation std，常量。
- a_m：action mean，常量。
- a_s：action std，常量。

在 RaisimEnv 中有两个行为：
- 输出 o 之前，会执行（公式一） o = (o - o_m) / o_s。
- 输入 a 之后，会执行（公式二） a = a * a_s + a_m。

两个角度来理解 RaisimEnv 的这两个行为：
- 从限制搜索空间的角度。（梦婷和陈晨提供）
  - 公式一使用 o_m 和 o_s 将 o 限制在了一个类似正态分布的分布上。   
  梦婷和陈晨皆言及这个做法他们以前没有见过，感觉并不是必要的过程。
  - 同理公式二将 a 限制在了一个分布上。
  - 这样可以将学习的搜索空间限制在一个相对有限的范围，大大提升学习效率。
- 从添加偏移的角度。（唐彬提供）
  - policy net 的输出是归一化的，所以并不能直接用于控制，需要使用公式二将 a 映射到真实的关节角。
  - 公式二则将原始的（直接观测到的数据） o 归一化了。

我的菜鸟视角：
- env 是使用先验知识建立的模型，主要包含：
  - o = n(p(m(a))) = f(a)，实质上就是映射 + 仿真引擎（物理引擎）。
  - r = g(a, o) = g(a, f(a)) = h(a)，实质上是靠设计者的经验 + 直觉设定的计算逻辑。
  - 上面提及的 p、m、n、f、g、h 都是广义的函数。p 对应物理引擎，m 对应公式二，n 对应公式一。
- policy net 要学习（拟合）的是部分 h，即 p 和 g（物理引擎和 reward 函数）。
- 所以对 h（f 和 g）的设计本质上是在设计 policy net 的学习目标，让 policy net 模仿 f 和 g 的行为。
- 传统的 h 的实现需要专家知识，开发成本相对较高（并不绝对）。
- policy net 的结构基本是领域内通用的，而设计一个比较符合直觉的 h 相对来说简单一些，
所以这种学习方法看起来成本低一些，看起来也比较容易扩展和自动化。
- o_m、o_s、a_m 和 a_s 都是 f 的常量部分（m 和 n），最终 policy net 在不同的 env 之间迁移的时候，m 和 n 要随同一起迁移。

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
