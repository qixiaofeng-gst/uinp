2020-6-16：
- [Note] 学习下 deep-speaker 和 ODAS 论文。记一些或许有用的链接：
  - https://github.com/Akella17/speaker-embedding
  - https://github.com/taylorlu/Speaker-Diarization
  - https://github.com/WeidiXie/VGG-Speaker-Recognition
  - https://github.com/ina-foss/inaSpeechSegmenter
  - https://www.openslr.org/38/
  - https://github.com/JRMeyer/open-speech-corpora
- [Done] 载入 wav 方式更新后，本地测试实时识别的结果与更新前并无太大分别。
  - 时常有 1.6 秒的声音数据只有 0.3～0.6 秒有实际声音的与某人相似度高达 0.6~0.7。
  - 时常有两人同时发声的被以 0.6~0.7 相似度识别为其中之一。
  - 笑声经常识别得错得离谱，比如我的笑声经常被识别为其他人。
- [Done] 检查从 wav 载入进来的值。
  - 目前 deep-speaker 中载入音频的手段有两种，一种是 librosa.load，一种是 scipy.io.wavfile.read。
  - librosa.load 载入进来的数据默认是 float32 的，scipy.io.wavfile.read 载入的数据默认是文件本身格式相关的。
  - 从前一直没有检查这两个东西载入进来的数据有什么差异，直到今天……
  - 直接打印数值，同一个文件 librosa: 0.29040527 0.24014282 0.28622437 ...; scipy: 9516 7869 9379 ...
  - 从 ODAS socket 接收过来的数据，是 int16 的，存储到本地的 wav 数据也是 int16 的。
  - 用 librosa 载入的本地 wav 数据采用了未知的一种方式将数据转换成了 float32，导致数值上与 scipy 方式不一样。
  - 为了排除 librosa 数据转换后的数据对识别网络的性能造成影响，后面准备载入数据的环节都用 scipy。
  - 由于 librosa 方式有一些数据类型、数值的检查工作，在使用 scipy 重构时会在必要的环节实现这些检查。
- [Done] 本地测试实时识别的效果。
  - Kinect 放置在桌面，距离我的头 50 ～ 60 厘米，我正常音调重复说“大葱过来”“大葱你好”。
  - 当相似度阀值设置在 0.6 的时候，识别结果包含大量的噪声、无声结果。
  - 阈值 0.7 时，识别结果多数包含语音，但存在两个问题：
    1. 出结果的频率较低，平均两秒出一次相似度大于 0.7 的结果。
    1. 命中率较低（0.19），全部是我的语音，多数被误判成了炫煜。还有部分噪声也被误判成了炫煜。
- [Done] 上狗子测试昨天采集的对照数据。
  - NUC 风扇开启时，噪声直接淹没人声，spleeter 无法分离出人声。
  - 关闭 NUC 风扇，狗子步行时，思钡的话音全部识别成我的，或者说，不管有没有人发声，识别结果全是我，且相似度在 0.5~0.6 附近波动。
    - 把录制下来的思钡的话音（带步行噪声的）放到测试脚本中测试命中率，结果为命中率为 0。
    - 用 spleeter 把思钡的话音与步行噪声分离放到测试脚本中测试，命中率为 0.09，命中为思钡时，相似度在 0.3~0.4 之间。

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

Context-free (General) Acronyms   
PCM, Pulse-Code Modulation, is a method used to digitally represent sampled analog signals.

Context-dependent Acronyms   
? AP, Audio Positive, is a positive audio.

Jargons   
d-vector, DNN-based i-vector.

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
