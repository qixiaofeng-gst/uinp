Next TODO:
- Hearing:
  - Three parts:
    - Loudness, a measure of sound wave intensity
    - Pitch, the frequency of the fundamental component
    - Timbre, harmonic content of the signal determine it
  - Property:
    - Base on the amplitude of the frequencies
    - Very insensitive to phase
  - Phenomena:
    - Shape of the time domain waveform only indirectly relate to hearing
    - Usually not consider time domain waveform in audio systems
- 一个 8 度(octave)频率增长一倍, human hearing about 10 octaves
- Audio information distribute in a logarithmical way:
  - 50-100Hz carry as much audio information as 10K-20KHz
  - Natural sounding speech requires about 3.2kHz, the frequency range is only 16% of human hearing(3.2KHz out of 20KHz)
  - The signal of natual sounding speech contains 80% of the original sound information(8 out of 10 octaves)
- Voice spectrogram, or voiceprint:
  - Broke the audio signal into short segments, say 2 to 40 milliseconds
  - Chose of the segment length is a tradeoff between frequency resolution(longer) and time resolution(shorter)
- Its impulse response completely describe the characteristics of a linear system.
- Signal and systems:
  - Filter is system
  - Impulse response can describe system
  - We also call impulse response system/filter kernel 
  - Below '*' means convolution
  - Superposition is the foundation of DSP
  - x[n] * \delta[n] = x[n]
  - y[n] = h[n] * x[n]
  - The central limit theorem implies that a pulse-like signal convolve with itself many times will produce a Gaussian
  - Duality:
    - Point vs Sinusoid
      - A single point in the frequency domain corresponds to a sinusoid in the time domain
      - A single point in the time domain corresponds to a sinusoid in the frequency domain
    - Convolution vs multiplication
      - Convolution in the frequency domain corresponds to multiplication in the time domain
      - Convolution in the time domain corresponds to multiplication in the frequency domain
  - Frequency domain representaion:
    - Rectangular form: ReX[] + ImX[] is the best choice for caculation
    - Polar form: MagX[] + PhaseX[] with graphs for human understanding
  - White noise contains an equal amount of all frequencies
  - May use zeros padding to make length of the DFT longer
  - Four early mentioned windows:
    - Rectangular window
    - Hamming window
    - Blackman window
    - Flat-top window
  - Understanding how information represent in signals is always the first step in successfull DSP:
    - For some field, phase contains much of the information about the shape of the time domain waveform,  
      with the magnitude playing a minor role
    - Some signals have encoded their information in magnitude, with the phase playing a minor role,  
      such as audio signals
  - 
- 我们的语音交互场景表现不好的原因：
  - 通常都出了一个波长的距离，低音人声（波长 1.1-4.25 米），女高音（波长 0.23-1.36 米），属于远场
  - 目前的混响处理通常都没有针对远场做处理，混响对识别效果的影响非常大
  - 测向算法对噪声的鲁棒性很差，我们知道踏步是噪声，但是算法不知道
  - 测向算法能取得稳定效果的前提是去噪
- 音频信号前端基础：
  - 噪声抑制 NS
  - 混响抑制 Reverberation Cancellation/Suppression，统归去混响 Dereverberation
    - AIR, acoustic impulse response
    - Cancellation depends on AIR estimation
    - Suppression does not depend on AIR estimation
  - 回声消除 AEC
  - 盲源分离 BSS，blind signal/source separation
  - 声源定位 DOA
  - 波束成型 Beamforming
    - Beamforming depends on DOA
  - 人声检测 VAD
  - 远场语音识别 3 难点：
    - 多通道同步采集硬件
    - 前端麦克风阵列降噪算法
      - 回声消除算法在 speex 和 webrtc 中都有，它们都使用了大量非线性处理手段，会降低语音识别的效果
      - 语音识别引擎对语音信号的非线性处理非常敏感
      - 语音失真少一些即使背景噪声有残留，对语音识别率的影响也会小
      - 房间混响会造成麦克风接收到的信号有很长的拖尾，人耳能够自动解混响，所以人在房间中交流并没有影响反而觉得声音饱满
      - 混响对语音识别有致命影响，房间冲击响应通常 400-1000ms，语音识别一帧长度只有 50ms
      - 在有远场需求之前，对去混响的研究少，一般用倒谱平均、谱减法，这类方法对于远场识别率提升不大
      - 目前比较好的去混响算法叫多步线性预测法
      - 非平稳噪声干扰主要利用波束形成去除，波束形成依赖测向，学术上一般研究测向精度和分辨率，这些对实践的意义不大
      - 实践对测向的要求是鲁棒性，好的波束形成可以大幅提高识别率
      - 测向对噪声的鲁棒性很差 -> 测向也依赖去噪
    - 后端语音识别与前端信号处理算法的匹配
      - 训练数据和测试数据越匹配效果越好
      - 训练数据的采集环境与远场环境一致，前端处理算法也一致，方能保证识别的性能
  - 麦克风距离远了之后，语音信号到达麦克风时衰减严重，让干扰信号（环境噪音、混响、音乐、其他人声）的影响不能被简单忽略
- 近场和远场的定义：
  - 近场，小于 1 个波长范围内
  - 通常辐射场传播 10 个波长后，就脱离了近场模式
- 声波：
  - 波速：340米每秒
  - 人耳能听到的波长：0.017-17米
  - 人耳听到频率：20Hz-20KHz。更低频的称次声波，20K-1GHz 的称超声波，大于 1GHz 的称微波超声
  - 低音人声 80-300Hz
  - 中低音人声 120-400Hz
  - 中音人声 150-450Hz
  - 女低音 180-800Hz
  - 女高音 250-1500Hz
  - 女声波长短，更容易进入远场模式
  - 高频部分在传播过程中能量消散更严重
- https://stackoverflow.com/questions/9981087/simple-c-sound-api
- https://stackoverflow.com/questions/31674416/python-realtime-audio-streaming-with-pyaudio-or-something-else
- https://www.alsa-project.org/wiki/Documentation
- http://www.mpg123.de/
- https://github.com/respeaker/mic_array
- https://github.com/toby0514/pdm2pcm
- https://en.wikipedia.org/wiki/Pulse-density_modulation
- https://github.com/wady101/PYNQ_Z2-Audio
- http://www.pynq.io/
- Implement the better emotion service, which can configure the limit of different behavior.
- synchronized & volatile(atomic access), interleave preventing and happens-before relation.
- In multithreading world you should not ask: Why this should not work, but rather Why this is guarranteed to work.
- IntelliJ IDEA 相关：
  - POM 错误 “找不到插件”： https://stackoverflow.com/questions/20496239/maven-plugins-can-not-be-found-in-intellij
- UDP stuff:
  - https://stackoverflow.com/questions/15734219/simple-python-udp-server-trouble-receiving-packets-from-clients-other-than-loca/36185759
  - https://stackoverflow.com/questions/10692956/what-does-it-mean-to-bind-a-multicast-udp-socket
- a1: control system. c1,c2,c3,c4,c5,c6: supporting systems.
- Docker logging:
  - https://jaxenter.com/docker-logging-gotchas-137049.html
  - https://superuser.com/questions/1482221/docker-logs-is-missing-log-entires-which-are-showing-in-docker-attach
- Docker auto-start: https://stackoverflow.com/questions/30449313/how-do-i-make-a-docker-container-start-automatically-on-system-boot
- 橙II：80.248
- 橙狗：200.50. 切换到 HachiDev 之后 80.58：HachiBot.dev2020, HachiBot.robot
- Start OpenGL playground with emacs.
- 看看 python multiprocessing Pool 中有进程出错会导致什么。
- A tiny program for passward handling (mask it).

Begin Packet
------------
Message ID: 7
Packet Size: 394
Begin MoCap Frame
-----------------
Frame #: 3838
Marker Set Count: 1
Model Name: all
Marker Count: 0
Unlabeled Markers Count: 4
        Marker 0 : -0.3645332455635071 , 0.7287006974220276 , 1.4096826314926147
        Marker 1 : -0.12072987854480743 , 0.7251772284507751 , 1.4182041883468628
        Marker 2 : -0.439276784658432 , 0.672813355922699 , 1.3586702346801758
        Marker 3 : -0.03738488256931305 , 0.6613466739654541 , 1.3952510356903076
Rigid Body Count: 0
Skeleton Count: 0
Labeled Marker Count: 10
marker_id:67600, pos:(-0.4167289137840271, 0.10561717301607132, 1.4084266424179077), size:(0.012000000104308128,)
occluded:False, point_cloud_solved:True, model_solved:False
Residual: 0.0
marker_id:67612, pos:(-0.10640587657690048, 0.18492943048477173, 0.5021811723709106), size:(0.012000000104308128,)
occluded:False, point_cloud_solved:True, model_solved:False
Residual: 0.0
...
marker_id:1591611883, pos:(-0.4227224290370941, 0.24622248113155365, 0.40662679076194763), size:(0.012000000104308128,)
occluded:False, point_cloud_solved:True, model_solved:False
Residual: 0.0
Force Plate Count: 0
Device Count: 0
stamp_camera_exposure:0, stamp_data_received:126919183750, stamp_transmit:126919186959
End Packet
----------

WARNING:tensorflow:5 out of the last 5 calls to <function AVSR.train.<locals>.distributed_train_step at 0x7fab2f298820>
 triggered tf.function retracing. Tracing is expensive and the excessive number of tracings could be due to
  (1) creating @tf.function repeatedly in a loop,
  (2) passing tensors with different shapes,
  (3) passing Python objects instead of tensors.
For (1), please define your @tf.function outside of the loop.
For (2), @tf.function has experimental_relax_shapes=True option that relaxes argument shapes that can avoid unnecessary retracing.
For (3), please refer to https://www.tensorflow.org/tutorials/customization/performance#python_or_tensor_args
 and https://www.tensorflow.org/api_docs/python/tf/function for  more details. 
(
    (<tf.Tensor 'inputs:0' shape=(2, 231, 36, 36, 3) dtype=float32>,
     <tf.Tensor 'inputs_1:0' shape=(2, 231, 2) dtype=float32>,
     <tf.Tensor 'inputs_2:0' shape=(2,) dtype=int64>,
     <tf.Tensor 'inputs_3:0' shape=(2,) dtype=string>),
    (<tf.Tensor 'inputs_4:0' shape=(2, 258, 240) dtype=float32>,
     <tf.Tensor 'inputs_5:0' shape=(2,) dtype=int64>,
     <tf.Tensor 'inputs_6:0' shape=(2,) dtype=string>),
    (<tf.Tensor 'inputs_7:0' shape=(2, 112) dtype=int64>,
     <tf.Tensor 'inputs_8:0' shape=(2,) dtype=int64>,
     <tf.Tensor 'inputs_9:0' shape=(2,) dtype=string>,
     <tf.Tensor 'inputs_10:0' shape=(2,) dtype=string>)
)
(
    (<tf.Tensor 'inputs:0' shape=(2, 143, 36, 36, 3) dtype=float32>,
     <tf.Tensor 'inputs_1:0' shape=(2, 143, 2) dtype=float32>,
     <tf.Tensor 'inputs_2:0' shape=(2,) dtype=int64>,
     <tf.Tensor 'inputs_3:0' shape=(2,) dtype=string>),
    (<tf.Tensor 'inputs_4:0' shape=(2, 177, 240) dtype=float32>,
     <tf.Tensor 'inputs_5:0' shape=(2,) dtype=int64>,
     <tf.Tensor 'inputs_6:0' shape=(2,) dtype=string>),
    (<tf.Tensor 'inputs_7:0' shape=(2, 116) dtype=int64>,
     <tf.Tensor 'inputs_8:0' shape=(2,) dtype=int64>,
     <tf.Tensor 'inputs_9:0' shape=(2,) dtype=string>,
     <tf.Tensor 'inputs_10:0' shape=(2,) dtype=string>)
)

(
    (
        tf.TensorSpec(shape=(None, None, 36, 36, 3), dtype=tf.float32, name=None),
        tf.TensorSpec(shape=(None, None, 2), dtype=tf.float32, name=None),
        tf.TensorSpec(shape=(None,), dtype=tf.int64, name=None),
        tf.TensorSpec(shape=(None,), dtype=tf.string, name=None)
    ),
    (
        tf.TensorSpec(shape=(None, None, 240), dtype=tf.float32, name=None),
        tf.TensorSpec(shape=(None,), dtype=tf.int64, name=None),
        tf.TensorSpec(shape=(None,), dtype=tf.string, name=None)
    ),
    (
        tf.TensorSpec(shape=(None, None), dtype=tf.int64, name=None),
        tf.TensorSpec(shape=(None,), dtype=tf.int64, name=None),
        tf.TensorSpec(shape=(None,), dtype=tf.string, name=None),
        tf.TensorSpec(shape=(None,), dtype=tf.string, name=None)
    )
)

(
    (
        tf.TensorSpec(shape=(2, None, 36, 36, 3), dtype=tf.float32, name=None),
        tf.TensorSpec(shape=(2, None, 2), dtype=tf.float32, name=None),
        tf.TensorSpec(shape=(2,), dtype=tf.int64, name=None),
        tf.TensorSpec(shape=(2,), dtype=tf.string, name=None)
    ),
    (
        tf.TensorSpec(shape=(2, None, 240), dtype=tf.float32, name=None),
        tf.TensorSpec(shape=(2,), dtype=tf.int64, name=None),
        tf.TensorSpec(shape=(2,), dtype=tf.string, name=None)
    ),
    (
        tf.TensorSpec(shape=(2, None), dtype=tf.int64, name=None),
        tf.TensorSpec(shape=(2,), dtype=tf.int64, name=None),
        tf.TensorSpec(shape=(2,), dtype=tf.string, name=None),
        tf.TensorSpec(shape=(2,), dtype=tf.string, name=None)
    )
)

ANN, CNN, RNN, GAN.
滤波算法的本质是基于某种模型预估未来信号，并用于修正真实信号。

基于 sipeed 板子弄的话，转接的芯片不太好找：
- 串口，串口没有时钟，靠约定波特率通信，不适用。
- SPI，MOSI/MISO/CLK，双向通信的，不适用。
- I2C，不适用。

我们需要的是 I2S，目前能淘宝搜索到的转接芯片只支持 2 声道转一个 USB，不经济。

BSP: Architecture(I guess Board) Support Package.
FPIOA: Field Programmable I/O Array. Map 256 functions to 48 free I/Os on the chip(K210).
GPIOHS: GPIO stands for General Purpose Input Output, HS stands for High Speed.
I2C bus: Inter-Integrated Circuit bus. Used to communicate with multiple external I2C devices.
I2S bus: I2C Sound bus. A serial bus interface standard used for connecting digital audio devices together. Three types of signals:
    1. BCK, the clock signal.
    1. WS, the channael selection signal.
    1. SD, the serial data signal.
PLL: Phase Lock Loop. A control system that generates an output signal whose phase related to the phase of an input signal.
UARTHS: UART stands for Universal Asynchronous Receiver/Transmitter, HS stands for High Speed.

C array into c++ vecter:
https://stackoverflow.com/questions/259297/how-do-you-copy-the-contents-of-an-array-to-a-stdvector-in-c-without-looping

ping 192.168.100.5 -s $((1024*32)) -D

厕所附近 - FE:EC:DA:B5:24:7B
办公区 - B6:FB:E4:2B:E6:D9
茶水间 - 86:83:C2:C8:DC:56

调试 tf 的指令：rosrun tf tf_echo base_link odom

淼神安利的三板斧：分解、量化、搜索。

conda_env_for_har.yaml
    cython-bbox==0.1.3
    detectron2==0.2.1+cu101
    graphsurgeon==0.4.1
    mmcv-full==1.1.1
    pycuda==2019.1.2
    python-graphviz==0.8.4
    torch==1.6.0+cu101
    pyaudio==0.2.11
conda_env_for_seg.yaml
    mmcv==1.1.1

======= trait
/hachi/catkin_ws/src/hachi_msgs
/hachi/projects/human-segmentation
python: har

Detectron2 for har: install it with github repository.
Models cache for har:
~/.torch/fvcore_cache/detectron2/COCO-Detection
~/.insightface/models
scripts/fastreid/logs # download from 100.5, or fastreid throw runtime error

_______
使用 cmake 设置 shared library 绝对路径的方法：   
https://stackoverflow.com/questions/33165270/force-cmake-to-use-the-full-library-path   
强制不绝对路径的办法：   
https://stackoverflow.com/questions/24958967/cmake-link-to-shared-libraries-without-using-full-path

使用 nvidia-smi 切换 GPU 的计算模式：`sudo nvidia-smi -c <n>`。n=0 时是默认模式，n=3 时是 exclusive 模式。

_______
可能的提议
- 唤醒到声纹做成一体，如果都依赖 kaldi 库的话，正好。

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
以后的量化周报备选：
- RL 算法针对不同目标的训练效率评估。
- 仿真环境的“真实性”衡量，真实性依赖于对真实环境的建模，对于不同方面有不同模型，比如地面摩擦、空气摩擦、运动学、动力学等等。
- 评估不同 RL 算法的上限，通过模型能够达到的控制效果来评估。
- 将仿真性能、仿真真实性、RL 算法效率、RL 算法效果这四个方面进行综合考虑，对训练项目做总体的量化评估。
- 团队开发的工程化程度评估。从需求管理、任务追踪、代码开发、代码库管理、发布库管理、测试系统这些方面抽取更多细节，评估权重，设计数值计算方式。

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
