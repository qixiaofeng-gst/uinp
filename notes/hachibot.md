2020-6-16：
- [In-progress] 学习 deep-speaker 和 ODAS 论文（读完会写 Wiki）。
  - https://github.com/TAMU-VITA/AutoSpeech
  - https://github.com/CorentinJ/Real-Time-Voice-Cloning
  - https://github.com/WeidiXie/VGG-Speaker-Recognition
  - https://github.com/iiscleap/NeuralPlda
  - https://github.com/joaoantoniocn/AM-MobileNet1D
- [Note] 基础知识链接：
  - https://developer.mozilla.org/en-US/docs/Web/Media/Formats/Audio_concepts
    - sample size, bits used for storing a single sample.
    - sample rate, sample quantity of one second.
    - frame, data record contains samples from all channels.
    - frame size, sample size multiplies channels count.
  - https://developers.google.com/machine-learning/crash-course/embeddings/video-lecture
    - embedding, a relatively low-dimensional space accept translated high-dimensional vectors.
- [Done] 预备 motion-runner 和真机联调。

Deep-Speaker system

Three stages framework of traditional approach
1. Collect sufficient statistics   
   Computed from MFCC optimized GMM-UBM
2. Extract speaker embeddings (i-vector)
3. Classify (PLDA)

DNN-based approach has a classification layer combining stage 1 and 2.   
Two major issues of DNN-based approach:
1. Stage 1 and 2 are not directly optimized with respect to speaker recognition.
2. There is a mismatch between training and test.   
   Give training labels at the frame-level.
   Made Utterance-level predictions in testing.

End-to-end neural speaker verification system combined all three stages.   
Deep-Speaker extends the end-to-end speaker embedding system.
1. A DNN is used to extract frame-level features from utterances.
2. Pooling and length normalization layers generate utterance-level speaker embeddings.
3. Training the model with triplet loss.
   Triplet loss minimizes the distance between embedding pairs from the same speaker   
   and maximizes the distance between pairs from different speakers.
4. Pre-training uses a softmax layer and cross-entropy over a fixed list of speakers.   
   Pre-training improves model performance. 

Context-free (General) Acronyms

| Acronym | Full Words | Description |
| ------- | -------    | -------     |
| DNN | Deep Neural Network | - |
| CNN | Convolutional Neural Network | - |
| RNN | Recurrent Neural Network | - |
| PCM | Pulse-Code Modulation | is a method used to digitally represent sampled analog signals. |
| EER | Equal Error Rate | the value of FAR and FRR meet each other, lower is better. |
| FAR | False Acceptance Rate | rate of the wrongly accepted situations. |
| FRR | False Rejection Rate | rate of the wrongly rejected situations. |
| ACC | ACCuracy | a metraic only for classification, gives the percentage of correctly classified instances. |
| PLDA | Probabilistic Linear Discriminant Analysis | https://ieeexplore.ieee.org/document/6639150 |
| GMM | Gaussian Mixture Model | |
| UBM | Universal Background Model | |
| MFCC | Mel-Frequency Cepstral Coefficients | |
| LSTM | Long-Short Term Memory | |
| NIN | Network-In-Network | |
| GRU | Gated Recurrent Unit | |

Context-dependent Acronyms [[Of Paper](https://arxiv.org/pdf/1705.02304.pdf)]

| Acronym | Full Words | Description |
| ------- | -------    | -------     |   
| ? AP | Audio Positive | It is a positive audio. |

| Jargon  | Explanation |
| ------- | -------     |
| d-vector | DNN-based i-vector. |
| i-vector | https://www.crim.ca/perso/patrick.kenny/Najim_TASLP2009.pdf |
| ResCNN | Residual CNN | - |
| ResNets | Residual Networks | - |

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
