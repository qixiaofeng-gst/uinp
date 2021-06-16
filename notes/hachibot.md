Next TODO:
```
vec3 palette(float d){
	return mix(vec3(0.2,0.7,0.9), vec3(1.,0.,1.), d);
}

vec2 rotate(vec2 p, float a){
	float c = cos(a);
    float s = sin(a);
    return p * mat2(c, s, -s, c);
}

float map(vec3 p){
    for( int i = 0; i<8; ++i){
        float t = iTime * 0.2;
        p.xz = rotate(p.xz, t);
        p.xy = rotate(p.xy, t * 1.89);
        p.xz = abs(p.xz);
        p.xz -= .5;
	}
	return dot(sign(p), p) / 5.;
}

vec4 rm (vec3 ro, vec3 rd){
    float t = 0.;
    vec3 col = vec3(0.);
    float d;
    for(float i =0.; i<64.; i++){
		vec3 p = ro + rd*t;
        d = map(p)*.5;
        if(d<0.02){
            break;
        }
        if(d > 100.){
        	break;
        }
        col += palette(length(p) * .1)/(400. * (d));
        t+=d;
    }
    return vec4(col,1./(d*100.));
}

void mainImage( out vec4 fragColor, in vec2 fragCoord )
{
    vec2 uv = (fragCoord - (iResolution.xy / 2.)) / iResolution.x;
	vec3 ro = vec3(0., 0., -50.);
    ro.xz = rotate(ro.xz, iTime);
    vec3 cf = normalize(-ro);
    vec3 cs = normalize(cross(cf, vec3(0., 1., 0.)));
    vec3 cu = normalize(cross(cf, cs));

    vec3 uuv = ro + cf * 3. + uv.x * cs + uv.y * cu;
    vec3 rd = normalize(uuv - ro);
    vec4 col = rm(ro, rd);
    fragColor = col;
}
```
- 可视化工具改进：
  - 依据根目录配置自动扫描并列出可选工作目录和目录的状态（稀疏、已压缩和正在压缩）；
  - 删除已压缩的原始图像；
- ai-pod 找不到有线网卡时，找 ~/r8xxx 目录之下的 autorun.sh 脚本。
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
  - Multirate techniques:
    - If we want a finer resolution, e.g. interpolate a 50 sample signal into a 400 sample signal:
      1. Take the 50 samples and add zeros to make the signal 64 samples long.
      1. Use a 64 point DFT to find frequency spectrum, which will consist of a 33 point real part, and a 33 point imaginary part.
      1. Pad the right side of the frequency spectrum with 224 zeros to take the frequency spectrum 257 points long.
      1. Use a 512 point inverse DFT to transform the data back in to the time domain.   
      The first 400 samples of the result are an interpolated version of the original 50 samples.
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
- 橙II：80.248，橙III：200.67。
- 橙狗：200.50. 切换到 HachiDev 之后 80.58：HachiBot.dev2020, HachiBot.robot; 192.168.200.1:admin:normal
- Start OpenGL playground with emacs.
- 看看 python multiprocessing Pool 中有进程出错会导致什么。
- A tiny program for passward handling (mask it).
- `rostopic pub -1 /hachi/voice/command hachi_msgs/VoiceCommand '{voice_text: opendoor, command_id: 3, track_id: test-command, angle: 0}'`

ANN, CNN, RNN, GAN.
滤波算法的本质是基于某种模型预估未来信号，并用于修正真实信号。

C array into c++ vecter:
https://stackoverflow.com/questions/259297/how-do-you-copy-the-contents-of-an-array-to-a-stdvector-in-c-without-looping

ping 192.168.100.5 -s $((1024*32)) -D

厕所附近 - FE:EC:DA:B5:24:7B
办公区 - B6:FB:E4:2B:E6:D9
茶水间 - 86:83:C2:C8:DC:56

调试 tf 的指令：rosrun tf tf_echo base_link odom

淼神安利的三板斧：分解、量化、搜索。

_______
使用 cmake 设置 shared library 绝对路径的方法：   
https://stackoverflow.com/questions/33165270/force-cmake-to-use-the-full-library-path   
强制不绝对路径的办法：   
https://stackoverflow.com/questions/24958967/cmake-link-to-shared-libraries-without-using-full-path

使用 nvidia-smi 切换 GPU 的计算模式：`sudo nvidia-smi -c <n>`。n=0 时是默认模式，n=3 时是 exclusive 模式。

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

_______
webrtcvad: audio segmentation
给正常语音添加噪音 https://zhuanlan.zhihu.com/p/98960359

_______
播放科大讯飞6麦克风 SDK 录制的原始音频：ffplay -autoexit -f s32le -ar 16000 -ac 8 mic_demo_vvui_ori.pcm
去噪过的：ffplay -autoexit -f s16le -ar 16000 mic_demo_vvui_deno.pcm

_______
new nuc: hachi@192.168.100.201, normal
Grafana: admin, normal

_______
quick-memo:
- to-har -> har-rsync

PD(proportional-derivative) controller: TeX[[ f = k_p (x_{ref}-x) + k_d (v_{ref}-v), k_p = \omega_n^2, k_d = 2 \xi \omega_n ]]

Useful information:
1. pybind11 is the bridge between c++ and python
2. check Qt installation: qtdiag
3. URDF: Unified Robot Description Format; SDF: Simulation Description Format; MJCF: Multi-Joint dynamics with Contact Format
4. Eigen: C++ template library for matrix/verctor/numerical solver, pure header files without binary library
5. Executables for Qt might start correctly with -qt=qt5 as argument

About audio processing:
-------
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

About motion control:
-------
PID: Proportional Integral Derivative
MPC: Model Predictive Control
RPC: Rate Predictive Control. It is model free. Adaptive to change, less oscillation.

About embeded OS infrastructure:
-------
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
