export HACHY_BUILD=/var/local/local-build/for-raisim
export NEW_RAISIM_BUILD=/var/local/local-build/for-new-raisim
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${NEW_RAISIM_BUILD}/lib

function lan-ip-prefix() {
    echo '192.168'
}
function lan-iot-prefix() {
    echo "$(lan-ip-prefix).2"
}
function lan-server-prefix() {
    echo "$(lan-ip-prefix).100"
}
function lan-ai5() {
    echo "$(lan-server-prefix).5"
}
function lan-ai6() {
    echo "$(lan-server-prefix).6"
}
function lan-ai7() {
    echo "$(lan-server-prefix).7"
}
function lan-agx() {
    echo "$(lan-server-prefix).20"
}
function hostname-bx() {
    echo 'ai-bixiao.hachibot.com'
}

raisim-cmake() {
    cmake .. -DCMAKE_INSTALL_PREFIX=$NEW_RAISIM_BUILD -DCMAKE_PREFIX_PATH=$NEW_RAISIM_BUILD -DPYBIND11_TEST=OFF
}
hachy-cmake() {
    cmake .. -DCMAKE_PREFIX_PATH=$HACHY_BUILD -DCMAKE_INSTALL_PREFIX=$HACHY_BUILD -DPYBIND11_TEST=OFF
}
hachy-make() {
    pushd build && make -j4
    popd
}
kaldi-make() {
    pushd ~/Documents/hachy-repos/kaldi-speaker-recognizer/src && make -j10
    popd
}
hachy-calibrate() {
    python -m launcher.calibrator
}
hachy-test() {
    python -m launcher.test
}
hachy-train() {
    python -m launcher.train
}
alias hc-cmake=hachy-cmake
alias hc-make=hachy-make
alias hc-calibrate=hachy-calibrate
alias hc-test=hachy-test
alias hc-train=hachy-train

export DACONG_BUILD=/var/local/local-build/for-dacong
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${DACONG_BUILD}/lib
dacong-cmake() {
    cmake .. -DCMAKE_PREFIX_PATH=$DACONG_BUILD -DCMAKE_INSTALL_PREFIX=$DACONG_BUILD -DPYBIND11_TEST=OFF
}
alias dc-cmake=dacong-cmake

local_ip_addr=$(hostname -I)
local_ip_addr=$(echo $local_ip_addr | sed 's/ *$//g')
export ROS_IP=$local_ip_addr
export ROS_HOSTNAME=$local_ip_addr
export ROS_MASTER_URI=http://$local_ip_addr:11311
# source /opt/ros/melodic/setup.bash

# ======= ======= ======= ======= ======= ======= =======
to-hc() {
    cd ~/Documents/hachy-repos/hachy
}
to-ros() {
    cd ~/Documents/local-sandbox/ros-learning-ws
}
to-mr() {
    cd ~/Documents/hachy-repos/motion-runner
}

# Below lines 'sw' is the abbreviation of 'switch'.
sw-ros-car() {
    export ROS_MASTER_URI=http://192.168.2.131:11311
}
sw-ros-ai5() {
    export ROS_MASTER_URI=http://$(lan-ai5):11311
}
sw-ros-pure-local() {
    # 下面一行针对 roscore 运行的机器。
    export ROS_MASTER_URI=http://127.0.0.1:11311
    # 下面两行针对本地的 ROS 节点，设置成当前机器的数字 IP。
    export ROS_IP=127.0.0.1
    export ROS_HOSTNAME=127.0.0.1
}
sw-ros-agx() {
    export ROS_MASTER_URI=http://$(lan-agx):11311
}
sw-ros-nuc() {
    export ROS_MASTER_URI=http://$(lan-server-prefix).25:11311
}
sw-ros-radar-nuc() {
    export ROS_MASTER_URI=http://$(lan-server-prefix).27:11311
}
sw-ros-to() {
    export ROS_MASTER_URI=http://$(lan-server-prefix).$1:11311
}
alias lock='gnome-screensaver-command -l'
to-uinp() {
    cd ~/Documents/git-repos/uinp
}
to-lt() {
    cd /home/qixiaofeng/Documents/git-repos/uinp/clanguages/learn-tensorflow2
}
start-tf() {
    docker run -u $(id -u):$(id -g) --gpus all -it tensorflow/tensorflow:latest-gpu bash
}
ssh-dog-blue() {
    sshpass -p $(cat ~/pass/hc) ssh -X hachi@$(lan-server-prefix).103
}
ssh-agx() {
    sshpass -p $(cat ~/pass/hc) ssh -X runner@$(lan-agx)
}
ssh-ai5-server() {
    sshpass -p $(cat ~/pass/hc) ssh -X runner@$(lan-ai5)
}
scp-ai5() {
    sshpass -p $(cat ~/pass/hc) scp $3 runner@$(lan-ai5):$1 $2
}
ssh-bx() {
    sshpass -p $(cat ~/pass/Hc) ssh runner@$(hostname-bx)
}
scp-from-bx() {
    sshpass -p $(cat ~/pass/Hc) scp $3 runner@$(hostname-bx):$1 $2
}
scp-to-bx() {
    sshpass -p $(cat ~/pass/Hc) scp $3 $1 runner@$(hostname-bx):$2
}
ssh-ai6-server() {
    ssh -X xiaofeng.qi@$(lan-ai6)
}
ssh-ai7-server() {
    ssh -X xiaofeng.qi@$(lan-ai7)
}
ssh-caihuan-to() {
    sshpass -p $(cat ~/pass/hc) ssh -X caihuan@$(lan-server-prefix).$1
}
ssh-runner-to() {
    sshpass -p $(cat ~/pass/hc) ssh -X runner@$(lan-server-prefix).$1
}
ssh-tv-nuc() {
    sshpass -p $(cat ~/pass/hc) ssh -X ai@$(lan-server-prefix).31
}
rdp-agx() {
    xfreerdp $(lan-agx)
}
vnc-agx() {
    ~/app-image/VNC-Viewer-6.20.113-Linux-x64
}
quick-cmake() {
    cmake .. -DCMAKE_PREFIX_PATH=$LOCAL_BUILD
}
lock-1h() {
    local -i lineNumber=0
    local -i timestamp=$((3000 + $(date +%s)))
    echo $timestamp
    while [ $(date +%s) -lt $timestamp ]; do
        if [ $(($timestamp - $(date +%s))) -lt 15 ]; then
            notify-send \
                'Save work message' \
                '>>>>>>> Save your work. PC will be locked soon. <<<<<<<' \
                -t 2000 -u normal
        fi
        echo Keep going: $(date) --- $((lineNumber++))
        sleep 5s
    done
    echo End $(lock) lock-1h.
}
hcr-ctrl() {
    ~/Documents/hachy-repos/dacong/build/user/MIT_Controller/mit_ctrl m s
}
count-lines() {
    wc -l $(find $1 -type f) | sort -n -
}
count-ext() {
    find $1 -name "*.$2" | wc -w
}

#======= ======= =======
# Belows for human-activity-recognizer project
har-sync() {
    rsync -v -r -f '- .git' -f '- data' -f '- models' ../human-activity-recognizer runner@$(lan-agx):/hachi/ai_motion/catkin_ws/src/
}
har-cd() {
    cd /hachi/catkin_ws/src/human-activity-recognizer
}
alias to-har=har-cd
har-test() {
    roslaunch human_activity_recognizer demo.launch
}
har-start() {
    roslaunch human_activity_recognizer recognizer.launch
}
har-record() {
    roslaunch human_activity_recognizer record.launch
}
rs-start() {
    roslaunch realsense2_camera rs_camera.launch
}

#======= ======= =======
# Belows for motion-imitation mini-cheetah train/test.
mc-train() {
    python3 motion_imitation/train.py
}
mc-test() {
    python3 motion_imitation/test.py
}
mc-kill() {
    kill -9 $(jobs -p)
}
mc-cd() {
    cd ~/Documents/hachy-repos/motion-imitation
}
alias to-mc=mc-cd

#======= ======= =======
# Belows for Ubuntu16 on docker.
dk-install-ubt16() { ## Docker install ubuntu 16.04
    docker pull ubuntu:16.04
}
dk-create-ubt16() { ## Create a docker container.
    docker run -itd --net=host --name ubuntu16 --volume /var/docker-ubt16-home:/home ubuntu:16.04
}
dk-start-ubt16() { ## Start a docker container. Use kill/stop to stop a docker container.
    docker start ubuntu16
}
dk-login-ubt16() { ## Login into a container with /bin/bash.
    docker exec -it ubuntu16 /bin/bash
}

#======= ======= =======
# Belows for ODAS stuff.
odas-build() {
    pushd ../..
    catkin_make
    popd
}
odas-cd() {
    har-cd
    cd ../odas-ros
}
odas-run() {
    roslaunch odas_ros kinect_tracking.launch
}
alias to-odas=odas-cd

#======= ======= =======
# Belows for speaker-recognition stuff.
sr-cd() {
    to-har
    cd ../speaker-recognition
}
sr-run() {
    roslaunch speaker_recognition demo.launch
}
alias to-sr=sr-cd

#======= ======= =======
# Belows for deep-speaker.
ds-cd() {
    to-har
    cd ../deep-speaker
}
ds-run() {
    roslaunch deep_speaker recognize_speaker.launch
}
alias to-ds=ds-cd
xf-cd() {
    to-har
    cd ../xf6mic-ros-node
    export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${PWD}/lib/x64
}
alias to-xf=xf-cd

#======= ======= =======
# Shortcut for sound processing projects.
sp-cd() {
    cd ~/Documents/hachy-repos/1_sound_processing
}
alias to-sp=sp-cd
run-spleeter() {
    local_dir=$PWD
    to-sp
    cd spleeter
    py-spleeter
    python -m spleeter separate -i ${local_dir}/$1 -p spleeter:2stems -o ${local_dir}/samples_for_debug
    deactivate
    cd $local_dir
}

#======= ======= =======
# Shortcut for virtualenv.
py-spleeter() {
    # Created by below command:
    # virtualenv -p python3.6 --copies venv-py3.6-for-spleeter
    source /var/local/venv-py3.6-for-spleeter/bin/activate
}
py-tf2() {
    source ~/Documents/local-sandbox/tensorflow2-venv/bin/activate
}

#======= ======= =======
# Belows for dacong simulation.
to-dc() {
    cd ~/Documents/hachy-repos/dacong/cmake-build-dacongbuild
}
dc-activate() {
    source /var/local/venv-py3.6-for-raisim/bin/activate
}
dc-sim() {
    gnome-terminal -- roscore
    dc-old-sim
}
dc-old-sim() {
    to-dc
    # rm -rf logs/*
    gnome-terminal -- sim/sim
}
dc-ctrl() {
    to-dc
    gnome-terminal -- user/MIT_Controller/mit_ctrl m s
}
record-video() {
    ffmpeg -video_size 1280x720 -framerate 25 -f x11grab -i :1.0+$1,$2 output.mp4
}
convert-audio() {
    ffmpeg -i $1 -f s16le -acodec pcm_s16le audio/awake.pcm
}
dc-make-lcm() {
    /var/local/dependencies/for-dacong/lcm-1.4.0/build/lcmgen/lcm-gen $1 $2
}

# 下面的命令用于与真机调试
check-nic-id() { ## nic: network interface card. 打印主网卡的 ID。
    local first_line=$(ifconfig | grep "^[a-z0-9]*:" | head -n 1)
    local nc_id=$(expr "$first_line" : '\(^[a-z0-9]*\)')
    echo $nc_id
}
setup-dc-connection() { # Check if the dog power on.
    local nic=$(check-nic-id)
    echo 1 >/proc/sys/net/ipv4/ip_forward
    iptables -t nat -A POSTROUTING -o $nic -j MASQUERADE
    iptables -t nat -A POSTROUTING -o wlp0s20f3 -j MASQUERADE
    iptables-save >/etc/iptables.rules
}
setup-dc-ctrl() {
    local nic=$(check-nic-id)
    sudo ifconfig $nic multicast
    sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev $nic
    sw-ros-pure-local
}
ssh-dc() {
    # hint: 1-6
    ssh hachi@192.168.10.10
}
