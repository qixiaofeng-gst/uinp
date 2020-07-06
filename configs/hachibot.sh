export RAISIM_BUILD=/var/local/local-build/for-raisim
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${RAISIM_BUILD}/lib
raisim-cmake() {
  cmake .. -DCMAKE_PREFIX_PATH=$RAISIM_BUILD -DCMAKE_INSTALL_PATH=$RAISIM_BUILD
}

export DACONG_BUILD=/var/local/local-build/for-dacong
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${DACONG_BUILD}/lib
dacong-cmake() {
  cmake .. -DCMAKE_PREFIX_PATH=$DACONG_BUILD -DCMAKE_INSTALL_PATH=$DACONG_BUILD
}

export ROS_IP=192.168.2.101
export ROS_HOSTNAME=192.168.2.101
source /opt/ros/melodic/setup.bash

# ======= ======= ======= ======= ======= ======= =======
to-hc(){
  cd ~/Documents/hachy-workspace/hachy
}
to-ros(){
  cd ~/Documents/local-sandbox/ros-learning-ws
}
to-mr(){
  cd ~/Documents/hachy-workspace/motion-runner
}

# Below lines 'sw' is the abbreviation of 'switch'.
sw-ros-car() {
  export ROS_MASTER_URI=http://192.168.2.131:11311
}
sw-ros-local() {
  export ROS_MASTER_URI=http://192.168.2.101:11311
}
sw-ros-pure-local() {
  export ROS_MASTER_URI=http://127.0.0.1:11311
  export ROS_IP=127.0.0.1
  export ROS_HOSTNAME=127.0.0.1
}
sw-ros-jetson() {
  export ROS_MASTER_URI=http://192.168.100.20:11311
}
sw-ros-nuc() {
  export ROS_MASTER_URI=http://192.168.100.25:11311
}
sw-ros-local
alias lock='gnome-screensaver-command -l'
to-uinp(){
  cd ~/Documents/git-repos/uinp
}
to-lt(){
  cd /home/qixiaofeng/Documents/git-repos/uinp/clanguages/learn-tensorflow2
}
start-tf(){
  docker run -u $(id -u):$(id -g) --gpus all -it tensorflow/tensorflow:latest-gpu bash
}
ssh-car(){
  ssh -X caihuan@192.168.2.131
}
ssh-agx(){
  ssh -X runner@192.168.100.20
}
ssh-ai-server(){
  ssh -X runner@192.168.100.5
}
ssh-dog-nuc(){
  ssh -X caihuan@192.168.100.25
}
rdp-agx(){
  xfreerdp 192.168.100.20
}
vnc-agx(){
  ~/app-image/VNC-Viewer-6.20.113-Linux-x64
}
quick-cmake(){
  cmake .. -DCMAKE_PREFIX_PATH=$LOCAL_BUILD
}
lock-1h(){
  local -i lineNumber=0
  local -i timestamp=$((3000 + $(date +%s)))
  echo $timestamp
  while [ $(date +%s) -lt $timestamp ] ; do
      if [ $(($timestamp - $(date +%s))) -lt 15 ] ; then
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
hcr-ctrl(){
  ~/Documents/hachy-workspace/dacong/build/user/MIT_Controller/mit_ctrl m s
}
count(){
  wc -l `ls` | sort -n -
}

#======= ======= =======
# Belows for human-activity-recognizer project
har-sync(){
  rsync -v -r -f '- .git' -f '- data' -f '- models' ../human-activity-recognizer runner@192.168.100.20:/hachi/ai_motion/catkin_ws/src/
}
har-cd(){
  cd ~/Documents/local-sandbox/ros-learning-ws/src/human-activity-recognizer
  source ../../devel/setup.bash
  source ../../../catkin_python3_ws/devel/setup.bash --extend
}
alias to-har=har-cd
har-test(){
  roslaunch human_activity_recognizer demo.launch
}
har-start(){
  roslaunch human_activity_recognizer recognizer.launch
}
har-record(){
  roslaunch human_activity_recognizer record.launch
}
rs-start(){
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
  kill -9 `jobs -p`
}
mc-cd() {
  cd ~/Documents/hachy-workspace/motion-imitation
}
alias to-mc=mc-cd

#======= ======= =======
# Belows for Ubuntu16 on docker.
dk-install-ubt16(){ ## Docker install ubuntu 16.04
  docker pull ubuntu:16.04
}
dk-create-ubt16(){
  docker run -itd --net=host --name ubuntu16 --volume /var/docker-ubt16-home:/home ubuntu:16.04
}
dk-start-ubt16(){
  docker start ubuntu16
}
dk-login-ubt16(){
  docker exec -it ubuntu16 /bin/bash
}

#======= ======= =======
# Belows for ODAS stuff.
odas-build(){
  pushd ../..
  catkin_make
  popd
}
odas-cd(){
  har-cd
  cd ../odas-ros
}
odas-run(){
  roslaunch odas_ros kinect_tracking.launch
}
alias to-odas=odas-cd

#======= ======= =======
# Belows for speaker-recognition stuff.
sr-cd(){
  to-har
  cd ../speaker-recognition
}
sr-run(){
  roslaunch speaker_recognition demo.launch
}
alias to-sr=sr-cd

#======= ======= =======
# Belows for deep-speaker.
ds-cd(){
  to-har
  cd ../deep-speaker
}
ds-run(){
  roslaunch deep_speaker demo.launch
}
alias to-ds=ds-cd

#======= ======= =======
# Shortcut for sound processing projects.
sp-cd(){
  cd ~/Documents/hachy-workspace/1_sound_processing
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
py-tf2(){
  source ~/Documents/local-sandbox/tensorflow2-venv/bin/activate
}

#======= ======= =======
# Belows for dacong simulation.
to-dc(){
  cd ~/Documents/hachy-workspace/dacong/build
}
dc-activate() {
  source /var/local/venv-py3.6-for-raisim/bin/activate
}
dc-sim() {
  to-dc
  gnome-terminal -- roscore
  dc-old-sim
}
dc-old-sim() {
  gnome-terminal -- sim/sim
}
dc-ctrl() {
  to-dc
  gnome-terminal -- user/MIT_Controller/mit_ctrl m s
}
