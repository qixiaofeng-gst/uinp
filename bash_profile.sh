gst() {
  ssh qixiaofeng@112.126.73.197
}

d() {
  gpg -o "${1/.gpg/.txt}" -d "$1"
}

c() {
  gpg -o "${1/.txt/.gpg}" -c "$1"
}

t1_mp4() { # iphone7 1334x750
  ffmpeg -i $1 -s 667x375 -vcodec mpeg4 -b:v 1024k -acodec aac -scodec copy "$2.mp4"
}

t2_mp4() { # iphone7 1334x750
  ffmpeg -i $1 -s 667x375 -vcodec mpeg4 -b:v 2048k -acodec aac -scodec copy "$2.mp4"
}

uinp_up() { # Only used under windows.
  local curr=`pwd`
  local bpFile="$curr/bash_profile.sh"
  if [ ! -f $bpFile ]; then
    echo "Must execute this under uinp project root."
    return $E_CONFFILE
  fi
  cd ~
  ln -fs $bpFile .bash_profile
  cd $curr
  echo ".bash_profile updated. Restart bash to take effect."
}

wx_rm() { # Used to remove a page from WeiXin project.
  svn rm --force "$1.js" "$1.json" "$1.wxml" "$1.wxss"
}

setup_git() {
  git config --global user.email "qixiaofeng@gsegment.com"
  git config --global user.name "XiaofengQi"
}

enhance_git() {
  git config --global alias.st "status"
  git config --global alias.ci "commit -m"
  git config --global alias.co "checkout"
  git config --global alias.plom "pull origin master"
  git config --global alias.psom "push origin master"
}

init_ssh() {
  env=~/.ssh/agent.env

  agent_load_env () { test -f "$env" && . "$env" >| /dev/null ; }

  agent_start () {
      (umask 077; ssh-agent >| "$env")
      . "$env" >| /dev/null ; }

  agent_load_env

  # agent_run_state: 0=agent running w/ key; 1=agent w/o key; 2= agent not running
  agent_run_state=$(ssh-add -l >| /dev/null 2>&1; echo $?)

  if [ ! "$SSH_AUTH_SOCK" ] || [ $agent_run_state = 2 ]; then
      agent_start
      ssh-add
  elif [ "$SSH_AUTH_SOCK" ] && [ $agent_run_state = 1 ]; then
      ssh-add
  fi

  unset env
}

init_ssh
