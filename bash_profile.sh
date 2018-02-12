gst() {
  ssh qixiaofeng@112.126.73.197
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
