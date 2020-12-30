# Linux 系统使用

## 系统安装并启动之后

### 配置命令行
* 实用的 ll 命令。在 ~/.bashrc 添加：`alias ll='ls -alh'`。
* 神器 tmux，装了不后悔。

### 配置开发工具
* 为 git 设置 ssh 密钥。
  * 查找已有密钥：`ll ~/.ssh`。
  * 创建新密钥：`ssh-keygen -t rsa -b 4096 -C '<user-email>'`。
  * 启动 ssh-agent：`eval "$(ssh-agent -s)"`。
  * 添加已有密钥：`ssh-add ~/.ssh/id_rsa`。
  * 显示公钥：`cat ~/.ssh/id_rsa.pub`。
  * 修改 pass phrase：`ssh-keygen -p`。
* IntelliJ 套餐。
  * https://download.jetbrains.com/webstorm/WebStorm-2020.1.2.tar.gz
  * https://download.jetbrains.com/cpp/CLion-2020.1.2.tar.gz
  * https://download.jetbrains.com/python/pycharm-professional-2020.1.2.tar.gz

## 时间同步
https://www.tecmint.com/synchronize-time-with-ntp-in-linux/
https://docs.fedoraproject.org/en-US/Fedora/18/html/System_Administrators_Guide/sect-Checking_if_chrony_is_synchronized.html
https://chrony.tuxfamily.org/doc/devel/chrony.conf.html
* chrony： `sudo apt install chrony`，use `chronyc`.
* `chronyc tracking`, `chronyc sources`, `chronyc sourcestats`, `/etc/chrony/chrony.conf`.
* Force resync: `sudo chronyc -a makestep`.
* `sudo service chrony restart` or `sudo systemctl chronyd restart`.

## 网络
* 代理。
  * https://github.com/Qv2ray/Qv2ray download AppImage from it.
  * https://github.com/v2ray/v2ray-core download the pre-built binary zip file.
  * v2rayL may replace Qv2ray + v2ray-core.
* 检查网络 ifconfig, hostname, netstat, nslookup。
* 更有效的 ping：`ping <ip> -s $((60 * 1024)) -D`。
* 查已占用端口 `sudo lsof -i -P -n | grep LISTEN`。
* 查看带宽占用 `sudo iftop -i <card-id> -P`。
* 查看域名解析情况 `ping domain.name` 或者 `nslookup domain.name`。
* wget download mirror of website: wget -m -p http://www.xxx.com，more details blow:
```
wget --recursive --level=inf --page-requisites --convert-links --adjust-extension --span-hosts --domains=domainA,domainB domainA
The shorthand for that would be: wget -rEDpkH -l inf domainA,domainB domainA
-r = --recursive
-l <depth> = --level=<depth>
-E = --adjust-extension
-p = --page-requisites
-K = --backup-converted
-k = --convert-links
-D <domain-list> = --domain-list=<domain-list>
-H = --span-hosts
-np = --no-parent
-U <agent-string> = --user-agent=<agent-string>
```
* axel multithread download: `axel -a -n 12 http://url.to.download`, `-a` shows a progress bar.

## 驱动
* 如果 nvidia-smi 突然说连不上显卡驱动了，可使用 `sudo apt install nvidia-driver-440` 来尝试重新安装。   
  如果显示驱动是已经安装好的，上面的指令会打印出版本号，比如 `450.80.02`。   
  使用 `sudo apt install dkms` 确保 Dynamic Kernel Module Support 工具存在。
  使用 `sudo dkms install -m nvidia -v 450.80.02` 重新生成驱动模块，然后重启电脑。

## 启停
* 卡死重启：按下 `Sys Rq` 键，通常该健与 `Prtscn` 共键，因此 `Alt + Prtscn` 即可，按住 `Sys Rq` 的时候输入 `reisub`：
  1. unRaw 从 X Server 取回键盘控制权；
  2. tErminate 给所有进程发送 SIGTERM 信号；
  3. kIll 给所有进程发送 SIGKILL 信号；
  4. Sync 将所有数据同步至磁盘；
  5. Unmount 将所有分区挂载为只读模式；
  6. reBoot 重启。
  *  助记：busier 倒过来。
* 卡在 boot 界面进不去图形界面，可尝试更换 greeter，从默认 gdm3 换到 lightdm:
  * `apt install slick-greeter`。
  * 用 `dpkg-reconfigure lightdm/gdm3` 可以切换。
* 在启动时显示日志而不是 splash screen：
  * 修改 `/etc/default/grub` 文件中的 GRUB_CMDLINE_LINUX_DEFAULT 的值为空字符串或 "loglevel=0"。
  * 然后使用 `sudo update-grub` 更新 grub 信息。

## 系统管理
* 查看系统版本信息 `lsb_release -a`。
* 查看已安装软件包 `dpkg -l`。
* 查看硬件信息 `lscpu/lshw/hwinfo/lspci/lsscsi/lsusb/lnxi/lsblk/df/fdisk/mount/free/dmidecode/hdparm`，
可以在 /proc 目录下找到一些系统硬件和配置信息。
* 查看 kernel ring buffer：`dmesg`。
* 日志目录 /var/log/。
* 如果某些不想删除的包出现在了 apt autoremove 的列表中，使用 apt install 这些包，可以将其从列表中移除。
* PPA: Personal Package Archive：
  * 使用 `apt install -y software-properties-common` 安装 `add-apt-repository` 指令；
  * 要移除使用 `add-apt-repository` 添加的个人仓库，使用 `apt-key list/del` 来完成。
* 管理应用的名称/版本 `update-alternatives /path/to/symbolic-name /path/to/real/executable`。
* 安装同一程序的多个不同版本之后：
  * 注册不同版本 `sudo update-alternatives --install <bin-path> <bin-name> <specific-version-path> <priority>`。
  * 例子 `sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 7`。
  * 切换版本 `sudo update-alternatives --config <bin-name>`。
  * 例子 `sudo update-alternatives --config gcc`。
* 系统级环境变量设置在 `/etc/profile.d/*.sh` 或 `/etc/environment`，推荐前者。
* 文件系统的挂载配置在 /etc/fstab 中。

## 应用程序
* 通过 deb 文件安装 `sudo dpkg -i /path/to/deb/file && sudo apt install -f`。
* 截屏工具 shutter。
* 音频录制和播放 `arecord/aplay`，编辑 `audacity`。
* 应用图标面板配置位置：~/.local/share/applications/;/usr/share/applications/。
* 应用相关配置指令 `gsettings`。
  * 查看已有目录 `gsettings get org.gnome.desktop.app-folders folder-children`。
  * 查看已有目录下的应用 `gsettings list-keys org.gnome.desktop.app-folders.folder:/org/gnome/desktop/app-folders/folders/Utilities/`。
  * 添加应用目录 `gsettings set org.gnome.destop.app-folders folder-children "[..., 'Audio']"`。
  * 设置目录名称 `gsettings set org.gnome.desktop.app-folders.folder:/org/gnome/desktop/app-folders/folders/Audio/ name 'Audio'`。
  * 添加应用到目录 `gsettings set org.gnome.desktop.app-folders.folder:/org/gnome/desktop/app-folders/folders/Audio/
  apps "['audacious.desktop', 'brasero.desktop']"`。
  * 更详细的解释 https://developer.gnome.org/AppFolders/。
* 手动创建应用程序 launcher 的步骤（https://averagelinuxuser.com/ubuntu_custom_launcher_dock/）：
  * `sudo nano /usr/share/applications/<appname>.desktop`
  * 将下列内容粘贴进去：
  ```
  #!/usr/bin/env xdg-open
  [Desktop Entry]
  Version=1.0
  Type=Application
  Terminal=false
  Exec=/path/to/<appname>
  Name=<AppName>
  Comment=Description of YourApp
  Icon=/path/to/yourapp.png
  ```
  * `sudo chmod +x /usr/share/applications/<appname>.desktop`
  * 退出系统账户后重新登录。

## 用户
* 列出用户 `cat /etc/passwd`，格式：username:password:UID:GID:Comment:HomeDirectory:ShellUsed。
* `cat /etc/passwd | cut -d: -f1` 或 `cat /etc/passwd | awk -F: '{print $1}'`。
* `getent passwd`。
* `who`。
* 列出用户组 `cat /etc/group`，格式：groupname:password:GID:users。
* 查看超级用户的配置 `cat /etc/sudoers`
* 查看系统支持的 shell 类型： `cat /etc/shells`
* 查看系统用户的登录 shell 类型： `cat /etc/passwd`
* 修改系统用户的登录 shell 类型：
  * `usermod --shell /bin/bash <username>`
  * 或者 `chsh --shell /bin/sh <username>`
  * 针对所有域用户 `sudo vim /etc/sssd/sssd.conf` 修改其中的 default_shell 的值
  * 针对当前登录的域用户，`getent passwd $(id -un) | sudo tee -a /etc/passwd`
* 密码的密文在 `/etc/shadow` 中，参见 https://www.cyberciti.biz/faq/where-are-the-passwords-of-the-users-located-in-linux/

## 文件
* 查看某目录占用存储空间 `sudo du -s -h --exclude=./mnt/*`。
* 检查文件属性指令 `file/wc`。
* 文本文件内容相关 `more/less/cat/head/tail/sort/vi/vim/emacs/nano`。
* 进入 emacs 的命令行模式 `emacs -nw`

## 其他
* 两种使命令脱离 shell 执行的方法：
  1. `nohup command_and_paramters &`。
  1. `command_and_paramters </dev/null &>/dev/null &`。
* 查看总体存储空间 `df`。
* /etc/default/grub 包含了系统启停界面的配置。
* qmake 可以通过传参 -qt=qt5 正确运行。
* 解压 `tar xvf file-name.tar.gz`。批量解压 `for f in *.tar.gz; do tar xf "$f"; done`。
* 查找命令相关 `updatedb/mlocate/whereis/find`。
* 查看窗口信息 `xwininfo -id $(xdotool getactivewindow)`。
* 录制屏幕 `ffmpeg -video_size 1280x720 -framerate 25 -f x11grab -i :1.0+99,88 output.mp4`。
* 录制摄像头 `ffmpeg -f v4l2 -framerate 25 -video_size 640x480 -i /dev/video0 output.mkv`。
  * 需要先使用 `v4l2-ctl --list-devices` 确认 v4l2 设备存在，不存在的按提示进行安装即可。
* 转换音频格式 `ffmpeg -i input.flv -f s16le -acodec pcm_s16le output.raw`，使用 `ffmpeg -formats` 查看格式支持。
* Under ubuntu use `cat /etc/X11/default-display-manager` to check using which display manager.
* Under ubuntu use `nautilus /path/to/folder` to open folder with GUI file manager.
* Under ubuntu use `eog <picture-path>` to view picture. Alternatives: `feh/xdg-open/display`.
* 解决 docker 权限问题：
  1. `sudo groupadd docker`。
  1. `sudo usermod -aG docker $USER`。
  1. `newgrp docker`。
* docker 的 image 存储目录：`/var/lib/docker/overlay2`
* docker 常用指令：
  * 获取镜像：`docker pull <image-name>`
  * 查看镜像列表：`docker image ls`
  * 查看进程：`docker ls`
  * 查看进程详情：`docker inspect <container-name>`
  * 查看日志：`docker logs <container-name>`

## 应急恢复
* 恢复被 rm 删除的文件。使用 extundelete 工具。入口命令：`extundelete <disk-path.e.g./dev/sd0> --inode <node-id:number>`。
