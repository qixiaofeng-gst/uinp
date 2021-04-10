# 自省三问：
1. 我的观念中哪些是错误的？
2. 别人觉得深不可测的事情，我能了解多少？
3. 大脑中究竟是什么在误导我？

# 当世所知皆基础，未知一切皆进阶。
What we know is elementary. What we do not know is advanced.

# Quick guide for pjo with gcc:
1. `cd /mnt/c/Users/qixia/Documents/uinp/gists/pjo`
2. `gcc solving.c && less input.txt | ./a.out`
3. `gcc creating.c && ./a.out >> input.txt`

# 护眼色
绿豆沙色能有效的减轻长时间用电脑的用眼疲劳。
色调(Hue)：85，饱和度(Sat)：123，亮度(Lum)：205；
RGB颜色红：199，绿：237，蓝：204；
十六进制颜色：#C7EDCC 或用 #CCE8CF。

redux/mobox

# Linux 系统使用

## 系统安装并启动之后

### 命令行
* 实用的 ll 命令。在 ~/.bashrc 添加：`alias ll='ls -alh'`。
* 神器 tmux，装了不后悔。在 tmux 中，
  * 使用 `C-b ?` 可查看帮助。
  * 使用 `tmux a -t 0` 可进入已打开的会话。
  * 使用 `C-b d` 可退出 tmux 并保持会话后台运行。
  * 使用 `C-b :` 输入 `set mouse` 可启用鼠标。
* 命令行输出出了问题之后有两种方式可以重置：
  * `echo -e "\033c"`
  * `reset`
* 剪贴板工具：
  * clipboard-cli, based on nodejs
  * xclip/xsel, xsel is more friendly with scripting, xclip is better on prompt.
* 三个 bash 的 builtins 用于检查命令或程序：
  * `command -v <cmd>`
  * `hash <cmd>`
  * `type <cmd>`
  * https://stackoverflow.com/questions/592620/how-can-i-check-if-a-program-exists-from-a-bash-script
  * `command -v foo >/dev/null 2>&1 || { echo 'Requiring foo but missing it.'; exit 1; }`

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

## 输入法
中文输入法安装：
* 可以 `sudo apt install ibus-pinyin` 或者 `sudo apt install ibus-sunpinyin`。
* 安装完毕后可以 `ibus restart` 启用。
* 中文输入法有几项很好用的配置（Preferences -> General -> Candidates）：
  * Dynamic adjust the candidates order.
  * Remember every input as a phrase.
  * Show suggestions.

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
* 查看 IP：`hostname -I`。
* 网络管理命令行工具 nmcli：
  * 18.04 重新扫描 wifi：`nmcli device wifi rescan`。
  * 20.04 列出 wifi：`nmcli device wifi list`。
  * 连接 wifi：`nmcli device wifi connect <SSID> password <password>`。
  * 列出网络链接：`nmcli connection show`。
  * 删除某链接历史记录（删除密码）：`nmcli connection delete <name/SSID/BSSID>`。
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
* qaxel multithread download: `axel -a -n 12 http://url.to.download`, `-a` shows a progress bar.

## 任务安排
* crontab
  - `crontab -e` to edit the crontab file
  - minute hour day_of_mount month day_of_week
  - e.g. `0 * * * *` means "at minute 0"
  - Full line: `0 * * * * /home/user_name/task.sh`

## 驱动
* 如果 nvidia-smi 突然说连不上显卡驱动了，可使用 `sudo apt install nvidia-driver-440` 来尝试重新安装。   
  如果显示驱动是已经安装好的，上面的指令会打印出版本号，比如 `450.80.02`。   
  使用 `sudo apt install dkms` 确保 Dynamic Kernel Module Support 工具存在。
  使用 `sudo dkms install -m nvidia -v 450.80.02` 重新生成驱动模块，然后重启电脑。
* U 盘格式化：
  - Simple way:
    - `df -h` check /dev/sda# or /dev/sdb#
    - `sudo umount <dev/xxx>`
    - `sudo mkfs.vfat <dev/xxx>`
  - Advanced way for renaming:
    - `sudo fdisk -l` show details of file system
    - `sudo file <dev/xxx> -s`
    - `sudo apt install mtools` install GNU tools for MSDOS file systems
    - `vim /etc/mtools.conf` then add:  
      drive u: file="/dev/sda1"
    - `sudo minfo -v u:` to check details
    - `sudo mlabel -i /dev/sda1 -s ::"LABEL HERE "` the label have to be 11 characters, no more no less
  - GUI way:
    - Open "Disks" application
    - Select the flash drive in the panel on the left
    - Press the unmount button(should look like a stop button)
    - Click on the gears icon and choose "Edit filesystem"

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
* 查看系统版本信息 `lsb_release -a`，使用 `arch`、`file /sbin/init`、`lscpu` 或 `uname -m` 来查看 architecture。
* Configure timezone:
  * `timedatectl list-timezones`
  * `sudo timedatectl set-timezone Asia/Shanghai`
* 查看已安装软件包 `dpkg -l`。
* 查看已安装的包的相关文件路径 `dpkg -L <package-name>`。
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
* 文件系统的挂载配置在 /etc/fstab 中。修改后重新载入：`mount -a`。
* 系统日志在 `/var/log/syslog` 路径。`dmesg | less` 可查看 kernel ring buffer，这里存的是 bootup 有关的信息。

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
* 切换 root 用户：`sudo -s`。
* 给 sudo 传密码：`echo <password> | sudo -S <command>`，`-S` 参数表明从 stdin 读取密码。
* `cat /etc/passwd | cut -d: -f1` 或 `cat /etc/passwd | awk -F: '{print $1}'`。
* `getent passwd`。
* `who`。
* 列出用户组 `cat /etc/group`，格式：groupname:password:GID:users。
* 查看超级用户的配置 `cat /etc/sudoers`
* 查看系统支持的 shell 类型： `cat /etc/shells`
* 查看系统用户的登录 shell 类型： `cat /etc/passwd`
* 修改系统用户的登录 shell 类型：
  * `usermod --shell /bin/bash $USER`
  * 或者 `chsh --shell /bin/sh $USER`
  * 针对所有域用户 `sudo vim /etc/sssd/sssd.conf` 修改其中的 default_shell 的值
  * 针对当前登录的域用户，`getent passwd $(id -un) | sudo tee -a /etc/passwd`
  * Install zsh with `sudo apt install zsh` then initialize zsh with  
    `sh -c "$(curl -fsSL https://raw.github.com/robbyrussell/oh-my-zsh/master/tools/install.sh)"`
  * After oh-my-zsh initialized, modify .zshrc with a line `ZSH_THEME="ys"` may make the style familiar.
* 密码的密文在 `/etc/shadow` 中，参见 https://www.cyberciti.biz/faq/where-are-the-passwords-of-the-users-located-in-linux/

## 文件
* 查看某目录占用存储空间 `sudo du -s -h --exclude=./mnt/*`。
* 检查文件属性指令 `file/wc`。
* 文本文件内容相关 `more/less/cat/head/tail/sort/vi/vim/emacs/nano`。
  - For vim: ctrl-s freeze vim, ctrl-q unfreeze it
* 进入 emacs 的命令行模式 `emacs -nw`。
* 查看读写的性能 `dd if=/dev/input.file  of=/path/to/output.file  bs=block-size  count=number-of-blocks  oflag=dsync`。
* 有时候 `du -sh` 命令非常慢，尤其是在文件数量特别多的情况下，可以使用这个命令来迂回： `ls -d */ | parallel du -s`。
  * 想要求和可以使用 `ls -d */ | parallel du -s | awk '{s+=$1}END{print s}'` 来达成。
  * 也可以使用 `ls -d */ | parallel du -s | awk '{print $1}' | paste -sd+ | bc -l` 来求和。

## 其他
* 两种使命令脱离 shell 执行的方法：
  1. `nohup command_and_paramters &`。
  1. `command_and_paramters </dev/null &>/dev/null &`。
* 查看总体存储空间 `df`。
* /etc/default/grub 包含了系统启停界面的配置。
* qmake 可以通过传参 -qt=qt5 正确运行。
* 解压 `tar xvf file-name.tar.gz`。批量解压 `for f in *.tar.gz; do tar xf "$f"; done`。
  * 可以指定目录 `tar -C <target-directory> xvf <tar-file>`。
* 查找命令相关 `updatedb/mlocate/whereis/find`。
* 查看窗口信息 `xwininfo -id $(xdotool getactivewindow)`。
* 录制屏幕 `ffmpeg -video_size 1280x720 -framerate 25 -f x11grab -i :1.0+99,88 output.mp4`。
* 录制摄像头 `ffmpeg -f v4l2 -framerate 25 -video_size 640x480 -i /dev/video0 output.mkv`。
  * 需要先使用 `v4l2-ctl --list-devices` 确认 v4l2 设备存在，不存在的按提示进行安装即可。
  * 同时播放 `ffmpeg -f v4l2 -framerate 25 -video_size 640x480 -i /dev/video0 -map 0 -c:v libx264 -f tee "output.mp4|[f=nut]pipe:" | ffplay pipe:`。
* 转换音频格式 `ffmpeg -i input.flv -f s16le -acodec pcm_s16le output.raw`，使用 `ffmpeg -formats` 查看格式支持。
* Under ubuntu use `cat /etc/X11/default-display-manager` to check using which display manager.
* Under ubuntu use `nautilus /path/to/folder` to open folder with GUI file manager.
* Under ubuntu use `eog <picture-path>` to view picture. Alternatives: `feh/xdg-open/display`.

## 应急恢复
* 恢复被 rm 删除的文件。使用 extundelete 工具。入口命令：`extundelete <disk-path.e.g./dev/sd0> --inode <node-id:number>`。

# 开发信息
* 图解开源协议 https://www.cnblogs.com/KruceCoder/p/7991052.html
* 查看可执行文件汇编信息：`objdump -S -d/--disassemble <file-path>`，`-S` 用于显示混在可执行文件中的源码（如果有）。
* 检查 RPATH：
  * `objdump -p <binary> | egrep 'RPATH|RUNPATH'`
  * `readelf -d <binary-or-library> | head -20`
  * `ldd <binary>`
* 将任意文件转换成 16 进制文本：`xxd <file-path>`，使用 `xxd -r <file-path>` 反向转换。类似工具还有 `hexdump`。
* 查看程序崩溃后的 coredump 调用栈 `coredumpctl gdb _PID_`：
  * coredumpctl 使用 `apt install systemd-coredump` 安装。
  * gdb 中输入 `bt` 查看完整 backtrace。
  * gdb 中输入 `frame <number>` 查看 backtrace 中对应行的 stack frame。
  * gdb 中输入 `list` 可查看改方法附近的代码。
  * gdb 中输入 `info locals` 可查看局部变量。
  * gdb 中输入 `print <variable-name>` 查看变量值。
* 查看程序运行时消息 `perf record -e _EVENTNAME_ -a -g _EXEPATH_`。
* 查看 include path：`echo | gcc -E -Wp,-v -`。
* 查看 so 版本 `ldconfig -v`，`ldconfig` 指令本身可以更新操作系统的 so 数据库。
* `#include<errno.h>` 然后 `printf("ERROR: %s\n", strerror(errno));` 可查看如 `fopen` 之类的调用出错的信息。
* `nm --demangle path/to/{*.a,*.so}` 可查看符号表。
* `ps huH p <PID_OF_U_PROCESS> | wc -l` 打印线程数目。
* 监控 GPU 状态 `watch -n 3 nvidia-smi`。
* 查 GPU 核数 `nvidia-settings -q CUDACores -t`。
## Docker
* 解决 docker 权限问题：
  1. `sudo groupadd docker`。
  1. `sudo usermod -aG docker $USER`。
  1. `newgrp docker`。
  1. 重启。
* docker 的 image 存储目录：`/var/lib/docker/overlay2`
  * How to modify: https://gist.github.com/nileshsimaria/ec2ea6847d494d2a1935c95d7c4b7155
* docker 服务重启：`sudo systemctl restart docker`
* docker 常用指令：
  * 获取镜像：`docker pull <image-name>`
  * 查看镜像列表：`docker image ls`
  * 查看进程：`docker ls`
  * 查看进程详情：`docker inspect <container-name>`
  * 查看日志：`docker logs -f <container-name/hash>`
  * 移除所有停止的容器：`docker container prune`
  * 移除所有 TAG=none 的镜像：`docker rmi $(docker images -f "dangling=true" -q)`
  * 登陆：`docker login <url>`
  * 查看完整容器 entry 命令：`docker ps --no-trunc`
## C/C++
* C/C++ 当中，程序抛出异常退出前的打印，如果是 printf，要加 '\n'，不加则可能没有输出。
* C on linux, `struct S a = *b;`, `b` pointed struct `S` is copied to `a`.
* C on linux, public struct should be put in header file, private one put in .c file.
* C 的 `volatile` 关键字，指示访问变量值时访问内存，不访问寄存器或缓存的值。
* 当 `static` 关键字用在 C 的方法上时，表明该方法只能在当前文件中使用。
* 当 ld (linker) 程序不是直接被调用时，比如通过 gcc 调用，传给它的参数都需要用 `-Wl,` 作为前缀。
* 在 compiler 的参数中 `-W` 开头的参数都是针对 warning 的。
## Python
* 多版本依赖共存时，需要虚拟环境 venv：`python -m venv --help`。
* Anaconda Usage:
  ```bash
  conda env export > environment.yaml
  conda env create -f environment.yaml
  conda env remove --name <env-name>
  
  conda create -n <env-name> <package-spec>
  # eg. conda create -n tf-py3.7 python=3.7
  ```
* 清华开源 https://mirror.tuna.tsinghua.edu.cn/help/ubuntu/
  * 设置 pip 镜像：`pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple`
* 安装 tensorflow 成功的前提是安装 python3.5.2，然后
  `pip install --upgrade https://storage.googleapis.com/tensorflow/windows/cpu/tensorflow-1.0.0-cp35-cp35m-win_amd64.whl`
升级的方法：
  ```
  pip search --version tensorflow
  pip install --upgrade tensorflow==1.1.0rc1
  ```
* 检查 tensorflow 版本 `python3 -c 'import tensorflow as tf; print(tf.__version__)'`
* WebStorm 编辑器批量处理行尾：选中一目录后，菜单 File -> LineSeparators 选择一个行尾格式即可。
* 查看系统中已经安装的 python 模块：`pydoc modules`/`pip list/freeze`/`help("modules")`
* python 的依赖管理工具的配置文件位置：
  * setuptools、easy_install: `~/.pydistutils.cfg`
  ```cfg
  [easy_install]
  index-url=<the-url>
  ```
  * pip: `~/.pip/pip.conf` 或者 `~/.config/pip/pip.conf`
  ```cfg
  [global]
  index-url = <the-url>
  ```
  * 阿里云的源： https://mirrors.aliyun.com/pypi/simple/
  * 临时改源 `pip install -i <mirror-url> --trusted-host <domain-name> <module-name>==<version>`
* 使用隐藏的方式解决 ‘declared with greater visibility than the type of its field’ 的问题将导致 so 文件报 ‘undefined referece’ 错误。
* Duck-typing 其实是无类型，执行任何运算时只看对象是否拥有所需的操作符或方法。

# cmake
## RPATH vs RUNPATH
* RUNPATH 只管当前二进制文件的依赖，不管级联依赖（依赖的文件的依赖，以及更深层的依赖）。
* RPATH 管全部的依赖。使用 `set(CMAKE_EXE_LINKER_FLAGS "-Wl,--disable-new-dtags")` 可启用。
* 如果 RUNPATH 和 RPATH 在依赖链中混合存在，可能会出现运行时找不到 so 文件的错误，一般将入口可执行文件的 RPATH 启用就好。

# 编辑器
## Atom
* 格式插件：atom-latex/latex, markdown-preview
* 格式插件：atom-latex/latex, markdown-preview
* 效率插件：cursor-history
* https://www.sitepoint.com/10-essential-atom-add-ons/
## IntelliJ
* Add Path for Python Project: `Settings(ctl+alt+s) -> Project Structure -> Add Content Root`.
* Add Launcher：`Tools(menu) -> Create Desktop Entry`。
## Emacs
* 在 github 找到 zenburn-emacs 项目并配置好该主题。
* Windows 中需要设置 HOME 环境变量用于设置指定 “~” 路径。
* 安装基于 el 语言的 helm 和 helm-gtags。需要先安装 emacs 的包管理工具 straight。
* 使用 `sudo apt install global` 安装 GNU global。
* 快捷键备忘：
  * 文件内容浏览：`C-n/p/f/b/v/l, M-v/>/<`
  * 缓存操作：`C-x b, C-x C-b`
  * 文件操作：`C-x C-f, C-x C-s, C-x C-c, C-x C-q`
  * 窗口操作：`C-x 0/1/2/3`
  * 跳转：`M-.`
  * 代码操作：`C-M-q, C-M-x`
  * 保存特殊 buffer 的内容：`C-x h` 全选，然后 `M-x write-region`
  * 搜索：
    * 文件内：`C-s, C-r`
    * 通过文件名搜索：有 helm 时使用 `C-x C-f`
    * 通过 grep 搜索：`M-x grep, M-x lgrep, M-x grep-find, M-x find-grep, M-x rgrep, M-x zrgrep, M-x kill-grep`
  * 撤销：`C-/`, `M-x revert-buffer`
  * 自动补完：`M-/`
  * 重载 el 文件：`M-x load-file`，如果当前文件打开着，`M-x eval-buffer`
  * 替换：`M-%`，询问时按 `!` 即可替换所有。

# 常用正则
* 分节删除：` \r\n-+\r\n\r\n分节阅读\s.+\r\n\r\n `。
* 等号空格：` (\w)=(\w) ` 替换 ` $1 = $2 `。

# 版本工具备忘
* SVN http://earth.bao.ac.cn/svn/gsegment/trunk/***
* GIT git@github.com:qixiaofeng/***
* 当 .gitignore 不生效时，用下列两语句
  ```
  git rm -r --cached .
  git add .
  ```
* 修改最近一次 commit 的信息 `git commit --amend`
* 修改分支名称：
  - `git branch -m <new-name>`
  - `git push origin -u <new-name>`
  - `git push origin --delete <old-name>`
* 切换到远程分支 `git co --track origin/<branch-name>`
* 切换分支 `git co <branch-name>`
* 新建分支 `git co -b <branch-name>` + `git push origin <branch-name>`
* 下载并合并到本地当前分支 `git pull`，仅下载 `git fetch`
* 撤销上次提交并保留所有修改 `git reset --soft <revision-hash>`
* 撤销所有进行中的修改（包括解决冲突的过程中） `git reset --hard HEAD`
* 回退某个文件到历史版本 `git co <revision-hash> -- file/path`
* 当解决冲突时，可使用 `git co --ours/theirs -- file/path` 来决定使用某方的版本
* 查看 head 与前一 commit 的差异 `git diff head~1 head`
* 关闭自动行尾转换 `git config --global core.autocrlf false`
* 清理所有已删除分支 `git fetch --prune --all` 或者 `git remote prune origin`
* 回退一个 commit `git revert <commit-hash>`
* 推送本地工程到远程空工程
  - `git remote add origin git@blablabla:url`
  - `git pull origin master --allow-unrelated-histories`
  - `git push -u origin master`
* 切换远程 URL `git remote set-url origin <url>`
* 查看远程分支 `git branch -r` or `git branch -a`
* 保存用户密码 `git config --global credential.helper store`，取消 `git config --global --unset credential.helper`。
* 新增 tag `git tag -a <tag-name> <commit-hash> && git push origin <tag-name>`
* 删除 tag `git tag -d <tag-name> && git push origin --delete <tag-name>`
* git 子模块相关：
  * 添加：`git submodule add -b <branch-name> <url-to-repo>` 然后 `git submodule init`。
  * 取得：`git submodule update --init --recursive`。
  * 更新：`git pull --recurse-submodules` 或者 `git submodule update --remote`。
* 查看库中所有文件：`git ls-tree --full-tree -r --name-only HEAD`

# 常用指令
* 使用命令行进行文本查找：`grep 'target-pattern' ./* -R`。
* 软链接 link-node -s target_to_link link_name
* 在 windows 下使命令脱离 cmd 执行的方法（关闭 cmd 后进程将退出，因此大多数情况无效）：
  `start "command_name" /B command_and_paramters > somefile.txt`
* windows 下环境变量相关命令：
  ```
  @rem List environment variables:
  set
  @rem Set user variables:
  setx VNAME "path\or\something"
  @rem Set system variables:
  setx VNAME "%VREF%\path\or\something" /M
  ```
* 将命令行输出的内容存到文件的同时仍输出到命令行：`<some-command> 2>&1 | tee <some-file>`。如果不需要 stderr 则去掉 2。
* 要将命令行的 stderr 和 stdout 重定向，可以使用 `2>&1` 或者 `&>`。重定向中用到的标识符：
  * `0` - stdin，`1` - stdout，`2` - stderr。
  * `n>` 指重定向输出，`n` 被省略时默认为 `1`。
  * `>>` 指添加而非重写。
  * `n<` 指重定向输入，`n` 省略时为 `0`。
  * https://www.gnu.org/software/bash/manual/html_node/Redirections.html

# Windows 中安装 MongoDB 服务
* 创建数据文件和日志文件目录；
  1. 以下列格式创建配置：
  ```
  systemLog:
    destination: file
    path: c:\data\log\mongod.log
  storage:
    dbPath: c:\data\db
  ```
* 安装服务：mongod.exe --config "config/file/path" --install
* 启动：net start MongoDB
* Windows 10 中在管理员权限下的 PowerShell 中（c:/windows/system32/windowspowershell 目录下找 powershell）关闭防火墙：
Set-NetFirewallProfile -Profile Domain, Public,Private -Enabled false

# Web 前端开发跨域 Chrome 设置
"C:/Program Files (x86)/Google/Chrome/Application/chrome.exe" --disable-web-security --user-data-dir="D:/TempDocs/chrome_data/"
Windows 下可创建快捷方式并添加参数

# 杂项
* Windows 重启 LxssManager 服务可重启 Linux 子系统
* Accessing Linux (WSL) files from Windows using \\wsl$
* ffplay 播放字幕 ffplay path/to/video -vf subtitles=path/to/text
* Java 执行指定 jar 包中的指定 class 和 library path 的命令行：
`java -Djava.library.path=dir/path -cp xxx.jar xxx.xxx.ClassName`
* 安装 sdkman：`curl -s "https://get.sdkman.io" | bash`，安装指定版本的 gradle：`sdk install gradle 6.7`。
* eclipse 中配置 ${user} 变量的方法：在 eclipse.ini 中的 -vmargs 之后一行添加 -Duser.name=XXX
* SQL 拷贝表数据语句：insert into table_1 (column_1, colum_2) select column_a, column_b from table_2;
* 如果 select column_names from table; 的 column_names 中有常量值，比如数值或字符串，则在返回结果中该列是该常量值
* select 语句可以用在 from 和 where 之后
* from 之后可以放多个表（或 select 语句），此时 from 对应的 select 语句中所有列需带表名
* SVN 解决树冲突，需先 svn resolve --accept=working，accept 的参数值随需要而定
* git-bash 出现 There are no available terminals (-1) 的错误时，用 cmd 输入 tasklist 找到 ssh 或者 ssh-agent 之类的进程，
  然后 taskkill /F /IM xxx.exe 干掉，通常能解决问题
* 检查文件签名 sha256sum/md5sum the-file-to-check 与网站列出的 sha256/md5 签名进行比对
* gpg 检查文件签名，`gpg --verify sig/file/path` or `gpg -d sig/file/path`，
  gpg 将自动检测同目录下无后缀同名文件，如果报出 No public key 的问题，可查看输出中的 RSA id，
  调 `gpg --search-keys the_rsa_id` 按命令行提示操作可导入 public key，`gpg --list-keys` 可列出所有已导入的 public key，
  `gpg --edit-key the_rsa_id` 可对相应的 public key 进行编辑。
* gpg 不提示密码的问题：https://unix.stackexchange.com/questions/395875/gpg-does-not-ask-for-password
  ~/.gnupg/gpg-agent.conf 中有两个值 default-cache-ttl（自上次使用计） 和 max-cache-ttl（总计）。
  conf 文件的配置格式：name value
* PDF 文件中的默认尺寸单位是 point (72 points = 1")，要将 point 转换成 pixel，
  需要依赖实际的 DPI(Dots Per Inch, 或者叫 PPI，Pixels Per Inch)。
  * The visual resolution of the human eye is about 1 arc minute. At a viewing distance of 20", that
    translates to about 170 DPI, which equals a dot pitch of around 0.14 mm.

# Legacy for GST JoyCity project
```
db.user.find({ 'mc_member.Mobile': '18621508640' })
db.user.find({ _id: ObjectId('5d706e4ee87c084e92637021') })
                5d706ec4e87c084e92637025
db.admin_order.find({ pay_id: ObjectId('5d706ec4e87c084e92637025') })
db.admin_gb.find({ _id: ObjectId('5d69fbc6e87c084e92635e3b') })
db.admin_order.find({ group_id: ObjectId('5d69fbc6e87c084e92635e3b') })
more /hd1/forever_logs/gOlQ.log | grep -C 20 '2019-8-31 12:46:5'
```
