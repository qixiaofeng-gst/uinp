# Linux 系统使用

## 启停
* 卡死重启：按下 `Sys Rq` 键，通常该健与 `Prtscn` 共键，因此 `Alt + Prtscn` 即可，按住 `Sys Rq` 的时候输入 `reisub`：
  1. unRaw 从 X Server 取回键盘控制权；
  2. tErminate 给所有进程发送 SIGTERM 信号；
  3. kIll 给所有进程发送 SIGKILL 信号；
  4. Sync 将所有数据同步至磁盘；
  5. Unmount 将所有分区挂载为只读模式；
  6. reBoot 重启。
  *  助记：busier 倒过来。

## 网络
* 代理 https://github.com/Qv2ray/Qv2ray
* 检查网络 ifconfig, hostname, netstat
* 查已占用端口 sudo lsof -i -P -n | grep LISTEN

## 系统管理
* 查看已安装软件包 `dpkg -l`
* 查看硬件信息 `lscpu/lshw/hwinfo/lspci/lsscsi/lsusb/lnxi/lsblk/df/fdisk/mount/free/dmidecode/hdparm`，可以在 /proc 目录下找到一些系统硬件和配置信息。
* 查看 kernel ring buffer：`dmesg`
* 应用图标面板配置位置：~/.local/share/applications/;/usr/share/applications/
* 应用相关配置指令 `gsettings`。
  * 查看已有目录 `gsettings get org.gnome.desktop.app-folders folder-children`
  * 查看已有目录下的应用 `gsettings list-keys org.gnome.desktop.app-folders.folder:/org/gnome/desktop/app-folders/folders/Utilities/`
  * 添加应用目录 `gsettings set org.gnome.destop.app-folders folder-children "[..., 'Audio']"`
  * 设置目录名称 `gsettings set org.gnome.desktop.app-folders.folder:/org/gnome/desktop/app-folders/folders/Audio/ name 'Audio'`
  * 添加应用到目录 `gsettings set org.gnome.desktop.app-folders.folder:/org/gnome/desktop/app-folders/folders/Audio/
  apps "['audacious.desktop', 'brasero.desktop']"`
  * 更详细的解释 https://developer.gnome.org/AppFolders/
* 日志目录 /var/log/
* 如果某些不想删除的包出现在了 apt autoremove 的列表中，使用 apt install 这些包，可以将其从列表中移除
* PPA: Personal Package Archive：
  * 使用 `apt install -y software-properties-common` 安装 `add-apt-repository` 指令；
  * 要移除使用 `add-apt-repository` 添加的个人仓库，使用 `apt-key list/del` 来完成。
* 列出用户 `cat /etc/passwd`，格式：username:password:UID:GID:Comment:HomeDirectory:ShellUsed
* `cat /etc/passwd | cut -d: -f1` 或 `cat /etc/passwd | awk -F: '{print $1}'`
* `getent passwd`
* `who`
* 列出用户组 `cat /etc/group`，格式：groupname:password:GID:users
* 管理应用的名称/版本 `update-alternatives /path/to/symbolic-name /path/to/real/executable`
* 系统级环境变量设置在 `/etc/profile.d/*.sh` 或 `/etc/environment`，推荐前者

## 开发
* 检查 RPATH： `objdump -p <binary> | egrep 'RPATH|RUNPATH'` 或 `readelf -d <binary-or-library> | head -20`
* 查看程序崩溃后的 coredump 调用栈 `coredumpctl gdb _PID_`：
  * coredumpctl 使用 `apt install systemd-coredump` 安装。
  * gdb 中输入 `bt` 查看完整 backtrace。
  * gdb 中输入 `frame <number>` 查看 backtrace 中对应行的 stack frame。
  * gdb 中输入 `list` 可查看改方法附近的代码。
  * gdb 中输入 `info locals` 可查看局部变量。
  * gdb 中输入 `print <variable-name>` 查看变量值。
* 查看程序运行时消息 `perf record -e _EVENTNAME_ -a -g _EXEPATH_`
* 查看 include path：`echo | gcc -E -Wp,-v -`
* 查看窗口信息 `xwininfo -id $(xdotool getactivewindow)`
* 查看 so 版本 `ldconfig -v`，`ldconfig` 指令本身可以更新操作系统的 so 数据库。
* `#include<errno.h>` 然后 `printf("ERROR: %s\n", strerror(errno));` 可查看如 `fopen` 之类的调用出错的信息。
* `nm --demangle path/to/{*.a,*.so}` 可查看符号表。
* `ps huH p <PID_OF_U_PROCESS> | wc -l` 打印线程数目。
### Pyton
* 多版本依赖共存时，需要虚拟环境 venv：`python -m venv --help`
* `pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple`

## 文件
* 查看某目录占用存储空间 `sudo du -s -h --exclude=./mnt/*`
* 检查文件属性指令 `file/wc`
* 文本文件内容相关 `more/less/cat/head/tail/sort/vi/vim/emacs/nano`

## 其他
* 两种使命令脱离 shell 执行的方法：
  1. `nohup command_and_paramters &`
  1. `command_and_paramters </dev/null &>/dev/null &`  
* 查看总体存储空间 `df`
* /etc/default/grub 包含了系统启停界面的配置
* qmake 可以通过传参 -qt=qt5 正确运行
* 解压 `tar zxvf file-name.tar.gz`
* 查找命令相关 `updatedb/mlocate/whereis/find`
* 录制屏幕 `ffmpeg -video_size 1280x720 -framerate 25 -f x11grab -i :1.0+99,88 output.mp4`
* Under ubuntu use `cat /etc/X11/default-display-manager` to check which display manager being used.
* Under ubuntu use `nautilus /path/to/folder` to open folder with GUI file manager
* 解决 docker 权限问题：
  1. sudo groupadd docker
  1. sudo usermod -aG docker $USER
  1. newgrp docker
