# Linux 系统使用
* 两种使命令脱离 shell 执行的方法：
  1. `nohup command_and_paramters &`
  1. `command_and_paramters </dev/null &>/dev/null &`
* 查已占用端口 sudo lsof -i -P -n | grep LISTEN
* 查看某目录占用存储空间 `sudo du -s -h --exclude=./mnt/*`
* 查看总体存储空间 `df`
* 查看已安装软件包 `dpkg -l`
* 检查网络 ifconfig, hostname, netstat
* /etc/default/grub 包含了系统启停界面的配置
* qmake 可以通过传参 -qt=qt5 正确运行
* 解压 `tar zxvf file-name.tar.gz`
* 检查 RPATH： `objdump -p <binary> | egrep 'RPATH|RUNPATH'` 或 `readelf -d <binary-or-library> | head -20`
* 查看程序崩溃后的 coredump 调用栈 `coredump gdb _PID_`；然后 gdb 中输入 bt
* 查看程序运行时消息 `perf record -e _EVENTNAME_ -a -g _EXEPATH_`
* 查看 include path：`echo | gcc -E -Wp,-v -`
* 查看硬件信息 `lscpu/lshw/hwinfo/lspci/lsscsi/lsusb/lnxi/lsblk/df/fdisk/mount/free/dmidecode/hdparm`，可以在 /proc 目录下找到一些系统硬件和配置信息。
* 查看 kernel ring buffer：`dmesg`
* 查找命令相关 `updatedb/mlocate/whereis/find`
* 检查文件属性指令 `file/wc`
* 应用图标面板配置位置：~/.local/share/applications/;/usr/share/applications/
* 应用相关配置指令 `gsettings`。
  * 查看已有目录 `gsettings get org.gnome.desktop.app-folders folder-children`
  * 查看已有目录下的应用 `gsettings list-keys org.gnome.desktop.app-folders.folder:/org/gnome/desktop/app-folders/folders/Utilities/`
  * 添加应用目录 `gsettings set org.gnome.destop.app-folders folder-children "[..., 'Audio']"`
  * 设置目录名称 `gsettings set org.gnome.desktop.app-folders.folder:/org/gnome/desktop/app-folders/folders/Audio/ name 'Audio'`
  * 添加应用到目录 `gsettings set org.gnome.desktop.app-folders.folder:/org/gnome/desktop/app-folders/folders/Audio/
  apps "['audacious.desktop', 'brasero.desktop']"`
  * 更详细的解释 https://developer.gnome.org/AppFolders/
* 文本文件内容相关 `more/less/cat/head/tail/sort/vi/vim/emacs/nano`
* 日志目录 /var/log/
* 如果某些不想删除的包出现在了 apt autoremove 的列表中，使用 apt install 这些包，可以将其从列表中移除
* 查看窗口信息 `xwininfo -id $(xdotool getactivewindow)`
* 录制屏幕 `ffmpeg -video_size 1280x720 -framerate 25 -f x11grab -i :1.0+99,88 output.mp4`
* 管理应用的名称/版本 `update-alternatives /path/to/symbolic-name /path/to/real/executable`
* Under ubuntu use `cat /etc/X11/default-display-manager` to check which display manager being used.
* Under ubuntu use `nautilus /path/to/folder` to open folder with GUI file manager
* PPA: Personal Package Archive：
  * 使用 `apt install -y software-properties-common` 安装 `add-apt-repository` 指令；
  * 要移除使用 `add-apt-repository` 添加的个人仓库，使用 `apt-key list/del` 来完成。
