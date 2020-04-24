# 自省三问：
1. 我的观念中哪些是错误的？
2. 别人觉得深不可测的事情，我能了解多少？
3. 大脑中究竟是什么在误导我？

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
* Under ubuntu use `cat /etc/X11/default-display-manager` to check which display manager being used.
* Under ubuntu use `nautilus /path/to/folder` to open folder with GUI file manager

# Quick guide for pjo with gcc:
1. `cd /mnt/c/Users/qixia/Documents/uinp/gists/pjo`
2. `gcc solving.c && less input.txt | ./a.out`
3. `gcc creating.c && ./a.out >> input.txt`

**Keep fit: Jeff Cavaliere 5 minutes**  
redux/mobox

# 开发信息
* 图解开源协议 https://www.cnblogs.com/KruceCoder/p/7991052.html
* 安装 tensorflow 成功的前提是安装 python3.5.2，然后
  `pip install --upgrade https://storage.googleapis.com/tensorflow/windows/cpu/tensorflow-1.0.0-cp35-cp35m-win_amd64.whl`
升级的方法：
  ```
  pip search --version tensorflow
  pip install --upgrade tensorflow==1.1.0rc1
  ```
* 检查 tensorflow 版本 `python3 -c 'import tensorflow as tf; print(tf.__version__)'`
* WebStorm 编辑器批量处理行尾：选中一目录后，菜单 File -> LineSeparators 选择一个行尾格式即可。
* C on linux, `struct S a = *b;`, b pointed struct S is copied to a.
* C on linux, public struct should be put in header file, private one put in .c file.
* 查看系统中已经安装的 python 模块：`pydoc modules`/`pip list/freeze`/`help("modules")`
* Duck-typing 其实是无类型，执行任何运算时只看对象是否拥有所需的操作符或方法。

# 版本工具备忘
* SVN http://earth.bao.ac.cn/svn/gsegment/trunk/***
* GIT git@github.com:qixiaofeng/***
* 当 .gitignore 不生效时，用下列两语句
  ```
  git rm -r --cached .
  git add .
  ```
* 切换到远程分支 `git co --track origin/branch-name`
* 切换分支 `git co branch-name`
* 新建分支 `git co -b branch-name` + `git push origin branch-name`
* 下载并合并到本地当前分支 `git pull`，仅下载 `git fetch`
* 撤销上次提交并保留所有修改 `git reset --soft revision-hash`
* 回退某个文件到历史版本 `git co revision-hash -- file/path`
* 查看 head 与前一 commit 的差异 `git diff head~1 head`
* 关闭自动行尾转换 `git config --global core.autocrlf false`
* 清理所有已删除分支 `git fetch --prune --all` 或者 `git remote prune origin`
* 推送本地工程到远程空工程
  ```
  git pull origin master --allow-unrelated-histories
  git remote add origin git@blablabla:url
  git push -u origin master
  ```
* 查看远程分支 `git branch -r` or `git branch -a`
* 保存用户密码 `git config --global credential.helper store`

# 常用指令
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

# 常用正则
* 分节删除：`\r\n-+\r\n\r\n分节阅读\s.+\r\n\r\n`

# Web 前端开发跨域 Chrome 设置
"C:/Program Files (x86)/Google/Chrome/Application/chrome.exe" --disable-web-security --user-data-dir="D:/TempDocs/chrome_data/"
Windows 下可创建快捷方式并添加参数

# 杂项
* Windows 重启 LxssManager 服务可重启 Linux 子系统
* Accessing Linux (WSL) files from Windows using \\wsl$
* ffplay 播放字幕 ffplay path/to/video -vf subtitles=path/to/text
* Java 执行指定 jar 包中的指定 class 和 library path 的命令行：
`java -Djava.library.path=dir/path -cp xxx.jar xxx.xxx.ClassName`
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
