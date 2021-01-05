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
## C/C++
* C/C++ 当中，程序抛出异常退出前的打印，如果是 printf，要加 '\n'，不加则可能没有输出。
* C on linux, `struct S a = *b;`, `b` pointed struct `S` is copied to `a`.
* C on linux, public struct should be put in header file, private one put in .c file.
* C 的 `volatile` 关键字，指示访问变量值时访问内存，不访问寄存器或缓存的值。
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
* 安装基于 el 语言的 helm 和 helm-gtags。需要先安装 emacs 的包管理工具 straight。
* 使用 `sudo apt install global` 安装 GNU global。
* 在 github 找到 zenburn-emacs 项目并配置好该主题。
* 快捷键备忘：
  * 文件内容浏览：`C-n/p/f/b/v/l, M-v/>/<`
  * 缓存操作：`C-x b, C-x C-b`
  * 文件操作：`C-x C-f, C-x C-s, C-x C-c, C-x C-q`
  * 窗口操作：`C-x 0/1/2/3`
  * 跳转：`M-.`
  * 代码操作：`C-M-q, C-M-x`

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
* 切换到远程分支 `git co --track origin/<branch-name>`
* 切换分支 `git co <branch-name>`
* 新建分支 `git co -b <branch-name>` + `git push origin <branch-name>`
* 下载并合并到本地当前分支 `git pull`，仅下载 `git fetch`
* 撤销上次提交并保留所有修改 `git reset --soft <revision-hash>`
* 撤销所有进行中的修改（包括解决冲突的过程中） `git reset --hard HEAD`
* 回退某个文件到历史版本 `git co <revision-hash> -- file/path`
* 查看 head 与前一 commit 的差异 `git diff head~1 head`
* 关闭自动行尾转换 `git config --global core.autocrlf false`
* 清理所有已删除分支 `git fetch --prune --all` 或者 `git remote prune origin`
* 回退一个 commit `git revert <commit-hash>`
* 推送本地工程到远程空工程
  ```
  git remote add origin git@blablabla:url
  git pull origin master --allow-unrelated-histories
  git push -u origin master
  ```
* 切换远程 URL `git remote set-url origin <url>`
* 查看远程分支 `git branch -r` or `git branch -a`
* 保存用户密码 `git config --global credential.helper store`，取消 `git config --global --unset credential.helper`。
* 新增 tag `git tag -a <tag-name> <commit-hash> && git push origin <tag-name>`
* 删除 tag `git tag -d <tag-name> && git push origin --delete <tag-name>`
* git 子模块相关：
  * 添加：`git submodule add -b <branch-name> <url-to-repo>` 然后 `git submodule init`。
  * 取得：`git submodule update --init --recursive`。
  * 更新：`git pull --recurse-submodules` 或者 `git submodule update --remote`。

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
