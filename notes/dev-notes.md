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
* 清华开源 https://mirror.tuna.tsinghua.edu.cn/help/ubuntu/
  * 设置 pip 镜像：`pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple`
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
* Duck-typing 其实是无类型，执行任何运算时只看对象是否拥有所需的操作符或方法。
* C/C++ 当中，程序抛出异常退出前的打印，如果是 printf，要加 '\n'，不加则可能没有输出。
* Anaconda Usage:
  ```bash
  conda env export > environment.yaml
  conda env create -f environment.yaml
  conda env remove --name <env-name>
  
  conda create -n <env-name> <package-spec>
  # eg. conda create -n tf-py3.7 python=3.7
  ```
* 查看可执行文件汇编信息：`objdump -S -d/--disassemble <file-path>`，`-S` 用于显示混在可执行文件中的源码（如果有）。
* 将任意文件转换成 16 进制文本：`xxd <file-path>`，使用 `xxd -r <file-path>` 反向转换。类似工具还有 `hexdump`。

# 编辑器
## Atom
* 格式插件：atom-latex/latex, markdown-preview
* 效率插件：cursor-history
* https://www.sitepoint.com/10-essential-atom-add-ons/
## IntelliJ
* Add Path for Python Project: `Settings(ctl+alt+s) -> Project Structure -> Add Content Root`.
* Add Launcher：`Tools(menu) -> Create Desktop Entry`。

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
* 回退某个文件到历史版本 `git co <revision-hash> -- file/path`
* 查看 head 与前一 commit 的差异 `git diff head~1 head`
* 关闭自动行尾转换 `git config --global core.autocrlf false`
* 清理所有已删除分支 `git fetch --prune --all` 或者 `git remote prune origin`
* 回退一个 commit `git revert <commit-hash>`
* 推送本地工程到远程空工程
  ```
  git pull origin master --allow-unrelated-histories
  git remote add origin git@blablabla:url
  git push -u origin master
  ```
* 切换远程 URL `git remote set-url origin <url>`
* 查看远程分支 `git branch -r` or `git branch -a`
* 保存用户密码 `git config --global credential.helper store`
* 新增 tag `git tag -a <tag-name> <commit-hash> && git push origin <tag-name>`
* 删除 tag `git tag -d <tag-name> && git push origin --delete <tag-name>`

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
