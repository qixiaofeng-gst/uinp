# Quick guide for pjo with gcc:
1. `cd /mnt/c/Users/qixia/Documents/uinp/gists/pjo`
2. `gcc solving.c && less input.txt | ./a.out`
3. `gcc creating.c && ./a.out >> input.txt`

**Keep fit: Jeff Cavaliere 5 minutes**  
redux/mobox

# 自省三问：
1. 我的观念中哪些是错误的？
2. 别人觉得深不可测的事情，我能了解多少？
3. 大脑中究竟是什么在误导我？

```
db.user.find({ 'mc_member.Mobile': '18621508640' })
db.user.find({ _id: ObjectId('5d706e4ee87c084e92637021') })
                              5d706ec4e87c084e92637025
db.admin_order.find({ pay_id: ObjectId('5d706ec4e87c084e92637025') })
db.admin_gb.find({ _id: ObjectId('5d69fbc6e87c084e92635e3b') })
db.admin_order.find({ group_id: ObjectId('5d69fbc6e87c084e92635e3b') })
more /hd1/forever_logs/gOlQ.log | grep -C 20 '2019-8-31 12:46:5'
```

# 开发信息
* 图解开源协议 https://www.cnblogs.com/KruceCoder/p/7991052.html
* 安装 tensorflow 成功的前提是安装 python3.5.2，然后 `pip install --upgrade https://storage.googleapis.com/tensorflow/windows/cpu/tensorflow-1.0.0-cp35-cp35m-win_amd64.whl`\
升级的方法：
```
pip search --version tensorflow
pip install --upgrade tensorflow==1.1.0rc1
```
* 检查 tensorflow 版本 `python3 -c 'import tensorflow as tf; print(tf.__version__)'`
* WebStorm 编辑器批量处理行尾：选中一目录后，菜单 File -> LineSeparators 选择一个行尾格式即可。

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
* 在 linux 下，两种使命令脱离 shell 执行的方法：
  1. nohup command_and_paramters &
  1. command_and_paramters </dev/null &>/dev/null &
* linux 查已占用端口 sudo lsof -i -P -n | grep LISTEN
* linux 下查看某目录占用存储空间 `sudo du -s -h --exclude=./mnt/*`
* linux 下查看总体存储空间 `df`
* Under ubuntu use `cat /etc/X11/default-display-manager` to check which display manager being used.
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
* git-bash 出现 There are no available terminals (-1) 的错误时，用 cmd 输入 tasklist 找到 ssh 或者 ssh-agent 之类的进程，然后 taskkill /F /IM xxx.exe 干掉，通常能解决问题
* linux 下查看系统中已经安装的 python 模块：pydoc modules
* 检查文件签名 sha256sum/md5sum the-file-to-check 与网站列出的 sha256/md5 签名进行比对
* gpg 检查文件签名，`gpg --verify sig/file/path` or `gpg -d sig/file/path`，gpg 将自动检测同目录下无后缀同名文件，如果报出 No public key 的问题，可查看输出中的 RSA id，调 `gpg --search-keys the_rsa_id` 按命令行提示操作可导入 public key，`gpg --list-keys` 可列出所有已导入的 public key，`gpg --edit-key the_rsa_id` 可对相应的 public key 进行编辑
* linux 下检查网络 ifconfig, hostname, netstat
* linux 下 /etc/default/grub 包含了系统启停界面的配置
* qmake 可以通过传参 -qt=qt5 正确运行
* 解压 `tar zxvf file-name.tar.gz`
* Linux 检查 RPATH： `objdump -p <binary> | egrep 'RPATH|RUNPATH'` 或 `readelf -d <binary-or-library> | head -20`
* Linux 查看 `coredump gdb _PID_`；然后 gdb 中输入 bt
* Linux 查看消息 `perf record -e _EVENTNAME_ -a -g _EXEPATH_`
* Linux 查看 include path：`echo | gcc -E -Wp,-v -`

wget download mirror of website: wget -m -p http://www.xxx.com，more details blow:
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

WebStorm 激活码
```
D6KY031L1G-eyJsaWNlbnNlSWQiOiJENktZMDMxTDFHIiwibGljZW5zZWVOYW1lIjoi5o6I5p2D5Luj55CG5ZWGOiB3d3cuaTkub3JnIiwiYXNzaWduZWVOYW1lIjoiIiwiYXNzaWduZWVFbWFpbCI6IiIsImxpY2Vuc2VSZXN0cmljdGlvbiI6IiIsImNoZWNrQ29uY3VycmVudFVzZSI6ZmFsc2UsInByb2R1Y3RzIjpbeyJjb2RlIjoiSUkiLCJmYWxsYmFja0RhdGUiOiIyMDE5LTA3LTIyIiwicGFpZFVwVG8iOiIyMDIwLTA3LTIxIn0seyJjb2RlIjoiQUMiLCJmYWxsYmFja0RhdGUiOiIyMDE5LTA3LTIyIiwicGFpZFVwVG8iOiIyMDIwLTA3LTIxIn0seyJjb2RlIjoiRFBOIiwiZmFsbGJhY2tEYXRlIjoiMjAxOS0wNy0yMiIsInBhaWRVcFRvIjoiMjAyMC0wNy0yMSJ9LHsiY29kZSI6IlBTIiwiZmFsbGJhY2tEYXRlIjoiMjAxOS0wNy0yMiIsInBhaWRVcFRvIjoiMjAyMC0wNy0yMSJ9LHsiY29kZSI6IkdPIiwiZmFsbGJhY2tEYXRlIjoiMjAxOS0wNy0yMiIsInBhaWRVcFRvIjoiMjAyMC0wNy0yMSJ9LHsiY29kZSI6IkRNIiwiZmFsbGJhY2tEYXRlIjoiMjAxOS0wNy0yMiIsInBhaWRVcFRvIjoiMjAyMC0wNy0yMSJ9LHsiY29kZSI6IkNMIiwiZmFsbGJhY2tEYXRlIjoiMjAxOS0wNy0yMiIsInBhaWRVcFRvIjoiMjAyMC0wNy0yMSJ9LHsiY29kZSI6IlJTMCIsImZhbGxiYWNrRGF0ZSI6IjIwMTktMDctMjIiLCJwYWlkVXBUbyI6IjIwMjAtMDctMjEifSx7ImNvZGUiOiJSQyIsImZhbGxiYWNrRGF0ZSI6IjIwMTktMDctMjIiLCJwYWlkVXBUbyI6IjIwMjAtMDctMjEifSx7ImNvZGUiOiJSRCIsImZhbGxiYWNrRGF0ZSI6IjIwMTktMDctMjIiLCJwYWlkVXBUbyI6IjIwMjAtMDctMjEifSx7ImNvZGUiOiJQQyIsImZhbGxiYWNrRGF0ZSI6IjIwMTktMDctMjIiLCJwYWlkVXBUbyI6IjIwMjAtMDctMjEifSx7ImNvZGUiOiJSTSIsImZhbGxiYWNrRGF0ZSI6IjIwMTktMDctMjIiLCJwYWlkVXBUbyI6IjIwMjAtMDctMjEifSx7ImNvZGUiOiJXUyIsImZhbGxiYWNrRGF0ZSI6IjIwMTktMDctMjIiLCJwYWlkVXBUbyI6IjIwMjAtMDctMjEifSx7ImNvZGUiOiJEQiIsImZhbGxiYWNrRGF0ZSI6IjIwMTktMDctMjIiLCJwYWlkVXBUbyI6IjIwMjAtMDctMjEifSx7ImNvZGUiOiJEQyIsImZhbGxiYWNrRGF0ZSI6IjIwMTktMDctMjIiLCJwYWlkVXBUbyI6IjIwMjAtMDctMjEifSx7ImNvZGUiOiJSU1UiLCJmYWxsYmFja0RhdGUiOiIyMDE5LTA3LTIyIiwicGFpZFVwVG8iOiIyMDIwLTA3LTIxIn1dLCJoYXNoIjoiMTM3ODQzMjAvMCIsImdyYWNlUGVyaW9kRGF5cyI6NywiYXV0b1Byb2xvbmdhdGVkIjpmYWxzZSwiaXNBdXRvUHJvbG9uZ2F0ZWQiOmZhbHNlfQ==-hBov92HEvNGvBzS2z190KAPxc9F6XY6jT1daMLlPrpCSEAdQX/955WkyGz+hCa3w/aeNExMEZIv2tALkFDOt857w4PZM8oYZ07s7My1NL7DxX9coFswbC6IIBijkAne9cPV9fSnGt5XcfsAkrF8KW1gj21H4EZGR6Jm4Cn7/j37rG1ASu2uvdoJ4dgCicfi78fvIw+zVvGm7L4cMjsmsilink-nodeNrUFPpDuVCp2kfU2ncDWm/M0lu+Dfeo3UO61/ICs9FvYAw0V4d8Q6pExzoqbAGH0IgkrHKJ2YQpKKOz3/+w4SGKhAX+85XYmfmLcUoqAZWaI95yhXN/czf/eeAf3ZEg==-MIIElTCCAn2gAwIBAgIBCTANBgkqhkiG9w0BAQsFADAYMRYwFAYDVQQDDA1KZXRQcm9maWxlIENBMB4XDTE4MTEwMTEyMjk0NloXDTIwMTEwMjEyMjk0NlowaDELMAkGA1UEBhMCQ1oxDjAMBgNVBAgMBU51c2xlMQ8wDQYDVQQHDAZQcmFndWUxGTAXBgNVBAoMEEpldEJyYWlucyBzlink-nodeIuby4xHTAbBgNVBAMMFHByb2QzeS1mcm9tLTIwMTgxMTAxMIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEAxcQkq+zdxlR2mmRYBPzGbUNdMN6OaXiXzxIWtMEkrJMO/5oUfQJbLLuMSMK0QHFmaI37WShyxZcfRCidwXjot4zmNBKnlyHodDij/78TmVqFl8nOeD5+07B8VEaIu7c3E1N+e1doC6wht4I4+IEmtsPAdoaj5WCQVQbrI8KeT8M9VcBIWX7fD0fhexfg3ZRt0xqwMcXGNp3DdJHiO0rCdU+Itv7EmtnSVq9jBG1usMSFvMowR25mju2JcPFp1+I4ZI+FqgR8gyG8oiNDyNEoAbsR3lOpI7grUYSvkB/xVy/VoklPCK2h0f0GJxFjnye8NT1PAywoyl7RmiAVRE/EKwIDAQABo4GZMIGWMAkGA1UdEwQCMAAwHQYDVR0OBBYEFGEpG9oZGcfLMGNBkY7SgHiMGgTcMEgGA1UdIwRBMD+AFKOetkhnQhI2Qb1t4Lm0oFKLl/GzoRykGjAYMRYwFAYDVQQDDA1KZXRQcm9maWxlIENBggkA0myxg7KDeeEwEwYDVR0lBAwwCgYIKwYBBQUHAwEwCwYDVR0PBAQDAgWgMA0GCSqGSIb3DQEBCwUAA4ICAQAF8uc+YJOHHwOFcPzmbjcxNDuGoOUIP+2h1R75Lecswb7ru2LWWSUMtXVKQzChlink-nodePn/72W0k+oI056tgiwuG7M49LXp4zQVlQnFmWU1wwGvVhq5R63Rpjx1zjGUhcXgayu7+9zMUW596Lbomsg8qVve6euqsrFicYkIIuUu4zYPndJwfe0YkS5nY72SHnNdbPhEnN8wcB2Kz+OIG0lih3yz5EqFhld03bGp222ZQCIghCTVL6QBNadGsiN/lWLl4JdR3lJkZzlpFdiHijoVRdWeSWqM4y0t23c92HXKrgppoSV18XMxrWVdoSM3nuMHwxGhFyde05OdDtLpCv+jlWf5REAHHA201pAU6bJSZINyHDUTB+Beo28rRXSwSh3OUIvYwKNVeoBY+KwOJ7WnuTCUq1meE6GkKc4D/cXmgpOyW/1SmBz3XjVIi/zprZ0zf3qH5mkphtg6ksjKgKjmx1cXfZAAX6wcDBNaCL+Ortep1Dh8xDUbqbBVNBL4jbiL3i3xsfNiyJgaZ5sX7i8tmStEpLbPwvHcByuf59qJhV/bZOl8KqJBETCDJcY6O2aqhTUy+9x93ThKs1GKrRPePrWPluud7ttlgtRveit/pcBrnQcXOl1rHq7ByB8CFAxNotRUYL9IF5n3wJOgkPojMy6jetQA5Ogc8Sm7RG6vg1yow==
```
