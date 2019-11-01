# Class member arrange rules:
1. Fields first. Static goes first. Public goes first, then protected, private last. Readonly goes first.
2. Constructor follows.
3. Methods last. Abstract goes first. Public goes first, then protected, private last. Static goes last.

# About MVP:
1. `view` use `presenter`, hold an instance of `presenter`
1. `presenter` query and translate data to `model`
1. `presenter` hold instance of `view`
1. `view` use `model`, hold instances of `model`\
Compliment:
1. Only `Fragment` is `view`, `Component`s are component of view.

# About files structure:
1. every module has `contract`, `model`, `presenter`, and `view`
1. every module should only has one `Fragment`, no limit on number of `comparent-only-nodeent`

Advices for team:
1. Use the github issues for task tracing?
2. More documents/guide-text for newbie?

Perhaps update the manual:
1. While running `npm install` first time under `nicelook.com`, failed with `postinstall` executing.
    Perhaps should run `npm install typings` first
2. `tsc --init` not working, any alternative? Or just skip it.
3. Perhaps there should be one more guide for newbie: `grunt` then run the index.html in WebStorm to have fun.

Quick guide for pjo with gcc:
1. `cd /mnt/c/Users/qixia/Documents/uinp/gists/pjo`
2. `gcc solving.c && less input.txt | ./a.out`
3. `gcc creating.c && ./a.out >> input.txt`

**Keep fit: Jeff Cavaliere 5 minutes**

自省三问：
1. 我的观念中哪些是错误的？
2. 别人觉得深不可测的事情，我能了解多少？
3. 大脑中究竟是什么在误导我？

c 的类型前置语法，迫使声明数组、指针、方法的时候，命名夹在类型当中，使得阅读变得相当困难。
go 的类型后置语法主要是为了解决前述问题。

快手的面试题：
1. offsetWidth, scrollWidth, clientWidth 是什么，异同
2. 垂直布局的几种实现方案
3. setTimeout 和 promise 的执行时机
4. js 运算时的类型转换 1 + [], 1 + [1, 2, 3, 4], 1 + '1', 1 * '3'
5. Promise.all 自行实现
6. commonjs 和 es6 的 require 和 import 的区别
7. [7, 6, 4, 1, 2, 3] 按日期排的价格，求最大可能的利润

像素管道 js > style > layout > paint > composite
优化的核心思想是，影响尽量少的环节

1. 算法练习
   - 排序已经差不多够了
2. 树、图复习
   - 已经看过了二叉、B-树
   - 没有练习过，但是聊起来差不多够了
3. React、VUE 源码阅读
   - 没有
4. ()* 的解，* 可用作空字符串、左括号或右括号，左右括号必须成对，判断输入字符串是否有效
   - 解出来了，正向来一次，反向来一次，过了，但是感觉可能有漏洞
5. id + 时间戳的日志，shell 命令输出按出现次数排序的 id
6. 前后端分离
   - 老做法是服务端产出所有内容
   - 现在是服务端只给数据、客户端负责呈现
   - 老做法耦合严重，维护不易
   - 现在新的做法可以前后端并行开发，各自更专注
7. JQuery 或 Vue、React 开发的区别
8. Local Storage、Cookie 和 Session Storage
9. 优化网页渲染性能
   - 聊起来应该差不多了，暂时还没面试时用过
0. 浏览器跨域访问
   - jsonp
0. react 组件生命周期、vue 组件生命周期
0. 自行实现 JS instanceof
0. webpack、gulp 实践

BEM: block, element, modifier.
block, 1. nested structure 2. arbitrary placement 3. re-use
element, a constituent part of a block that can't be used outside of it
modifier, defines the appearance and behavior of a block or an element

db.user.find({ 'mc_member.Mobile': '18621508640' })
db.user.find({ _id: ObjectId('5d706e4ee87c084e92637021') })
                              5d706ec4e87c084e92637025
db.admin_order.find({ pay_id: ObjectId('5d706ec4e87c084e92637025') })
db.admin_gb.find({ _id: ObjectId('5d69fbc6e87c084e92635e3b') })
db.admin_order.find({ group_id: ObjectId('5d69fbc6e87c084e92635e3b') })
more /hd1/forever_logs/gOlQ.log | grep -C 20 '2019-8-31 12:46:5'

redux/mobox

# 开发信息
* 图解开源协议 https://www.cnblogs.com/KruceCoder/p/7991052.html
* 安装 tensorflow 成功的前提是安装 python3.5.2，然后 `pip install --upgrade https://storage.googleapis.com/tensorflow/windows/cpu/tensorflow-1.0.0-cp35-cp35m-win_amd64.whl`\
升级的方法：
```
pip search --version tensorflow
pip install --upgrade tensorflow==1.1.0rc1
```

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
* `git pull` 会下载并合并到本地当前分支，`git fetch` 仅下载
* 撤销上次提交并保留所有修改 `git reset --soft revision-hash`
* 回退某个文件到历史版本 `git co revision_hash -- file/path`
* 查看 head 与前一 commit 的差异 `git diff head~1 head`
* 关闭自动行尾转换 `git config --global core.autocrlf false`

# 常用指令
* 软链接 link-node -s target_to_link link_name
* 在 linux 下，两种使命令脱离 shell 执行的方法：
  1. nohup command_and_paramters &
  1. command_and_paramters </dev/null &>/dev/null &
* 查 linux 已占用端口 sudo lsof -i -P -n | grep LISTEN
* GIT 推送本地工程到远程空工程
```
git pull origin master --allow-unrelated-histories
git remote add origin git@blablabla:url
git push -u origin master
```
* 在 windows 下使命令脱离 cmd 执行的方法（关闭 cmd 后进程将退出，因此大多数情况无效）：
`start "command_name" /B command_and_paramters > somefile.txt`

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
* 分节删除：\r\n-+\r\n\r\n分节阅读\s.+\r\n\r\n

# Web 前端开发跨域 Chrome 设置
"C:/Program Files (x86)/Google/Chrome/Application/chrome.exe" --disable-web-security --user-data-dir="D:/TempDocs/chrome_data/"
Windows 下可创建快捷方式并添加参数

* Windows 重启 LxssManager 服务可重启 Linux 子系统
* ffplay 播放字幕 ffplay path/to/video -vf subtitles=path/to/text
* gpg 检查文件签名 sha256sum/md5sum the-file-to-check 与网站列出的 sha256/md5 签名进行比对
* wget download mirror of website: wget -m -p http://www.xxx.com

* Java 执行指定 jar 包中的指定 class 和 library path 的命令行：
`java -Djava.library.path=dir/path -cp xxx.jar xxx.xxx.ClassName`
* eclipse 中配置 ${user} 变量的方法：在 eclipse.ini 中的 -vmargs 之后一行添加 -Duser.name=XXX

* SQL 拷贝表数据语句：insert into table_1 (column_1, colum_2) select column_a, column_b from table_2;
* 如果 select column_names from table; 的 column_names 中有常量值，比如数值或字符串，则在返回结果中该列是该常量值
* select 语句可以用在 from 和 where 之后
* from 之后可以放多个表（或 select 语句），此时 from 对应的 select 语句中所有列需带表名

* SVN 解决树冲突，需先 svn resolve --accept=working，accept 的参数值随需要而定

WebStorm 激活码 ```
D6KY031L1G-eyJsaWNlbnNlSWQiOiJENktZMDMxTDFHIiwibGljZW5zZWVOYW1lIjoi5o6I5p2D5Luj55CG5ZWGOiB3d3cuaTkub3JnIiwiYXNzaWduZWVOYW1lIjoiIiwiYXNzaWduZWVFbWFpbCI6IiIsImxpY2Vuc2VSZXN0cmljdGlvbiI6IiIsImNoZWNrQ29uY3VycmVudFVzZSI6ZmFsc2UsInByb2R1Y3RzIjpbeyJjb2RlIjoiSUkiLCJmYWxsYmFja0RhdGUiOiIyMDE5LTA3LTIyIiwicGFpZFVwVG8iOiIyMDIwLTA3LTIxIn0seyJjb2RlIjoiQUMiLCJmYWxsYmFja0RhdGUiOiIyMDE5LTA3LTIyIiwicGFpZFVwVG8iOiIyMDIwLTA3LTIxIn0seyJjb2RlIjoiRFBOIiwiZmFsbGJhY2tEYXRlIjoiMjAxOS0wNy0yMiIsInBhaWRVcFRvIjoiMjAyMC0wNy0yMSJ9LHsiY29kZSI6IlBTIiwiZmFsbGJhY2tEYXRlIjoiMjAxOS0wNy0yMiIsInBhaWRVcFRvIjoiMjAyMC0wNy0yMSJ9LHsiY29kZSI6IkdPIiwiZmFsbGJhY2tEYXRlIjoiMjAxOS0wNy0yMiIsInBhaWRVcFRvIjoiMjAyMC0wNy0yMSJ9LHsiY29kZSI6IkRNIiwiZmFsbGJhY2tEYXRlIjoiMjAxOS0wNy0yMiIsInBhaWRVcFRvIjoiMjAyMC0wNy0yMSJ9LHsiY29kZSI6IkNMIiwiZmFsbGJhY2tEYXRlIjoiMjAxOS0wNy0yMiIsInBhaWRVcFRvIjoiMjAyMC0wNy0yMSJ9LHsiY29kZSI6IlJTMCIsImZhbGxiYWNrRGF0ZSI6IjIwMTktMDctMjIiLCJwYWlkVXBUbyI6IjIwMjAtMDctMjEifSx7ImNvZGUiOiJSQyIsImZhbGxiYWNrRGF0ZSI6IjIwMTktMDctMjIiLCJwYWlkVXBUbyI6IjIwMjAtMDctMjEifSx7ImNvZGUiOiJSRCIsImZhbGxiYWNrRGF0ZSI6IjIwMTktMDctMjIiLCJwYWlkVXBUbyI6IjIwMjAtMDctMjEifSx7ImNvZGUiOiJQQyIsImZhbGxiYWNrRGF0ZSI6IjIwMTktMDctMjIiLCJwYWlkVXBUbyI6IjIwMjAtMDctMjEifSx7ImNvZGUiOiJSTSIsImZhbGxiYWNrRGF0ZSI6IjIwMTktMDctMjIiLCJwYWlkVXBUbyI6IjIwMjAtMDctMjEifSx7ImNvZGUiOiJXUyIsImZhbGxiYWNrRGF0ZSI6IjIwMTktMDctMjIiLCJwYWlkVXBUbyI6IjIwMjAtMDctMjEifSx7ImNvZGUiOiJEQiIsImZhbGxiYWNrRGF0ZSI6IjIwMTktMDctMjIiLCJwYWlkVXBUbyI6IjIwMjAtMDctMjEifSx7ImNvZGUiOiJEQyIsImZhbGxiYWNrRGF0ZSI6IjIwMTktMDctMjIiLCJwYWlkVXBUbyI6IjIwMjAtMDctMjEifSx7ImNvZGUiOiJSU1UiLCJmYWxsYmFja0RhdGUiOiIyMDE5LTA3LTIyIiwicGFpZFVwVG8iOiIyMDIwLTA3LTIxIn1dLCJoYXNoIjoiMTM3ODQzMjAvMCIsImdyYWNlUGVyaW9kRGF5cyI6NywiYXV0b1Byb2xvbmdhdGVkIjpmYWxzZSwiaXNBdXRvUHJvbG9uZ2F0ZWQiOmZhbHNlfQ==-hBov92HEvNGvBzS2z190KAPxc9F6XY6jT1daMLlPrpCSEAdQX/955WkyGz+hCa3w/aeNExMEZIv2tALkFDOt857w4PZM8oYZ07s7My1NL7DxX9coFswbC6IIBijkAne9cPV9fSnGt5XcfsAkrF8KW1gj21H4EZGR6Jm4Cn7/j37rG1ASu2uvdoJ4dgCicfi78fvIw+zVvGm7L4cMjsmsilink-nodeNrUFPpDuVCp2kfU2ncDWm/M0lu+Dfeo3UO61/ICs9FvYAw0V4d8Q6pExzoqbAGH0IgkrHKJ2YQpKKOz3/+w4SGKhAX+85XYmfmLcUoqAZWaI95yhXN/czf/eeAf3ZEg==-MIIElTCCAn2gAwIBAgIBCTANBgkqhkiG9w0BAQsFADAYMRYwFAYDVQQDDA1KZXRQcm9maWxlIENBMB4XDTE4MTEwMTEyMjk0NloXDTIwMTEwMjEyMjk0NlowaDELMAkGA1UEBhMCQ1oxDjAMBgNVBAgMBU51c2xlMQ8wDQYDVQQHDAZQcmFndWUxGTAXBgNVBAoMEEpldEJyYWlucyBzlink-nodeIuby4xHTAbBgNVBAMMFHByb2QzeS1mcm9tLTIwMTgxMTAxMIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEAxcQkq+zdxlR2mmRYBPzGbUNdMN6OaXiXzxIWtMEkrJMO/5oUfQJbLLuMSMK0QHFmaI37WShyxZcfRCidwXjot4zmNBKnlyHodDij/78TmVqFl8nOeD5+07B8VEaIu7c3E1N+e1doC6wht4I4+IEmtsPAdoaj5WCQVQbrI8KeT8M9VcBIWX7fD0fhexfg3ZRt0xqwMcXGNp3DdJHiO0rCdU+Itv7EmtnSVq9jBG1usMSFvMowR25mju2JcPFp1+I4ZI+FqgR8gyG8oiNDyNEoAbsR3lOpI7grUYSvkB/xVy/VoklPCK2h0f0GJxFjnye8NT1PAywoyl7RmiAVRE/EKwIDAQABo4GZMIGWMAkGA1UdEwQCMAAwHQYDVR0OBBYEFGEpG9oZGcfLMGNBkY7SgHiMGgTcMEgGA1UdIwRBMD+AFKOetkhnQhI2Qb1t4Lm0oFKLl/GzoRykGjAYMRYwFAYDVQQDDA1KZXRQcm9maWxlIENBggkA0myxg7KDeeEwEwYDVR0lBAwwCgYIKwYBBQUHAwEwCwYDVR0PBAQDAgWgMA0GCSqGSIb3DQEBCwUAA4ICAQAF8uc+YJOHHwOFcPzmbjcxNDuGoOUIP+2h1R75Lecswb7ru2LWWSUMtXVKQzChlink-nodePn/72W0k+oI056tgiwuG7M49LXp4zQVlQnFmWU1wwGvVhq5R63Rpjx1zjGUhcXgayu7+9zMUW596Lbomsg8qVve6euqsrFicYkIIuUu4zYPndJwfe0YkS5nY72SHnNdbPhEnN8wcB2Kz+OIG0lih3yz5EqFhld03bGp222ZQCIghCTVL6QBNadGsiN/lWLl4JdR3lJkZzlpFdiHijoVRdWeSWqM4y0t23c92HXKrgppoSV18XMxrWVdoSM3nuMHwxGhFyde05OdDtLpCv+jlWf5REAHHA201pAU6bJSZINyHDUTB+Beo28rRXSwSh3OUIvYwKNVeoBY+KwOJ7WnuTCUq1meE6GkKc4D/cXmgpOyW/1SmBz3XjVIi/zprZ0zf3qH5mkphtg6ksjKgKjmx1cXfZAAX6wcDBNaCL+Ortep1Dh8xDUbqbBVNBL4jbiL3i3xsfNiyJgaZ5sX7i8tmStEpLbPwvHcByuf59qJhV/bZOl8KqJBETCDJcY6O2aqhTUy+9x93ThKs1GKrRPePrWPluud7ttlgtRveit/pcBrnQcXOl1rHq7ByB8CFAxNotRUYL9IF5n3wJOgkPojMy6jetQA5Ogc8Sm7RG6vg1yow==
```
