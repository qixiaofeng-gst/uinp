c 的类型前置语法，迫使声明数组、指针、方法的时候，命名夹在类型当中，使得阅读变得相当困难。
go 的类型后置语法主要是为了解决前述问题。

# 求职面试相关

## 快手的面试题：
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
