/*
Number of operations

分值：200

程序执行时限: 600 ms

有一种奇怪的计算机，它的内存是由若干位组成的（初始都是0），并且它只能进行如下操作：
选择某一位和一个值（0或1），从选中的位开始到最后一位的值都会被改成选定的值。比如，当前内存为“0010”，若选择第二位和值1，那么内存将会被改成“0111”。
现在要你将内存从初始状态改成某个特定的序列A，最少需要多少次操作呢？

输入
一行，一个由“0”和“1”组成的字符串，表示特定序列A

输入约束
序列A中仅包含字符0或1，且字符个数（内存大小）范围是 [1,50]

输出
一行，一个整数，表示为了得到这个序列最少需要的操作次数

例子
输入
0011
输出
1
*/

#include <stdio.h>

#define input_len 51
const char zero = '0';
const char one = '1';

int main()
{
  char input[input_len];
  scanf("%s", input);
  int count = 0;
  char last = zero;
  for (int i = 0; i < input_len; ++i) {
    const char current = input[i];
    if ('\0' == current) {
      break;
    }
    if (current == last) {
      continue;
    }
    ++count;
    last = current;
  }
  printf("%d\n", count);
  return 0;
}
