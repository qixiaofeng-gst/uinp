/*
Formula without brackets

分值：400

程序执行时限: 600 ms

有一个写在黑板上的加减法算式，运算符只包含加号、减号和括号，但其中的括号被清洁工擦去了。现在需要你写一个算法计算这个算式括号被擦去之前的可能的最小结果值。

输入：
仅有一行，被擦去括号后的算式

输入约束：
算式最多有50个字符，且其中仅包含0-9和+、-
算式的第一个字符一定是数字
算式中不会连续出现两个运算符
算式中每个整数最多有5位

输出：
一个整数：即括号被擦去之前，该算式可能的最小结果值

举例1：
输入：
55-50+40
输出：
-35
解释：
通过增加括号，该算式有两种可能的结果：55-50+40=45和55-(50+40)=-35

举例2：
输入：
10+20+30+40
输出：
100
解释：
由于输入中没有减号，因此无论怎么加括号，结果也只能是100

举例3：
输入：
00009-00009
输出：
0
解释：注意算式中的整数可能有前导0.
*/

#include <stdio.h>

#define plus -1
#define minus -2
#define input_len 51
#define ten 10

int calc(int current_num[5], int num_len) {
  int result = 0;
  for (int i = 0, p = num_len - 1; i < num_len; ++i, --p) {
    int to_add = current_num[i];
    for (int n = 0; n < p; ++n) {
      to_add *= ten;
    }
    result += to_add;
  }
  return result;
}

int main()
{
  char input[input_len];
  scanf("%s", input);
  int formula[input_len];
  int formula_len = 0;

  int current_num[5];
  int num_len = 0;
  for (int i = 0; i < input_len; ++i) {
    const char current = input[i];
    if ('\0' == current) {
      formula[formula_len++] = calc(current_num, num_len);
      break;
    }
    if ('-' == current) {
      formula[formula_len++] = calc(current_num, num_len);
      formula[formula_len++] = minus;
      num_len = 0;
      continue;
    }
    if ('+' == current) {
      formula[formula_len++] = calc(current_num, num_len);
      formula[formula_len++] = plus;
      num_len = 0;
      continue;
    }
    current_num[num_len++] = current - '0';
  }

  int second[formula_len];
  int second_len = 0;
  for (int i = 0; i < formula_len; ++i) {
    const int current = formula[i];
    if (minus == current) {
      continue;
    }
    if (plus == current) {
      second[second_len - 1] += formula[++i];
      continue;
    }
    second[second_len++] = current;
  }

  int result = second[0];
  for (int i = 1; i < second_len; ++i) {
    result -= second[i];
  }
  printf("%d\n", result);
  return 0;
}
