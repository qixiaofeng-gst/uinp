/*
Power of two

分值：50

程序执行时限: 1000 ms

描述
请使用递归的方式判断一个给定的整数是否为2的整数次幂。
提示：当一个数 n = 2^k （k为非负整数）时，我们说n是2的整数（k）次幂。比如 2、4、8、16都是2的整数次幂，但3、7、14就不是。

输入
一行，一个正整数n

输入约束：
1<=n<=2^31

输出
一行，数字1或0。
如果输入为2的整数次幂，则输出1，否则输出0。
*/

#include <stdio.h>

int check(int num) {
  if (1 == num) {
    return 1;
  }
  int rest = num % 2;
  if (1 == rest) {
    return 0;
  }
  return check(num / 2);
}

int main()
{
  int input;
  scanf("%d", &input);
  printf("%d\n", check(input));
  return 0;
}
