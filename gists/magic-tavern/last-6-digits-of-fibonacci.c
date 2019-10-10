/*
Last 6 digits of Fibonacci

分值：250

程序执行时限: 600 ms

假设n为正整数，斐波那契数列定义为：
f(n) = 1, n < 3;
f(n) = f(n-1) + f(n-2), n>=3

现在请你来计算f(n)的值，但是不需要给出精确值，只要结果的后六位即可。

输入：一行，包含一个正整数n，且0<n<1000
输出：一行，f(n)的后6位（十进制，不足6位不补零）
*/

#include <stdio.h>

#define limit 1000000

int fibonacci(int n, int fn1, int fn2) {
  if (2 > n) {
    return fn1;
  }
  return fibonacci(n - 1, fn2 % limit, (fn1 + fn2) % limit);
}

int main()
{
  int input;
  scanf("%d", &input);
  printf("%d\n", fibonacci(input, 1, 1) % limit);
  return 0;
}
