/*
1005

area of circle = pi * r^2
const float year_lose = 50.0f
area of semicircle a = 1/2 * pi * r^2
n * year_lose = 1/2 * pi * r^2
2n * year_lose / pi = r^2
*/

#include <stdio.h>
#include <math.h>

#define year_limit 100
#define coeffecient 31.830990f

int main()
{
  float r_2s[year_limit];
  float sum = 0.0f;
  for (int i = 0; i < year_limit; ++i, sum += coeffecient) {
    r_2s[i] = sum;
    //printf("%f\n", sum);
  }
  //printf("----- %f\n", 100.f / 3.1415926f);
  int n = 0;
  scanf("%d", &n);
  int m = 0;
  while (m++ < n) {
    float x, y;
    scanf("%f %f", &x, &y);
    float r_2 = x * x + y * y;
    int i = 0;
    for (; i < year_limit; ++i) {
      if (r_2 < r_2s[i]) {
        break;
      }
    }
    printf("Property %d: This property will begin eroding in year %d.\n", m, i);
  }
  printf("END OF OUTPUT.\n");
  return 0;
}
