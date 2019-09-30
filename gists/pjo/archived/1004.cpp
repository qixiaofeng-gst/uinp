/*
1004
*/

#include <stdio.h>
#include <math.h>

#define month_count 12
#define handred 100.0f

int main()
{
  float avg = 0.0f;
  for (int i = 0; i < month_count; ++i) {
    float input = 0.0f;
    scanf("%f", &input);
    avg += input;
  }
  avg /= month_count;
  printf("$%.2f\n", (round(avg * handred) / handred));
  return 0;
}
