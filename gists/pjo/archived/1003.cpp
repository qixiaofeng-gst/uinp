/*
1003

It could be improved by binary search.
*/

#include <stdio.h>
#include <string>
#include <cstring>

#define cache_size 276

int main()
{
  float allsums[cache_size];
  float sum = 0.0f;
  for (int i = 0;; ++i) {
    float dividen = i + 2;
    sum += (1.0f / dividen);
    allsums[i] = sum;
    if (sum > 5.2f) {
      break;
    }
  }
  
  float input = 0.0f;
  do {
    scanf("%f", &input);
    if (0.0f == input) {
      break;
    }
    for (int i = 0; i < cache_size; ++i) {
      if (input < allsums[i]) {
        printf("%d card(s)\n", (i + 1));
        break;
      }
    }
  } while (input > 0);
  return 0;
}
