/*
Random number sequence generator
*/

#include <stdio.h>
#include <stdlib.h>

#define numbers_count 100000

int main()
{
  printf("%d\n", numbers_count);
  for (int n = 0; n < numbers_count; ++n) {
    printf("%d\n", rand() % numbers_count);
  }
  return 0;
}
