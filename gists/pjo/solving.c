/*
Merge sort
*/

#include <stdio.h>
#include <time.h>

int main()
{
  int count;
  scanf("%d", &count);
  char buffer[count];
  int readed_count = 0;
  while (readed_count < count) {
    int length = fread(buffer, 1, count, stdin);
    for (int i = 0; i < length; ++i) {
      readed_count++;
    }
  }
  
  printf("%ld clock used\n", clock());
  return 0;
}
