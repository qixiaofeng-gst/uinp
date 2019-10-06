/*
Random number sequence generator
*/

#include <stdio.h>
#include <stdlib.h>

#define row_count 10
#define col_count 10

struct Node {
  int value;
  int next_index;
  int prev_index;
};

int main()
{
  const int numbers_count = row_count * col_count;
  const char* fmt = "%-3d";
  int head = 0;
  int length = numbers_count;
  struct Node to_output[numbers_count];
  for (int i = 0; i <= numbers_count; ++i) {
    to_output[i].value = i;
    to_output[i].next_index = i + 1;
    to_output[i].prev_index = i - 1;
  }
  
  printf("%d %d\n", row_count, col_count);
  for (int n = 0; n < numbers_count; ++n) {
    if (0 == (n % col_count)) {
      printf("\n");
    }
    const int idx = rand() % (length--);
    if (0 == idx) {
      printf(fmt, to_output[head].value);
      head = to_output[head].next_index;
    } else {
      int index = head;
      for (int i = 0; i < idx; ++i) {
        index = to_output[index].next_index;
      }
      const int next = to_output[index].next_index;
      const int prev = to_output[index].prev_index;
      printf(fmt, to_output[index].value);
      to_output[next].prev_index = prev;
      to_output[prev].next_index = next;
    }
  }
  return 0;
}
