/*
Random number sequence generator
*/

#include <stdio.h>
#include <stdlib.h>

#define row_count 5
#define col_count 5
const int numbers_count = row_count * col_count;
const int tail = col_count - 1;
const char* fmt = "%-5d";

struct Node {
  int value;
  int next_index;
  int prev_index;
};

int main()
{
  struct Node to_output[numbers_count];
  for (int i = 0; i <= numbers_count; ++i) {
    to_output[i].value = i;
    to_output[i].next_index = i + 1;
    to_output[i].prev_index = i - 1;
  }
  
  int head = 0;
  printf("%d %d\n", row_count, col_count);
  for (int n = 0, rest = numbers_count; n < numbers_count; ++n, --rest) {
    const int idx = rand() % rest;
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
    if (tail == (n % col_count)) {
      printf("\n");
    }
  }
  return 0;
}
