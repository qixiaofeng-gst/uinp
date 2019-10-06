/*
Merge sort
*/

#include <stdbool.h>
#include <stdio.h>
#include <time.h>
#include <string.h>

int d2n(int digits[], int size) {
  const int max_zeros = size - 1;
  int number = 0;
  for (int i = 0; i < size; ++i) {
    const int zeros = max_zeros - i;
    int to_add = digits[i];
    for (int j = 0; j < zeros; ++j) {
      to_add *= 10;
    }
    number += to_add;
  }
  return number;
}

void merge_sort(int to_sort[][2], int count) {
  const int mem_size = 2 * sizeof(int) * count;
  int cache[count][2];
  for (int step = 2; step < count; step *= 2) {
    const int half_step = step / 2;
    for (int i = 0; i < count; i += step) {
      const int first_left = i;
      int first_offset = 0;
      const int first_right = i + half_step;
      const int second_left = i + half_step;
      int second_offset = 0;
      const int second_right = i + step;
      for (int j = 0, index = i; index < count && j < step; ++j, ++index) {
        const int first = first_left + first_offset;
        const int second = second_left + second_offset;
        int to_cache = first;
        if (first >= count || first >= first_right) {
          ++second_offset;
          cache[index][0] = to_sort[second][0];
          cache[index][1] = to_sort[second][1];
        } else if (second >= count || second >= second_right) {
          ++first_offset;
          cache[index][0] = to_sort[first][0];
          cache[index][1] = to_sort[first][1];
        } else if (to_sort[first][0] > to_sort[second][0]) {
          ++second_offset;
          cache[index][0] = to_sort[second][0];
          cache[index][1] = to_sort[second][1];
        } else {
          ++first_offset;
          cache[index][0] = to_sort[first][0];
          cache[index][1] = to_sort[first][1];
        }
      }
    }
    memcpy(to_sort, cache, mem_size);
  }
}

int main()
{
  int count;
  scanf("%d", &count);
  
  int all_nums[count][2];
  
  char buffer[count];
  int number[5];
  int readed_count = 0;
  int num_length = 0;
  while (readed_count < count) {
    int length = fread(buffer, 1, count, stdin);
    for (int i = 0; i < length; ++i) {
      if ('\n' == buffer[i]) {
        if (num_length == 0) {
          continue;
        }
        
        all_nums[readed_count][0] = d2n(number, num_length);
        all_nums[readed_count][1] = readed_count;
        readed_count++;
        num_length = 0;
        continue;
      }
      int digit = buffer[i] - 48;
      number[num_length++] = digit;
    }
  }
  
  merge_sort(all_nums, count);
  for (int i = 0; i < 10; ++i) {
    printf("%d %d\n", all_nums[i][0], all_nums[i][1]);
  }
  
  printf("%ld clock used.\n", clock());
  return 0;
}
