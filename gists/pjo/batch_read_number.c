#include <stdio.h>

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

void batch_read_number(int nums[], int count, int width) {
  const static int zero = '0';
  char buffer[count];
  int number[width];
  int readed_count = 0;
  int num_length = 0;
  
  while (readed_count < count) {
    int length = fread(buffer, 1, count, stdin);
    for (int i = 0; i < length; ++i) {
      if (zero > buffer[i] || '9' < buffer[i]) {
        if (num_length == 0) {
          continue;
        }
        
        nums[readed_count++] = d2n(number, num_length);
        num_length = 0;
        continue;
      }
      int digit = buffer[i] - zero;
      number[num_length++] = digit;
    }
  }
}
