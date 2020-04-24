#include <string.h>
#include <stdbool.h>

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
          to_cache = second;
        } else if (second >= count || second >= second_right) {
          ++first_offset;
        } else if (to_sort[first][0] > to_sort[second][0]) {
          ++second_offset;
          to_cache = second;
        } else {
          ++first_offset;
        }
        
        cache[index][0] = to_sort[to_cache][0];
        cache[index][1] = to_sort[to_cache][1];
      }
    }
    memcpy(to_sort, cache, mem_size);
  }
}