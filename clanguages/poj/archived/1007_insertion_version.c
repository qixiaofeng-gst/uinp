/*
1007
Rank  Run ID      User       Memory  Time  Language  Code Length  Submit Time
1     2367318     chenda91   4K      0MS   Pascal    847B         2007-07-21 15:48:11
2     527405(3)   zjpgalois  8K      0MS   Pascal    360B         2005-07-13 16:48:43
3     2390755(5)  tianjie    8K      0MS   C         426B         2007-07-26 04:15:47

Run ID    User         Problem  Result    Memory  Time  Language  Code Length  Submit Time
20917033  qi_xiaofeng  1007     Accepted  340K    32MS  GCC       2283B        2019-10-02 13:28:50
*/

#include <stdio.h>
#include <time.h>

#ifdef ONLINE_JUDGE
  #define debug_p //
#else
  #define debug_p printf
#endif

//#define time_func_M time(NULL)
#define time_func_M clock()

int calc(int exists[], int index) {
  int unsortedness = 0;
  for (int i = 0; i < index; ++i) {
    unsortedness += exists[i];
  }
  if (index < 3) {
    exists[index]++;
  }
  return unsortedness;
}

int get_unsortedness(char input[], int length) {
  int exists[] = { 0, 0, 0 };
  int unsortedness = 0;
  for (int i = length; i >= 0; --i) {
    switch(input[i]){
    case 65: // A
      unsortedness += calc(exists, 0);
    break;
    case 67: // C
      unsortedness += calc(exists, 1);
    break;
    case 71: // G
      unsortedness += calc(exists, 2);
    break;
    case 84: // T
      unsortedness += calc(exists, 3);
    }
  }
  return unsortedness;
}

void swap(int to_swap[][2], int i_a, int i_b) {
  const int tmp_0 = to_swap[i_a][0],
            tmp_1 = to_swap[i_a][1];
  to_swap[i_a][0] = to_swap[i_b][0];
  to_swap[i_a][1] = to_swap[i_b][1];
  to_swap[i_b][0] = tmp_0;
  to_swap[i_b][1] = tmp_1;
}

void insertion_sort(int to_sort[][2], int length) {
  for (int i = 1; i < length; ++i) {
    for (int j = i; j > 0; --j) {
      if (to_sort[j][0] >= to_sort[j - 1][0]) {
        break;
      }
      swap(to_sort, j, j - 1);
    }
  }
}

int main()
{
  int length, line_count;
  scanf("%d %d", &length, &line_count);
  
  char all_dna[line_count][length + 1];
  int index[line_count][2];
  for (int i = 0; i < line_count; ++i) {
    char* input = all_dna[i];
    int* idx = index[i];
    input[length] = '\0';
    scanf("%s", input);
    idx[0] = get_unsortedness(input, length);
    idx[1] = i;
  }
  insertion_sort(index, line_count);
  for (int i = 0; i < line_count; ++i) {
    printf("%s\n", all_dna[index[i][1]]);
    debug_p("%d <<<< the unsortedness\n", index[i][0]);
  }
  debug_p("%ld <<<< %ld \n", time_func_M, CLOCKS_PER_SEC);
  return 0;
}
