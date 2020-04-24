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

void quick_sort(int to_sort[][2], int start, int end) {
  if (start >= end) {
    return;
  }
  const int limit = end - start;
  int gts[limit][2];
  int lts[limit][2];
  int gt_count = 0;
  int lt_count = 0;
  
  int pivot_mami[2];
  pivot_mami[0] = to_sort[start][0];
  pivot_mami[1] = to_sort[start][1];
  
  for (int i = start + 1; i <= end; ++i) {
    if (to_sort[start][0] > to_sort[i][0]) {
      lts[lt_count][0] = to_sort[i][0];
      lts[lt_count][1] = to_sort[i][1];
      lt_count++;
    } else {
      gts[gt_count][0] = to_sort[i][0];
      gts[gt_count][1] = to_sort[i][1];
      gt_count++;
    }
  }
  int idx = start;
  for (int i = 0; i < lt_count; ++i) {
    to_sort[idx][0] = lts[i][0];
    to_sort[idx][1] = lts[i][1];
    idx++;
  }
  const int pivot = idx;
  to_sort[idx][0] = pivot_mami[0];
  to_sort[idx][1] = pivot_mami[1];
  idx++;
  for (int i = 0; i < gt_count; ++i) {
    to_sort[idx][0] = gts[i][0];
    to_sort[idx][1] = gts[i][1];
    idx++;
  }
  
  quick_sort(to_sort, start, pivot - 1);
  quick_sort(to_sort, pivot + 1, end);
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
  quick_sort(index, 0, line_count - 1);
  for (int i = 0; i < line_count; ++i) {
    printf("%s\n", all_dna[index[i][1]]);
  }
  return 0;
}
