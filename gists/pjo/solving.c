/*
1088
*/

#include "batch_read_number.c"
#include <time.h>
#include <stdbool.h>

struct Node {
  int height;
  int max_path_length;
  int valid_neighbor_count;
  int neighbors[4];
};

bool validate_index(int index, int limit) {
  return index >= 0 && index < limit;
}

void build_node(struct Node all[], int index, int count, int col) {
  if (false == validate_index(index, count)) {
    return;
  }
  // left, right, up, down
}

int main()
{
  int row, col;
  scanf("%d %d", &row, &col);
  const int count = row * col;
  
  struct Node nodes[count];
  int nums[count];
  batch_read_number(nums, count, 5);
  for (int i = 0; i < count; ++i) {
    struct Node* n = &nodes[i];
    n->height = nums[i];
    n->max_path_length = 0;
    n->valid_neighbor_count = 0;
  }
  
  for (int i = 0; i < count; ++i) {
    build_node(nodes, i, count, col);
  }
  
  for (int i = 0; i < 10; ++i) {
    printf("%d %d\n", nodes[i].height, nodes[i].valid_neighbor_count);
  }
  
  printf("%ld clock used.\n", clock());
  return 0;
}
