/* 1059 Chutes and Ladders
Basic rules:
1. Has: player_counter += dice_number.
2. Jump to the end if stop at start.
3. Throw-again grid and stop-one-turn grid. Call them special grid.
4. Overflow ignorance.
5. First reach end win.
Input rules:
1. The first line is less than 1000 dice_numbers, and 0 < dice_number < 7.
2. The follow line is number of players.
3. Chutes and ladders description end with a pair of 0.
4. Special grids description end with a 0. The negative ones are stop-one-turn grid and positive ones are throw-again grid.
5. Then following another game description with the number of players.
6. The game descriptions end with a zero number of players.

Sample input:
3 6 3 2 5 1 3 4 2 3 1 2 0
2
6 95
99 1
0 0
-3
98
0
2
3 99
6 90
0 0
0
0
Sample output:
2
2
*/

#include "batch_read_number.c"
#include <stdbool.h>

#ifdef ONLINE_JUDGE
  #define debug_p //
#else
  #include <time.h>
  #define debug_p printf
#endif

struct Node {
  int height;
  int max_path_length;
  int valid_neighbor_count;
  int neighbors[4];
};

void build_node(struct Node all[], int index, int count, int col) {
  if (index >= count || index < 0) {
    return;
  }
  struct Node* node = &all[index];
  const int height = node->height;
  
  const int tail = col - 1;
  const int in_row = index % col;
  if (0 < in_row) { // left
    const int left = index - 1;
    if (height > all[left].height) {
      node->neighbors[node->valid_neighbor_count++] = left; 
    }
  }
  if (tail > in_row) { // right
    const int right = index + 1;
    if (height > all[right].height) {
      node->neighbors[node->valid_neighbor_count++] = right;
    }
  }
  const int up = index - col;
  if (0 <= up) { // up
    if (height > all[up].height) {
      node->neighbors[node->valid_neighbor_count++] = up;
    }
  }
  const int down = index + col;
  if (count > down) { // down
    if (height > all[down].height) {
      node->neighbors[node->valid_neighbor_count++] = down;
    }
  }
}

int search_node(struct Node all[], int index) {
  struct Node* node = &all[index];
  if (node->max_path_length > 0) {
    return node->max_path_length;
  }
  const int neighbor_count = node->valid_neighbor_count;
  int pathes[neighbor_count];
  for (int i = 0; i < neighbor_count; ++i) {
    pathes[i] = 1 + search_node(all, node->neighbors[i]);
  }
  
  int max = 0;
  for (int i = 0; i < neighbor_count; ++i) {
    const int curr = pathes[i];
    if (curr > max) {
      max = curr;
    }
  }
  node->max_path_length = max;
  return max;
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
  for (int i = 0; i < count; ++i) {
    search_node(nodes, i);
  }
  
  int mpl = 0;
  for (int i = 0; i < count; ++i) {
    const int curr = nodes[i].max_path_length;
    debug_p("%-5d %-5d %-5d\n", i, nodes[i].height, curr);
    if (mpl < curr) {
      mpl = curr;
    }
  }
  
  debug_p("%ld clock used.\n", clock());
  printf("%d\n", mpl + 1);
  return 0;
}
