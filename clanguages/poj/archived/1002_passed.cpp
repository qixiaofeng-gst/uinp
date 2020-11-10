/*
1002

Important: cin is very slow, fread is a good choice for massive data
cint cost 70-150s
fread cost about 50s
*/

#include <stdio.h>
#include <string>
#include <cstring>
//#include <time.h>

#define num_length 7
#define separater 10000
#define ten 10

using namespace std;

/*
A, B, and C map to 2
D, E, and F map to 3
G, H, and I map to 4
J, K, and L map to 5
M, N, and O map to 6
P, R, and S map to 7
T, U, and V map to 8
W, X, and Y map to 9
*/
const int map[] = {
  0, // 48, 0
  1, // 49, 1
  2, // 50, 2
  3, // 51, 3
  4, // 52, 4
  5, // 53, 5
  6, // 54, 6
  7, // 55, 7
  8, // 56, 8
  9, // 57, 9
  -1,// 58, -1
  -1,// 59, -1
  -1,// 60, -1
  -1,// 61, -1
  -1,// 62, -1
  -1,// 63, -1
  -1,// 64, -1
  2, // 65, A
  2, // 66, B
  2, // 67, C
  3, // 68, D
  3, // 69, E
  3, // 70, F
  4, // 71, G
  4, // 72, H
  4, // 73, I
  5, // 74, J
  5, // 75, K
  5, // 76, L
  6, // 77, M
  6, // 78, N
  6, // 79, O
  7, // 80, P
  -1,// 81, Q
  7, // 82, R
  7, // 83, S
  8, // 84, T
  8, // 85, U
  8, // 86, V
  9, // 87, W
  9, // 88, X
  9, // 89, Y
};

int get_number(int code) {
  static const int Y =  89;
  static const int zero = 48;
  if (Y < code || zero > code) {
    return -1;
  }
  return map[code - zero];
}

const int counter_size = 10000000;
int counter[counter_size];

int get_code(int seven_nums[]) {
  int code = 0;
  for (int i = 0, p = num_length - 1; i < num_length; ++i, --p) {
    int to_add = seven_nums[i];
    for (int j = 0; j < p; ++j) {
      to_add *= ten;
    }
    code += to_add;
  }
  return code;
}

void print_phone_number(int pn) {
  const int first = pn / separater;
  const int last = pn % separater;
  printf("%03d-%04d %d\n", first, last, counter[pn]);
}

int main()
{
  //time_t start = time(NULL);
  memset(counter, 0, counter_size * sizeof(int));
  int count = 0;
  scanf("%d", &count);
  char temp[count];
  int readed = 0;
  int processed = 0;
  int cache[num_length];
  do {
    readed = fread(temp, 1, count, stdin);
    //printf("%d <<<< \n", readed);
    for (int i = 0; i < readed; ++i) {
      const int current = get_number(temp[i]);
      if (current < 0) {
        continue;
      }
      cache[processed++] = current;
      if (num_length == processed) {
        //printf("%d\n", get_code(cache));
        processed = 0;
        counter[get_code(cache)]++;
      }
    }
  } while (readed == count);
  
  int printed = 0;
  for (int i = 0; i < counter_size; ++i) {
    if (counter[i] > 1) {
      printed++;
      print_phone_number(i);
    }
  }
  if (0 == printed) {
    printf("No duplicates.\n");
  }
  //cout << "Time cost: " << (time(NULL) - start) << "s." << endl;
  return 0;
}
