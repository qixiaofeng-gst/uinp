/* 1002 */

#include <iostream>
#include <string>
#include <cstring>

#define num_length 7
#define dash_pos 3
#define ten 10
#define input_limit 100000

using namespace std;

const int mem_len = num_length * sizeof(int);
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

struct PhoneNumber {
  PhoneNumber* next;
  int count;
  int numbers[num_length];
  int code;
  
  PhoneNumber(string str_num) {
    code = 0;
    for (int i = 0, j = 0, p = num_length - 1; i < str_num.length(); ++i) {
      const int current = get_number(str_num[i]);
      if (current < 0) {
        continue;
      }
      numbers[j++] = current;
      int to_add = current;
      for (int n = 0; n < p; ++n) {
        to_add *= ten;
      }
      code += to_add;
      --p;
      if (num_length == j) {
        break;
      }
    }
    count = 1;
    next = NULL;
  }
  
  void Print() {
    for (int i = 0; i < num_length; ++i) {
      if (dash_pos == i) {
        cout << "-";
      }
      cout << numbers[i];
    }
    cout << " " << count << endl;
  }
  
  void Increase() {
    count++;
  }
  
  int Sub(PhoneNumber* another) {
    return code - another->code;
  }
  
  bool IsValid() {
    return count > 1;
  }
};

void Insert(PhoneNumber* parent, PhoneNumber* pn) {
  while (false == (NULL == parent)) {
    //cout << parent << " ==== 2" << endl;
    if (NULL == parent->next) {
      parent->next = pn;
      break;
    } else {
      const int trait = pn->Sub(parent->next);
      if (0 > trait) {
        pn->next = parent->next;
        parent->next = pn;
        break;
      } else if (0 == trait) {
        delete pn;
        parent->next->Increase();
        break;
      } else {
        parent = parent->next;
      }
    }
  }
}

int main()
{
  int count = 0;
  cin >> count;
  count = count > input_limit ? input_limit : count;
  const int length = count;
  PhoneNumber* head = NULL;
  while (count-- > 0) {
    string str_num;
    cin >> str_num;
    PhoneNumber* pn = new PhoneNumber(str_num);
    //cout << str_num << " ==== 1" << endl;
    if (NULL == head) {
      head = pn;
    } else {
      const int trait = pn->Sub(head);
      if (0 > trait) {
        pn->next = head;
        head = pn;
      } else if (0 == trait) {
        delete pn;
        head->Increase();
      } else {
        Insert(head, pn);
      }
    }
  }
  
  int printed = 0;
  PhoneNumber* current = head;
  while (false == (current == NULL)) {
    PhoneNumber* to_delete = current;
    if (current->IsValid()) {
      printed++;
      current->Print();
    }
    current = current->next;
    delete to_delete;
  }
  if (0 == printed) {
    cout << "No duplicates." << endl;
  }
  return 0;
}
