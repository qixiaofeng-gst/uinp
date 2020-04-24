/* 1002 */

#include <iostream>
#include <string>
#include <cstring>

#define num_length 7
#define dash_pos 3
#define ten 10

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

class PhoneNumber {
  bool is_inited;
  int count;
  int numbers[num_length];
  int code;
  public:
  PhoneNumber() {
    memset(numbers, 0, mem_len);
    is_inited = false;
    count = 0;
    code = 0;
  }
  
  PhoneNumber(string str_num) {
    From(str_num);
  }
  
  void From(string str_num) {
    count = 0;
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
    }
    count++;
    is_inited = true;
  }
  
  void From(PhoneNumber& another) {
    memcpy(numbers, another.numbers, mem_len);
    count = another.count;
    code = another.code;
    is_inited = true;
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
  
  void Reset() {
    count = 0;
  }
  
  bool Equals(PhoneNumber& another) {
    return code == another.code;
  }
  
  bool LessThan(PhoneNumber& another) {
    return count > 1 && code < another.code;
  }
  
  bool IsValid() {
    return count > 1;
  }
};

int GetMin(PhoneNumber pns[], int pn_length) {
  int result = -1;
  for (int i = 0; i < pn_length; ++i) {
    if (result > -1) {
      if (pns[i].LessThan(pns[result])) {
        result = i;
      }
    } else {
      if (pns[i].IsValid()) {
        result = i;
      }
    }
  }
  return result;
}

int main()
{
  int count = 0;
  cin >> count;
  const int length = count;
  PhoneNumber pns[count];
  int n = 0;
  while (count-- > 0) {
    string str_num;
    cin >> str_num;
    PhoneNumber pn = str_num;
    bool is_new = true;
    for (int i = 0; i < n; ++i) {
      if (pns[i].Equals(pn)) {
        is_new = false;
        pns[i].Increase();
        break;
      }
    }
    if (is_new) {
      pns[n++].From(pn); 
    }
  }
  const int pn_length = n;
  int c = 0;
  for (int i = 0; i < pn_length; ++i) {
    if (pns[i].IsValid()) {
      c++;
    }
  }
  const int out_length = c;
  for (int i = 0; i < out_length; ++i) {
    const int index = GetMin(pns, pn_length);
    pns[index].Print();
    pns[index].Reset();
  }
  if (0 == out_length) {
    cout << "No duplicates." << endl;
  }
  return 0;
}
