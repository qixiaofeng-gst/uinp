#include <iostream>
#include <string>

using namespace std;

int main()
{
  const int zero = 49;
  string number;
  int p;
  while (cin >> number >> p) {
    string output = "====" + to_string(1);
    try {
      cout << number << output << p << number[0] << endl;
      int a = number[0] - zero;
      cout << a << endl;
    } catch (const char* msg) {
      cout << ">>>> error catched: " << msg << endl;
    }
  }
  return 0;
}