/* 1002 */

#include <iostream>
#include <random>

using namespace std;

int main()
{
  default_random_engine e; 
  uniform_int_distribution<unsigned> u(0, 9);
  int output_count = 100000;
  cout << output_count << endl;
  while (output_count-- > 0) {
    for (int i = 0; i < 7; ++i) {
      cout << "-" << u(e) << "-";
    }
    cout << endl;  
  }
  return 0;
}
