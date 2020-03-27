#include <iostream>
#include <random>
#include <functional>
#include <chrono>

int main() {
  const int count = 4e8;
  std::cout << "Do " << count << " times multiplication." << '\n';
  std::default_random_engine generator;
  std::uniform_int_distribution<int> distribution(1,6);
  auto dice = std::bind (distribution, generator);
  double result = 0;

  auto timestamp = std::chrono::system_clock::now();
  for (int i = 0; i < count; ++i) {
    result += dice() * dice();
  }
  auto timecost = std::chrono::system_clock::now() - timestamp;
  double castedTimecost = timecost.count();
  std::cout << "Done within " << (castedTimecost / 1e9) << " seconds." << '\n';
  return 0;
}
