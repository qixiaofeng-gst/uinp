#include <iostream>
#include <string>
#include <cstring>

#define input_length 6 // as the problem given
#define arr_size 250 // 6 * 25, the power will not greater than 25
#define ten 10 // yes, ten

using namespace std;

struct NumberPart {
  int length;
  int decimal_length;
  int mami[arr_size];
  
  NumberPart();
  void CopyFrom(NumberPart& src);
  void Append(int, bool);
  void ShiftToTail();
};

bool digital_gt_zero(NumberPart&);
void digital_mul(NumberPart&, NumberPart&);
void print_digital(NumberPart&);

int main()
{
  const int zero = 48;
  
  string number;
  int p;
  while (cin >> number >> p) {
    NumberPart input_digital;
    NumberPart result_digital;

    bool is_decimal = false;
    for (int i = 0; i < input_length; ++i) {
      int current = number[i] - zero;
      if (0 > current) {
        is_decimal = true;
        continue;
      }

      input_digital.Append(current, is_decimal);
    }
    input_digital.ShiftToTail();
    result_digital.CopyFrom(input_digital);
    
    for (int i = 1; i < p; ++i) {
      digital_mul(result_digital, input_digital);
    }
    result_digital.decimal_length = input_digital.decimal_length * p;
    result_digital.length = input_digital.length * p;
    
    print_digital(result_digital);
  }
  return 0;
}

void digital_add(NumberPart& result, NumberPart& to_add) {
  const int length = to_add.length > result.length ? to_add.length : result.length;
  const int first = arr_size - length;
  const int tail = arr_size - 1;
  int carry = 0;
  for (int i = tail; i >= first; --i) {
    const int added = carry + result.mami[i] + to_add.mami[i];
    result.mami[i] = added % ten;
    carry = added / ten;
  }
  if (carry > 0) {
    result.mami[first - 1] += carry;
    result.length = (length + 1);
  } else {
    result.length = length;
  }
}

void digital_mul(NumberPart& result, NumberPart& base, int multiplier, int shift) {
  NumberPart calculating;
  const int first = arr_size - base.length;
  const int tail = arr_size - 1;
  int carry = 0;
  for (int i = tail; i >= first; --i) {
    const int shifted = i - shift;
    const int multiplied = carry + base.mami[i] * multiplier;
    calculating.mami[shifted] += multiplied % ten;
    carry = multiplied / ten;
  }
  calculating.mami[first - 1 - shift] += carry;
  calculating.length = base.length + shift + (carry > 0 ? 1 : 0);
  digital_add(result, calculating);
}

void digital_mul(NumberPart& result, NumberPart& base) {
  NumberPart calculating;
  calculating.length = result.length + base.length - 1;
  const int first = arr_size - base.length;
  const int tail = arr_size - 1;
  for (int i = tail, j = 0; i >= first; --i, ++j) {
    digital_mul(calculating, result, base.mami[i], j);
  }
  result.CopyFrom(calculating);
}

void print_digital(NumberPart& n) {
  const int first = arr_size - n.length;
  const int dot = arr_size - n.decimal_length;
  bool meet_dot = false;
  bool meet_non_zero = false;
  for (int i = first; i < arr_size; ++i) {
    if (dot == i) {
      meet_dot = true;
      cout << ".";
    }
    const int current = n.mami[i];
    if (current > 0) {
      meet_non_zero = true;
    }
    if (false == meet_dot && false == meet_non_zero) {
      continue;
    }
    cout << current;
  }
  cout << endl;
}

NumberPart::NumberPart() {
  this->length = 0;
  this->decimal_length = 0;
  memset(this->mami, 0, arr_size * sizeof(int));
}

void NumberPart::CopyFrom(NumberPart& src) {
  this->length = src.length;
  this->decimal_length = src.decimal_length;
  memcpy(this->mami, src.mami, arr_size * sizeof(int));
}

void NumberPart::Append(int n, bool d) {
  const int i = this->length++;
  this->mami[i] = n;
  if (d) {
    this->decimal_length++;
  }
}

void NumberPart::ShiftToTail() {
  if (this->decimal_length <= this->length) {
    int count = 0;
    for (int i = this->length - 1, j = 0; j < this->decimal_length; ++j, --i) {
      if (0 == this->mami[i]) {
        ++count;
      } else {
        break;
      }
    }
    this->length -= count;
    this->decimal_length -= count;
  }
  
  const int non_zero_start = 0;
  const int non_zero_end = this->length - 1;
  for (int i = non_zero_end, j = arr_size - 1; i >= non_zero_start; --i, --j) {
    this->mami[j] = this->mami[i];
    this->mami[i] = 0;
  }
}
