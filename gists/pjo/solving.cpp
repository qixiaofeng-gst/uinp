#include <iostream>
#include <string>
#include <cstring>

#define input_length 6 // as the problem given
#define arr_size 250 // 6 * 25, the power will not greater than 25
#define ten 10 // yes, ten

using namespace std;

struct NumberPart {
  int length;
  int mami[arr_size];
  
  NumberPart();
  void CopyFrom(NumberPart& src);
  void Append(int n);
  void ShiftToTail();
};

bool digital_gt_zero(NumberPart&);
bool decimal_gt_zero(NumberPart&);
void digital_mul(NumberPart&, NumberPart&);
void decimal_mul(NumberPart&, NumberPart&);
void print_digital(NumberPart&);
void print_decimal(NumberPart&);

int main()
{
  const int zero = 48;
  
  string number;
  int p;
  while (cin >> number >> p) {
    string output = " <<<<<<< input <<";
    try {
      cout << number << "^" << p << output << endl;
      
      NumberPart input_digital;
      NumberPart input_decimal;
      NumberPart result_digital;
      NumberPart result_decimal;

      bool is_decimal = false;
      for (int i = 0; i < input_length; ++i) {
        int current = number[i] - zero;
        if (0 > current) {
          is_decimal = true;
          continue;
        }
       
        if (is_decimal) {
          input_decimal.Append(current);
        } else {
          input_digital.Append(current);
        }
      }
      input_digital.ShiftToTail();
      result_digital.CopyFrom(input_digital);
      result_decimal.CopyFrom(input_decimal);
      
      for (int i = 1; i < p; ++i) {
        digital_mul(result_digital, input_digital);
        decimal_mul(result_decimal, input_decimal);
      }
      
      if (result_digital.length > 0) {
        print_digital(result_digital);
      }
      if (result_decimal.length > 0) {
        cout << ".";
        print_decimal(result_decimal);
      }
      cout << endl;
    } catch (const char* msg) {
      cout << ">>>> error catched: " << msg << endl;
    }
  }
  return 0;
}

void digital_add(NumberPart& result, NumberPart& to_add) {
  const int first = arr_size - to_add.length;
  const int tail = arr_size - 1;
  int carry = 0;
  for (int i = tail; i >= first; --i) {
    const int added = carry + result.mami[i] + to_add.mami[i];
    result.mami[i] = added % ten;
    carry = added / ten;
  }
  result.mami[first - 1] += carry;
  if (carry > 0 && result.length <= to_add.length) {
    result.length = to_add.length + 1;
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

void decimal_add(NumberPart& result, NumberPart& to_add) {
  const int first = 0;
  const int tail = to_add.length - 1;
  int carry = 0;
  for (int i = tail; i >= first; --i) {
    const int added = carry + result.mami[i] + to_add.mami[i];
    result.mami[i] = added % ten;
    carry = added / ten;
  }
}

void decimal_mul(NumberPart& result, NumberPart& base, int multiplier, int shift) {
  NumberPart calculating;
  calculating.length = base.length + shift;
  const int first = 0;
  const int tail = base.length - 1;
  int carry = 0;
  for (int i = tail; i >= first; --i) {
    const int shifted = i + shift;
    const int multiplied = carry + base.mami[i] * multiplier;
    calculating.mami[shifted] += multiplied % ten;
    carry = multiplied / ten;
  }
  calculating.mami[first + shift - 1] = carry;
  decimal_add(result, calculating);
}

void decimal_mul(NumberPart& result, NumberPart& base) {
  NumberPart calculating;
  calculating.length = result.length + base.length;
  const int first = 0;
  const int tail = base.length - 1;
  for (int i = tail; i >= first; --i) {
    decimal_mul(calculating, result, base.mami[i], i + 1);
  }
  result.CopyFrom(calculating);
}

void print_digital(NumberPart& n) {
  const int first = arr_size - n.length;
  for (int i = first; i < arr_size; ++i) {
    cout << n.mami[i];
  }
}

void print_decimal(NumberPart& n) {
  for (int i = 0; i < n.length; ++i) {
    cout << n.mami[i];
  }
}

NumberPart::NumberPart() {
  this->length = 0;
  memset(this->mami, 0, arr_size * sizeof(int));
}

void NumberPart::CopyFrom(NumberPart& src) {
  this->length = src.length;
  memcpy(this->mami, src.mami, arr_size * sizeof(int));
}

void NumberPart::Append(int n) {
  const int i = this->length++;
  this->mami[i] = n;
}

void NumberPart::ShiftToTail() {
  for (int i = 0, j = arr_size - this->length; i < this->length; ++i, ++j) {
    this->mami[j] = this->mami[i];
    this->mami[i] = 0;
  }
}
