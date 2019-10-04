/*
1007

Input

The first line contains two integers: a positive integer n (0 < n <= 50) giving the length of the strings; and a positive integer m (0 < m <= 100) giving the number of strings. These are followed by m lines, each containing a string of length n.

Output

Output the list of input strings, arranged from ``most sorted'' to ``least sorted''. Since two strings can be equally sorted, then output them according to the orginal order.
*/

#include <stdio.h>
#include <stdlib.h>

#define dna_length 50

int main()
{
  char chars[] = { 'A', 'C', 'G', 'T' };
  char to_output[dna_length + 1];
  to_output[dna_length] = '\0';
  for (int n = 0; n < 100; ++n) {
    for (int i = 0; i < dna_length; ++i) {
      to_output[i] = chars[rand() % 4];
    }
    printf("%s\n", to_output);
  }
  return 0;
}
