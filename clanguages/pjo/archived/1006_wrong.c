/*
1006
*/

#include <stdbool.h>
#include <stdio.h>

#define up_limit 21252
#define cease_mark -1
#define cycle_p 23
#define cycle_e 28
#define cycle_i 33

bool should_cease(int p, int e, int i, int d) {
  return cease_mark == p && cease_mark == e && cease_mark == i && cease_mark == d;
}

int calc_triple_peak(int p, int e, int i, int d) {
  const int limit = up_limit + d;
  for (int n = 0; n <= limit; ++n) {
    if (n < d) {
      continue;
    }
    const int current = 1 + n;
    if (((current - p) % cycle_p) > 0) {
      continue;
    }
    if (((current - e) % cycle_e) > 0) {
      continue;
    }
    if (((current - i) % cycle_i) > 0) {
      continue;
    }
    return current - d;
  }
  return up_limit - d;
}

int main()
{
  int sn = 0;
  int p, e, i, d;
  scanf("%d %d %d %d", &p, &e, &i, &d);
  while (false == should_cease(p, e, i, d)) {
    int triple_peak = calc_triple_peak(p % cycle_p, e % cycle_e, i % cycle_i, d);
    printf("Case %d: the next triple peak occurs in %d days.\n", ++sn, triple_peak);
    scanf("%d %d %d %d", &p, &e, &i, &d);
  }
  return 0;
}
