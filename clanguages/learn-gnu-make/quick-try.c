#include <stdio.h>

int
add(int a, int b)
{
    return a + b;
}

int (*
getAddFunction(void)
) (int, int)
{
    return add;
}

int
main(int argc, char const *argv[])
{
    int (*fPtr)(int, int) = add;

    int a = 10;
    int *ptr = &a;
    printf("%p, %d, %d, %d\n", ptr, *ptr, fPtr(1, 2), getAddFunction()(3, 4));
    return 0;
}
