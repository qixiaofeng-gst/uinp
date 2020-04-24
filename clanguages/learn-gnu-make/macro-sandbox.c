/**
use `cc -E file-name.c` to check the preprocessed content.
*/

#define MAIN(result, passed) int \
main()\
{\
    char *localString = passed; \
    return result; \
}
#define CONST 0b1000

#define NICE(name, returnType, ...)\
returnType \
func_##name(__VA_ARGS__)\
{\
    printf("%s is invoked.\n", #name); \
    return (returnType) 0; \
}

#define COUNT_PIECE(target) \
if (pieceFlag == target) { \
    ++count; \
} else { \
    return count; \
}

MAIN(CONST, "Hello world")
NICE(dummy, int, int blas, int laplace)

COUNT_PIECE(board[i][j])
