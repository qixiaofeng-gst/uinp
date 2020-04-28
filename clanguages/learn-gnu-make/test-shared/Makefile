all:
	$(CC) -Wall -Werror -fPIC foo.c -shared -o bin/libfoo.so
	$(CC) -Wall -Werror -L$(PWD)/bin main.c -o bin/main -lfoo
.PHONY: all
