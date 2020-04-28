all:
	$(CC) -Wall -Werror -fPIC foo.c -shared -o bin/libfoo.so
	$(CC) -Wall -Werror -L$(PWD)/bin -Wl,-rpath=$(PWD)/bin -o bin/main main.c -lfoo
.PHONY: all
