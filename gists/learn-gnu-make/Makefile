args_includes = -Iinclude
args_c_standard = -std=c99

path_src = src
path_lib = lib
path_bin = bin
main_name = main.c
app_name = test

sources := $(filter-out $(path_src)/$(main_name), $(wildcard $(path_src)/*.c))
objects := $(sources:.c=.o)
objects := $(subst $(path_src)/, $(path_lib)/, $(objects))

src_main = $(path_src)/$(main_name)
app_path = $(path_bin)/$(app_name)

all: $(objects)
	@$(CC) $(args_c_standard) $(args_includes) \
	 	$(objects) $(src_main) -o $(app_path)
	@echo '====>>> Make Done.'
.PHONY: all

$(objects): $(path_lib)/%.o: $(path_src)/%.c
	$(CC) $(args_c_standard) $(args_includes) -c $< -o $@

cpp:
	$(CXX) -std=c++11 sandbox.cpp -o test
.PHONY: cpp
