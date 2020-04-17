args_includes = -Iinclude
args_c_standard = -std=c99

path_src = src
path_lib = lib
path_bin = bin

objects = table-utilities.o terminal-utilities.o

obj_table_utilities = $(path_lib)/table-utilities.o
obj_termi_utilities = $(path_lib)/terminal-utilities.o

src_table_utilities = $(path_src)/table-utilities.c
src_termi_utilities = $(path_src)/terminal-utilities.c
src_main = $(path_src)/main.c

app_path = $(path_bin)/test

all: $(obj_table_utilities) $(obj_termi_utilities)
	$(CC) $(args_c_standard) $(args_includes) $(obj_table_utilities) $(obj_termi_utilities) $(src_main) -o $(app_path)
	@echo '======= Make Done ======='
.PHONY: all

$(obj_table_utilities): $(src_table_utilities)
	$(CC) $(args_c_standard) $(args_includes) \
	 	-c $(src_table_utilities) \
		-o $(obj_table_utilities)

$(obj_termi_utilities): $(src_termi_utilities)
	$(CC) $(args_c_standard) $(args_includes) \
	 	-c $(src_termi_utilities) \
		-o $(obj_termi_utilities)

cpp:
	$(CXX) -std=c++11 sandbox.cpp -o test
.PHONY: cpp
