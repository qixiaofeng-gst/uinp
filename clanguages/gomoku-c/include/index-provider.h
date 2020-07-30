//
// Created by qixiaofeng on 2020/7/13.
//

#ifndef GOMOKU_C_INDEX_PROVIDER_H
#define GOMOKU_C_INDEX_PROVIDER_H

#include <stdbool.h>

#define M_table_index_count 225
typedef struct p_IndexProvider {
    int count;
    int indexes[M_table_index_count];
} IndexProvider;

void init_index_provider(IndexProvider * provider);

bool has_more_index(IndexProvider const * provider);

int provide_index(IndexProvider * provider);

bool remove_index(IndexProvider * provider, int targetIndex);

#endif //GOMOKU_C_INDEX_PROVIDER_H
