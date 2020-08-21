//
// Created by qixiaofeng on 2020/7/13.
//

#include "log.h"
#include "lcg.h"

#include "index-provider.h"

void init_index_provider(IndexProvider *const provider) {
    provider->count = M_table_index_count;
    for (int i = 0; i < M_table_index_count; ++i) {
        provider->indexes[i] = i;
    }
}

bool has_more_index(IndexProvider const *const provider) {
    return provider->count > 0;
}

void p_do_remove_index(IndexProvider *const provider, int const targetIndex) {
    int lastIndex = provider->count - 1;
    provider->indexes[targetIndex] = provider->indexes[lastIndex];
    provider->count = lastIndex;
}

bool remove_index(IndexProvider *const provider, int const targetIndex) {
    if (
            (targetIndex >= M_table_index_count) ||
            (targetIndex < 0)
            ) {
        return false;
    }
    if ((targetIndex == provider->indexes[targetIndex]) && (targetIndex < provider->count)) {
        if (1 == provider->count) {
            provider->count = 0;
        } else {
            p_do_remove_index(provider, targetIndex);
        }
        return true;
    }
    for (int i = 0; i < provider->count; ++i) {
        if (targetIndex == provider->indexes[i]) {
            p_do_remove_index(provider, i);
            return true;
        }
    }
    M_debug_line()
    return false;
}

/**
Have to check with has_more_index(provider) before use provide_index(provider).
May cause undefined behavior without checking.
*/
int provide_index(IndexProvider *const provider) {
    // Generate random number, and convert to index.
    int targetIndex = lcg_get() % provider->count;
    // Get the index.
    int resultIndex = provider->indexes[targetIndex];
    // Remove the index.
    if (false == remove_index(provider, resultIndex)) {
        printf("[\033[31mError\033[0m] Failed to remove "
               "targetIndex( %d ) for "
               "IndexProvider( %p )\n",
               targetIndex,
               provider
        );
    }
    return resultIndex;
}
