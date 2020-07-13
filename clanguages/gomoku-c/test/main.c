#include "board-test.h"

// ======= Temporary block for quick prototype.
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

#include "testool.h"

/*
TODO
Design a dynamic indexer.
Design a expanding tree.
Implement Monte-Carlo tree search.
*/

typedef struct _Tree Tree;
typedef struct _Node Node;
typedef struct _NodeIndex NodeIndex;
typedef struct _NodeIndexSection NodeIndexSection;

struct _NodeIndexSection {
    Node *refs[10];
    NodeIndexSection *prevRef;
    NodeIndexSection *nextRef;
};

struct _NodeIndex {
    // This struct is a link list.
    int refCount;
    int sectionCount;
    NodeIndexSection initialSection;
};

struct _Node {
    NodeIndex const childrenIndex;
};

struct _Tree {
    Node *rootRef;
    NodeIndex const leafsIndex;
    int height;
    int nodeCount;
};

/*
TODO
Make the testtools with M_ style macro function have to be ended with semicolon.
Provide a non-pass-print M_test_int_npp macro function.

Design a index provider.
*/
#define M_table_index_count 225
typedef struct _IndexProvider {
    int count;
    int indexes[M_table_index_count];
} IndexProvider;

void
init_index_provider(IndexProvider *const provider) {
    provider->count = M_table_index_count;
    for (int i = 0; i < M_table_index_count; ++i) {
        provider->indexes[i] = i;
    }
}

bool
has_more_index(IndexProvider const *const provider) {
    return provider->count > 0;
}

void
_do_remove_index(IndexProvider *const provider, int const targetIndex) {
    int lastIndex = provider->count - 1;
    provider->indexes[targetIndex] = provider->indexes[lastIndex];
    provider->count = lastIndex;
}

bool
remove_index(IndexProvider *const provider, int const targetIndex) {
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
            _do_remove_index(provider, targetIndex);
        }
        return true;
    }
    for (int i = 0; i < provider->count; ++i) {
        if (targetIndex == provider->indexes[i]) {
            _do_remove_index(provider, i);
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
int
provide_index(IndexProvider *const provider) {
    // Generate random number, and convert to index.
    int targetIndex = rand() % provider->count;
    // Get the index.
    int resultIndex = provider->indexes[targetIndex];
    // Remove the index.
    if (false == remove_index(provider, resultIndex)) {
        printf("[\033[31mError\033[0m] Failed to remove \
targetIndex( %d ) for \
IndexProvider( %p )\n",
               targetIndex,
               provider
        );
    }
    return resultIndex;
}

void
try_random_number(void) {
    printf("Random numner: [%d].\n", rand() % 100);
}

void
try_build_tree(void) {
    Tree tree = {
            .rootRef = malloc(sizeof(Node)),
            .leafsIndex = {
                    .refCount = 0,
                    .sectionCount = 1,
                    .initialSection = {
                            .prevRef = NULL,
                            .nextRef = NULL,
                    },
            },
            .height = 0,
            .nodeCount = 0,
    };
    printf("\
Hello tree! height: %d, nodeCount: %d,\n\
leafsIndex.sectionCount: %d, rootRef: %p,\n\
leafsIndex.initialSection[0]: %p,\n\
leafsIndex.initialSection[9]: %p.\n",
           tree.height, tree.nodeCount, tree.leafsIndex.sectionCount,
           tree.rootRef, tree.leafsIndex.initialSection.refs[0],
           tree.leafsIndex.initialSection.refs[9]
    );
}
// ======= Temporary block end.

int
main(void) {
    srand(time(NULL)); // time(NULL) returns time in seconds.

    test_board_checkers();

    // ======= Temporary block for quick prototype.
    try_random_number();
    try_build_tree();

    IndexProvider *const indexProvider = malloc(sizeof(IndexProvider));
    init_index_provider(indexProvider);
    M_test_int(has_more_index(indexProvider), true)
    M_test_int(remove_index(indexProvider, 224), true)
    M_test_int(remove_index(indexProvider, M_table_index_count), false)
    M_test_int(remove_index(indexProvider, 250), false)
    M_test_int(remove_index(indexProvider, -1), false)
    M_test_int(remove_index(indexProvider, 0), true)
    M_test_int(indexProvider->count, 223)
    M_test_int(provide_index(indexProvider) < 224, true)
    for (int i = 0; i < 222; ++i) {
        M_test_int(provide_index(indexProvider) < 224, true)
        M_test_int(indexProvider->count, 221 - i)
    }
    // ======= Temporary block end.

    return 0;
}
