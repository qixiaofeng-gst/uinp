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
    Node * refs[10];
    NodeIndexSection * prevRef;
    NodeIndexSection * nextRef;
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
    Node * rootRef;
    NodeIndex const leafsIndex;
    int height;
    int nodeCount;
};

/*
TODO
Design a index provider.
*/
#define M_table_index_count 225
typedef struct _IndexProvider {
    int count;
    int indexes[M_table_index_count];
} IndexProvider;

void
initIndexProvider(IndexProvider * const provider)
{
    provider->count = M_table_index_count;
    for (int i = 0; i < M_table_index_count; ++i) {
        provider->indexes[i] = i;
    }
}

bool
hasMoreIndex(IndexProvider const * const provider)
{
    return provider->count > 0;
}

int
provideIndex(IndexProvider * const provider)
{
    return 0;
}

void
_doRemoveIndex(IndexProvider * const provider, int const targetIndex)
{
    int lastIndex = provider->count - 1;
    provider->indexes[targetIndex] = provider->indexes[lastIndex];
    provider->count = lastIndex;
}

bool
removeIndex(IndexProvider * const provider, int const targetIndex)
{
    if (
        (targetIndex >= M_table_index_count) ||
        (targetIndex < 0)
    ) {
        return false;
    }
    printf("=== targetIndex: %d, provider->indexes[targetIndex]: %d.\n", targetIndex, provider->indexes[targetIndex]);
    if (
        (targetIndex == provider->indexes[targetIndex]) &&
        (targetIndex < provider->count)
    ) {
        if (1 == provider->count) {
            provider->count = 0;
        } else {
            _doRemoveIndex(provider, targetIndex);
        }
        return true;
    }
    for (int i = 0; i < provider->count; ++i) {
        if (targetIndex == provider->indexes[i]) {
            _doRemoveIndex(provider, i);
            return true;
        }
    }
    return false;
}

void
tryRandomNumber(void)
{
    printf("Random numner: [%d].\n", rand() % 100);
}

void
tryBuildTree(void)
{
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
Hello tree! height: %d, nodeCount: %d, \
leafsIndex.sectionCount: %d, rootRef: %p, \
leafsIndex.initialSection[0]: %p, \
leafsIndex.initialSection[9]: %p.\n",
        tree.height, tree.nodeCount, tree.leafsIndex.sectionCount,
        tree.rootRef, tree.leafsIndex.initialSection.refs[0],
        tree.leafsIndex.initialSection.refs[9]
    );
}
// ======= Temporary block end.

int
main(void)
{
    srand(time(NULL)); // time(NULL) returns time in seconds.

    testBoardCheckers();

    // ======= Temporary block for quick prototype.
    tryRandomNumber();
    tryBuildTree();

    IndexProvider * const indexProvider = malloc(sizeof(IndexProvider));
    initIndexProvider(indexProvider);
    M_test_int(hasMoreIndex(indexProvider), true)
    M_test_int(removeIndex(indexProvider, 224), true)
    M_test_int(removeIndex(indexProvider, M_table_index_count), false)
    M_test_int(removeIndex(indexProvider, 250), false)
    M_test_int(removeIndex(indexProvider, -1), false)
    M_test_int(removeIndex(indexProvider, 0), true)
    M_test_int(indexProvider->count, 223)
    M_test_int(provideIndex(indexProvider) < 224, true)
    // ======= Temporary block end.

    return 0;
}
