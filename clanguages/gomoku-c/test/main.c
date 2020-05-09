#include "board-test.h"

// ======= Temporary block for quick prototype.
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

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

void
tryRandomNumber()
{
    srand(time(NULL));
    printf("Random numner: [%d].\n", rand() % 100);
}

void
tryBuildTree()
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
main()
{
    testBoardCheckers();

    // ======= Temporary block for quick prototype.
    tryRandomNumber();
    tryBuildTree();
    // ======= Temporary block end.

    return 0;
}
