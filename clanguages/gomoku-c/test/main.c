#include "board-test.h"
#include "ai-test.h"

// ======= Temporary block for quick prototype.
#include <stdlib.h>
#include <stdio.h>

#include "testool.h"
#include "lcg.h"
#include "index-provider.h"

/*TODO -
 * Design a dynamic indexer.
 * Design a expanding tree.
 * Implement Monte-Carlo tree search.
 */

typedef struct p_Tree Tree;
typedef struct p_Node Node;
typedef struct p_NodeIndex NodeIndex;
typedef struct p_NodeIndexSection NodeIndexSection;

struct p_NodeIndexSection {
    Node *refs[10];
    NodeIndexSection *prevRef;
    NodeIndexSection *nextRef;
};

struct p_NodeIndex {
    // This struct is a link list.
    int refCount;
    int sectionCount;
    NodeIndexSection initialSection;
};

struct p_Node {
    NodeIndex const childrenIndex;
};

struct p_Tree {
    Node *rootRef;
    NodeIndex const leafsIndex;
    int height;
    int nodeCount;
};

void
try_random_number(void) {
    printf("Random numner: [%d].\n", lcg_get() % 100);
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
    printf("Hello tree! height: %d, nodeCount: %d,\n"
           "leafsIndex.sectionCount: %d, rootRef: %p,\n"
           "leafsIndex.initialSection[0]: %p,\n"
           "leafsIndex.initialSection[9]: %p,\n"
           "rootRef->childrenIndex.sectionCount: %d.\n",
           tree.height, tree.nodeCount, tree.leafsIndex.sectionCount,
           tree.rootRef, tree.leafsIndex.initialSection.refs[0],
           tree.leafsIndex.initialSection.refs[9],
           tree.rootRef->childrenIndex.sectionCount
    );
}
// ======= Temporary block end.

void p_test_index_provider() {
    IndexProvider indexProvider;
    init_index_provider(&indexProvider);
    M_test_int(has_more_index(&indexProvider), true)
    M_test_int(remove_index(&indexProvider, 224), true)
    M_test_int(remove_index(&indexProvider, 225), false)
    M_test_int(remove_index(&indexProvider, 250), false)
    M_test_int(remove_index(&indexProvider, -1), false)
    M_test_int(remove_index(&indexProvider, 0), true)
    M_test_int(indexProvider.count, 223)
    M_test_int(provide_index(&indexProvider) < 224, true)
    for (int i = 0; i < 222; ++i) {
        M_test_int(provide_index(&indexProvider) < 224, true)
        M_test_int(indexProvider.count, 221 - i)
    }
}

int
main(void) {
    // ======= Temporary block for quick prototype.
    try_random_number();
    try_build_tree();
    // ======= Temporary block end.

    M_run_test_suite(p_test_index_provider)
    test_board_checkers();
    test_ai();
    return 0;
}
