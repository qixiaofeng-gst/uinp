//
// Created by qixiaofeng on 2020/7/30.
//

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include "lcg.h"
#include "board.h"
#include "ai-player.h"
#include "macro-functions.h"

#define m_point_cache_capacity 64
typedef struct p_PointValueCache {
    int size;
    PointValues points[m_point_cache_capacity];
} PointValueCache;

Point const G_ray_directions[] = {
        {1, 0}, // Horizontal
        {0, 1}, // Vertical
        {1, 1}, // Right down
        {1, -1},// Right up
};
size_t const G_directions_size = sizeof(G_ray_directions) / sizeof(Point);

/*!
 * 1. The dot (.) means current point.
 * 2. The star (*) means any point.
 * 3. The character (o) means ally point.
 * 4. The character (x) means non-ally point.
 * 5. The underscore (_) means empty point.
 * 6. The vertical line (|) means enemy or border point.
 * 7. The colon (:) means ally or empty point.
 */
char const *const G_piece_patterns[] = {
        // dead area, 0 score
        "\x02|.***|\xFF", "\x03|*.**|\xFF", "\x03|*.*|\xFF", "\x02|.**|\xFF", "\x02|.*|\xFF", "\x02|.|\xFF",
        // one dead end empty area
        "\x02|.____\x02", "\x03|_.____\x02", "\x04|__.____\x02", "\x05|___.____\x02",
        // dead 1
        "\x03|o.____\x03", "\x04|o_.___\x03", "\x05|o__.__\x03",
        // open empty area
        "\x05____.____\x04",
        // alive 1, constructing alive 2
        "\x03__.o__\x05", "\x02_._o__\x05", "\x02_.__o__\x05",
        // dead 2, constructing half 3
        "\x04|oo.__x\x06", "\x05|oo_._x\x06", "\x06|oo__.x\x06",
        // alive 2, constructing alive 3
        "\x03x_.oo__x\x07", "\x04x__.oo_x\x07", "\x04x_o.o__x\x07", "\x04x_o._o_x\x07",
        // half 3, constructing winning 4
        "\x02x._ooo|\x08", "\x03x_.ooo|\x08", "\x04x_o.oo|\x08", "\x05x_oo.o|\x08", "\x06x_ooo.|\x08",
        // alive 3, constructing winning 4
        "\x03x_.ooo_x\x09", "\x04x_o.oo_x\x09",
        // winning 4, win point
        "\x02x.oooox\x0A", "\x03xo.ooox\x0A", "\x04xoo.oox\x0A",
        // low area, 1 score
        "\x02|.::::|\x01", "\x03|:.:::|\x01", "\x04|::.::|\x01",
};
size_t const G_piece_patterns_size = sizeof(G_piece_patterns) / sizeof(char *);

unsigned const G_value_patterns[][2] = {
        {0x0Au, 0x00u,},
        {0x09u, 0x00u,},
        {0x08u, 0x08u,},
        {0x08u, 0x07u,},
        {0x07u, 0x07u,},
};
size_t const G_value_patterns_size = sizeof(G_value_patterns) / sizeof(G_value_patterns[0]);

bool p_g_enable_print = true;
wchar_t p_g_appearance = m_empty_appearance;
PointValueCache p_g_point_cache;
BoardValues p_g_board_values;

void
p_print_point(Point const *point) {
    if (false == p_g_enable_print) {
        return;
    }
    printf(
            "Print Point(%d, %d). Address: %p.\n",
            point->x, point->y, point
    );
    printf("======= %lu\n", G_value_patterns_size);
}

void
p_clear_point_cache() {
    p_g_point_cache.size = 0;
}

void
p_add_to_point_cache(PointValues const *pointValues) {
    size_t const index = p_g_point_cache.size++;
    p_g_point_cache.points[index].x = pointValues->x;
    p_g_point_cache.points[index].y = pointValues->y;
    for (int i = 0; i < 4; ++i) {
        p_g_point_cache.points[index].values[i] = pointValues->values[i];
    }
}

void
p_pick_from_point_cache(HandDescription *hand) {
    int const index = lcg_get() % p_g_point_cache.size;
    hand->x = (int) p_g_point_cache.points[index].x;
    hand->y = (int) p_g_point_cache.points[index].y;
    p_clear_point_cache();
}

int
p_evaluate_values(PointValues const *pointValues) {
    for (size_t i = 0; i < G_value_patterns_size; ++i) {
        if (G_value_patterns[i][0] == pointValues->values[0]) {
            bool const isSecondZero = (0x00u == G_value_patterns[i][1]);
            if (isSecondZero) {
                return i;
            }
            if (G_value_patterns[i][1] == pointValues->values[1]) {
                return i;
            }
        }
    }
    return -1;
}

bool
p_wild_pv(Board const *board, Point const *point, wchar_t allyAppearance) {
    (void) board, (void) point, (void) allyAppearance;
    return true;
}

bool
p_ally_pv(Board const *board, Point const *point, wchar_t allyAppearance) {
    if (validate_board_point(point)) {
        return allyAppearance == board->grids[point->x][point->y];
    }
    return false;
}

bool
p_non_ally_pv(Board const *board, Point const *point, wchar_t allyAppearance) {
    return false == p_ally_pv(board, point, allyAppearance);
}

bool
p_empty_pv(Board const *board, Point const *point, wchar_t allyAppearance) {
    (void) allyAppearance;
    if (validate_board_point(point)) {
        return m_empty_appearance == board->grids[point->x][point->y];
    }
    return false;
}

bool
p_friend_pv(Board const *board, Point const *point, wchar_t allyAppearance) {
    if (validate_board_point(point)) {
        wchar_t const actualAppearance = board->grids[point->x][point->y];
        return (
                (m_empty_appearance == actualAppearance) ||
                (allyAppearance == actualAppearance)
        );
    }
    return false;
}

bool
p_barrier_pv(Board const *board, Point const *point, wchar_t allyAppearance) {
    return false == p_friend_pv(board, point, allyAppearance);
}

cb_point_validator_t
p_get_point_validator(char pattern) {
    switch (pattern) {
        case '*':
            return p_wild_pv;
        case 'o':
            return p_ally_pv;
        case 'x':
            return p_non_ally_pv;
        case '|':
            return p_barrier_pv;
        case ':':
            return p_friend_pv;
        case '.':
        case '_':
        default:
            return p_empty_pv;
    }
}

bool
p_validate_pattern(char const *pattern) {
    size_t const size = strlen(pattern);
    size_t const dotIndex = pattern[0];
    if (dotIndex >= size) {
        return false;
    }
    if (dotIndex < 2) {
        return false;
    }
    bool const dotIsValid = (pattern[dotIndex] == '.');
    if (false == dotIsValid) {
        return false;
    }
    bool const scoreIsValid = (pattern[size - 1] < 0x0B);
    if (false == scoreIsValid) {
        return false;
    }
    return true;
}

int
p_validate_patterns() {
    for (size_t i = 0; i < G_piece_patterns_size; ++i) {
        if (false == p_validate_pattern(G_piece_patterns[i])) {
            return i;
        }
    }
    return -1; // means valid
}

wchar_t
p_ai_board_get(Board const *board, Point const *point) {
    return board->grids[point->x][point->y];
}

bool
p_match_mono(Board const *board, Ray const *ray, char const *pattern, int scanLimit) {
    Point p = {ray->x, ray->y};
    for (int i = 1; i < scanLimit; ++i) {
        int const c = i - 1;
        cb_point_validator_t pv = p_get_point_validator(pattern[i]);
        p.x = ray->x + c * ray->dx, p.y = ray->y + c * ray->dy;
        if (false == pv(board, &p, ray->appearance)) {
            return false;
        }
    }
    return true;
}

bool
p_match_pattern(Board const *board, Ray const *ray, char const *pattern) {
    int const scanLimit = (int) strlen(pattern) - 1;
    int const offsetIndex = pattern[0] - 1;
    int const dx = ray->dx, dy = ray->dy, dxr = -dx, dyr = -dy;
    int const
            ox = ray->x + dxr * offsetIndex, oy = ray->y + dyr * offsetIndex,
            oxr = ray->x + dx * offsetIndex, oyr = ray->y + dy * offsetIndex;
    char const appearance = ray->appearance;
    Ray pRay = {ox, oy, dx, dy, appearance}, rRay = {oxr, oyr, dxr, dyr, appearance};
    return (p_match_mono(board, &pRay, pattern, scanLimit) || p_match_mono(board, &rRay, pattern, scanLimit));
}

unsigned
p_default_evaluator(Board const *board, Ray const *ray) {
    for (size_t i = 0; i < G_piece_patterns_size; ++i) {
        char const *const pattern = G_piece_patterns[i];
        size_t const length = strlen(pattern);
        if (p_match_pattern(board, ray, pattern)) {
            char const value = pattern[length - 1];
            return ('\xFF' == value) ? 0 : value;
        }
    }
    return 0;
}

int
p_desc_compare_unsigned(void const *a, void const *b) {
    unsigned ua = *((unsigned *) a), ub = *((unsigned *) b);
    if (ua > ub) {
        return -1;
    }
    if (ub > ua) {
        return 1;
    }
    return 0;
}

unsigned
p_sum_point_values(PointValues const *pv) {
    unsigned sum = 0;
    for (int i = 0; i < 4; ++i) {
        sum += pv->values[i];
    }
    return sum;
}

int
p_desc_compare_point_values(void const *a, void const *b) {
    PointValues const *pvA = (PointValues *) a, *pvB = (PointValues *) b;
    int const vA = p_evaluate_values(pvA), vB = p_evaluate_values(pvB);
    bool const invalidA = (-1 == vA), invalidB = (-1 == vB);
    if (invalidA && invalidB) {
        unsigned const sumA = p_sum_point_values(pvA), sumB = p_sum_point_values(pvB);
        return p_desc_compare_unsigned(&sumA, &sumB);
    } else if (invalidA) {
        return 1;
    } else if (invalidB) {
        return -1;
    } else {
        return vA - vB;
    }
}

PointValues
p_evaluate_point(
        Board const *board, Point const *const sourcePoint,
        wchar_t allyAppearance, cb_evaluator_t cbEvaluator
) {
    /*!
     * Empty grid is with initial value 1.
     * Ocuppied grid is with value 0.
     * Accumulate value with counting neighbors.
     *
     * Stop while meet enemy piece or border.
     */
    wchar_t appearance = board->grids[sourcePoint->x][sourcePoint->y];
    bool notEmpty = (false == (m_empty_appearance == appearance));
    PointValues pv = {sourcePoint->x, sourcePoint->y, {0, 0, 0, 0}};
    if (notEmpty) {
        return pv;
    }
    Ray ray = {sourcePoint->x, sourcePoint->y, 0, 0, allyAppearance};
    for (size_t i = 0; i < G_directions_size; ++i) {
        Point const *const d = &(G_ray_directions[i]);
        ray.dx = d->x, ray.dy = d->y;
        pv.values[i] = cbEvaluator(board, &ray);
    }
    return pv;
}

void
p_evaluate_board(Board const *board, BoardValues *values, wchar_t allyAppearance) {
    for (int i = 0; i < m_table_logic_size; ++i) {
        for (int j = 0; j < m_table_logic_size; ++j) {
            int const pIndex = j * m_table_logic_size + i;
            Point point = {i, j};
            PointValues pv = p_evaluate_point(board, &point, allyAppearance, p_default_evaluator);
            qsort(pv.values, 4, sizeof(unsigned), p_desc_compare_unsigned);
            values->points[pIndex] = pv;
        }
    }
    qsort(
            values->points, m_table_logic_size * m_table_logic_size,
            sizeof(PointValues), p_desc_compare_point_values
    );
}

void
p_batch_add_to_point_cache(BoardValues const *boardValues) {
    p_add_to_point_cache(&(boardValues->points[0]));
    for (int i = 1; i < 225; ++i) {
        int const j = i - 1;
        if (0 == p_desc_compare_point_values(
                &(boardValues->points[j]),
                &(boardValues->points[i])
        )) {
            p_add_to_point_cache(&(boardValues->points[i]));
        } else {
            break;
        }
    }
}

void
p_shrink_point_cache() {
    for (int i = 1; i < p_g_point_cache.size; ++i) {
        int const j = i - 1;
        if (0 == p_desc_compare_point_values(
                &(p_g_point_cache.points[j]),
                &(p_g_point_cache.points[i])
        )) {
            continue;
        } else {
            p_g_point_cache.size = i;
            break;
        }
    }
}

wchar_t
ai_get_appearance() {
    return p_g_appearance;
}

void
ai_set_appearance(wchar_t const targetAppearance) {
    p_g_appearance = targetAppearance;
}

void
ai_play_hand(Board *board, HandDescription const *prevHand, HandDescription *currHand) {
    (void) prevHand;

    p_evaluate_board(board, &p_g_board_values, p_g_appearance);
    p_batch_add_to_point_cache(&p_g_board_values);
    p_evaluate_board(
            board, &p_g_board_values,
            p_g_appearance == m_first_appearance ? m_second_appearance : m_first_appearance
    );
    p_batch_add_to_point_cache(&p_g_board_values);
    qsort(&(p_g_point_cache.points), p_g_point_cache.size, sizeof(PointValues), p_desc_compare_point_values);
    p_shrink_point_cache();
    p_pick_from_point_cache(currHand);
    currHand->appearance = p_g_appearance;
    put_piece_at(board, currHand);
}
