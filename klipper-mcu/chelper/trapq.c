/**
 * @file    trapq.c
 * @brief   Trapezoidal motion queue implementation
 * 
 * Adapted from Klipper klippy/chelper/trapq.c for MCU use.
 * 
 * Key adaptations:
 * - Static memory pools instead of malloc/free
 * - Removed Python FFI markers (__visible)
 * - C99 compatible
 */

#include "trapq.h"
#include <stdint.h>
#include <string.h>

/* ========== Static Memory Pools ========== */

/**
 * Move pool - pre-allocated array of moves
 */
static struct move move_pool[TRAPQ_MAX_MOVES];
static uint8_t move_pool_used[TRAPQ_MAX_MOVES];

/**
 * Trapq pool - typically only need 1-2 trapqs
 */
#define TRAPQ_POOL_SIZE     2
static struct trapq trapq_pool[TRAPQ_POOL_SIZE];
static uint8_t trapq_pool_used[TRAPQ_POOL_SIZE];

/* ========== Memory Pool Functions ========== */

void
trapq_pool_init(void)
{
    memset(move_pool_used, 0, sizeof(move_pool_used));
    memset(trapq_pool_used, 0, sizeof(trapq_pool_used));
}

struct trapq *
trapq_alloc(void)
{
    for (int i = 0; i < TRAPQ_POOL_SIZE; i++) {
        if (!trapq_pool_used[i]) {
            trapq_pool_used[i] = 1;
            struct trapq *tq = &trapq_pool[i];
            list_init(&tq->moves);
            list_init(&tq->history);
            return tq;
        }
    }
    return NULL;  /* Pool exhausted */
}

void
trapq_free(struct trapq *tq)
{
    if (tq == NULL) {
        return;
    }
    
    /* Free all moves in this trapq */
    struct move *m, *tmp;
    list_for_each_entry_safe(m, tmp, &tq->moves, struct move, node) {
        list_del(&m->node);
        move_free(m);
    }
    list_for_each_entry_safe(m, tmp, &tq->history, struct move, node) {
        list_del(&m->node);
        move_free(m);
    }
    
    /* Return trapq to pool */
    int idx = tq - trapq_pool;
    if (idx >= 0 && idx < TRAPQ_POOL_SIZE) {
        trapq_pool_used[idx] = 0;
    }
}

struct move *
move_alloc(void)
{
    for (int i = 0; i < TRAPQ_MAX_MOVES; i++) {
        if (!move_pool_used[i]) {
            move_pool_used[i] = 1;
            struct move *m = &move_pool[i];
            memset(m, 0, sizeof(*m));
            return m;
        }
    }
    return NULL;  /* Pool exhausted */
}

void
move_free(struct move *m)
{
    if (m == NULL) {
        return;
    }
    int idx = m - move_pool;
    if (idx >= 0 && idx < TRAPQ_MAX_MOVES) {
        move_pool_used[idx] = 0;
    }
}

/* ========== Move Calculation Functions ========== */

/**
 * Calculate distance traveled at time t within a move using trapezoidal profile
 */
double
move_get_distance(const struct move *m, double move_time)
{
    if (move_time <= 0.0) {
        return 0.0;
    }
    if (move_time >= m->move_t) {
        move_time = m->move_t;
    }
    
    /* Trapezoidal velocity profile:
     * distance = start_v * t + half_accel * t^2
     * 
     * For a full trapezoidal profile with accel/cruise/decel phases,
     * we need to handle each phase separately.
     */
    double accel_t = m->accel_t;
    double cruise_t = m->cruise_t;
    double decel_t = m->decel_t;
    
    double dist = 0.0;
    double t = move_time;
    
    /* Acceleration phase */
    if (t > 0.0 && accel_t > 0.0) {
        double at = (t < accel_t) ? t : accel_t;
        dist += m->start_v * at + m->half_accel * at * at;
        t -= at;
    }
    
    /* Cruise phase */
    if (t > 0.0 && cruise_t > 0.0) {
        double ct = (t < cruise_t) ? t : cruise_t;
        dist += m->cruise_v * ct;
        t -= ct;
    }
    
    /* Deceleration phase */
    if (t > 0.0 && decel_t > 0.0) {
        double dt = (t < decel_t) ? t : decel_t;
        /* Decel starts at cruise_v, decelerates at -half_accel*2 */
        dist += m->cruise_v * dt - m->half_accel * dt * dt;
    }
    
    return dist;
}

void
move_get_coord(const struct move *m, double move_time, struct coord *pos)
{
    double dist = move_get_distance(m, move_time);
    
    pos->x = m->start_pos.x + m->axes_r.x * dist;
    pos->y = m->start_pos.y + m->axes_r.y * dist;
    pos->z = m->start_pos.z + m->axes_r.z * dist;
    pos->e = m->start_pos.e + m->axes_r.e * dist;
}

/* ========== Trapq Functions ========== */

void
trapq_append(struct trapq *tq, double print_time,
             double accel_t, double cruise_t, double decel_t,
             const struct coord *start_pos, const struct coord *axes_r,
             double start_v, double cruise_v, double accel)
{
    struct move *m = move_alloc();
    if (m == NULL) {
        /* Pool exhausted - should not happen in normal operation */
        return;
    }
    
    m->print_time = print_time;
    m->move_t = accel_t + cruise_t + decel_t;
    m->accel_t = accel_t;
    m->cruise_t = cruise_t;
    m->decel_t = decel_t;
    m->start_v = start_v;
    m->cruise_v = cruise_v;
    m->half_accel = accel * 0.5;
    m->start_pos = *start_pos;
    m->axes_r = *axes_r;
    
    list_add_tail(&m->node, &tq->moves);
}

void
trapq_finalize_moves(struct trapq *tq, double print_time)
{
    struct move *m, *tmp;
    
    list_for_each_entry_safe(m, tmp, &tq->moves, struct move, node) {
        double move_end = m->print_time + m->move_t;
        if (move_end <= print_time) {
            /* Move is complete - move to history */
            list_del(&m->node);
            list_add_tail(&m->node, &tq->history);
        }
    }
}

void
trapq_free_moves(struct trapq *tq, double print_time)
{
    struct move *m, *tmp;
    
    list_for_each_entry_safe(m, tmp, &tq->history, struct move, node) {
        double move_end = m->print_time + m->move_t;
        if (move_end < print_time) {
            list_del(&m->node);
            move_free(m);
        }
    }
}

int
trapq_get_position(const struct trapq *tq, double print_time, struct coord *pos)
{
    /* Search in active moves first */
    struct move *m;
    list_for_each_entry(m, &tq->moves, struct move, node) {
        double move_end = m->print_time + m->move_t;
        if (print_time >= m->print_time && print_time <= move_end) {
            double move_time = print_time - m->print_time;
            move_get_coord(m, move_time, pos);
            return 0;
        }
    }
    
    /* Search in history */
    list_for_each_entry(m, &tq->history, struct move, node) {
        double move_end = m->print_time + m->move_t;
        if (print_time >= m->print_time && print_time <= move_end) {
            double move_time = print_time - m->print_time;
            move_get_coord(m, move_time, pos);
            return 0;
        }
    }
    
    return -1;  /* No move at this time */
}

int
trapq_has_moves(const struct trapq *tq)
{
    return !list_empty(&tq->moves);
}

struct move *
trapq_first_move(struct trapq *tq)
{
    return list_first_entry(&tq->moves, struct move, node);
}

struct move *
trapq_last_move(struct trapq *tq)
{
    return list_last_entry(&tq->moves, struct move, node);
}
