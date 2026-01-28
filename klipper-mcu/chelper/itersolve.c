/**
 * @file    itersolve.c
 * @brief   Iterative solver for stepper timing implementation
 * 
 * Adapted from Klipper klippy/chelper/itersolve.c for MCU use.
 * 
 * Key adaptations:
 * - Static memory pools instead of malloc/free
 * - Removed Python FFI markers (__visible)
 * - C99 compatible
 */

#include "itersolve.h"
#include <stdint.h>
#include <string.h>
#include <math.h>

/* ========== Static Memory Pools ========== */

static struct stepper_kinematics sk_pool[ITERSOLVE_MAX_STEPPERS];
static uint8_t sk_pool_used[ITERSOLVE_MAX_STEPPERS];

/* ========== Memory Pool Functions ========== */

void
itersolve_pool_init(void)
{
    memset(sk_pool_used, 0, sizeof(sk_pool_used));
    memset(sk_pool, 0, sizeof(sk_pool));
}

struct stepper_kinematics *
itersolve_alloc(void)
{
    for (int i = 0; i < ITERSOLVE_MAX_STEPPERS; i++) {
        if (!sk_pool_used[i]) {
            sk_pool_used[i] = 1;
            struct stepper_kinematics *sk = &sk_pool[i];
            memset(sk, 0, sizeof(*sk));
            sk->step_dist = 1.0;  /* Default: 1mm per step */
            return sk;
        }
    }
    return NULL;  /* Pool exhausted */
}

void
itersolve_free(struct stepper_kinematics *sk)
{
    if (sk == NULL) {
        return;
    }
    int idx = sk - sk_pool;
    if (idx >= 0 && idx < ITERSOLVE_MAX_STEPPERS) {
        sk_pool_used[idx] = 0;
    }
}

/* ========== Configuration Functions ========== */

void
itersolve_set_trapq(struct stepper_kinematics *sk, struct trapq *tq)
{
    sk->tq = tq;
}

void
itersolve_set_calc_callback(struct stepper_kinematics *sk, sk_calc_callback cb)
{
    sk->calc_position_cb = cb;
}

void
itersolve_set_step_dist(struct stepper_kinematics *sk, double step_dist)
{
    sk->step_dist = step_dist;
}

void
itersolve_set_position(struct stepper_kinematics *sk, double pos)
{
    sk->commanded_pos = pos;
    sk->step_pos = pos;
}

double
itersolve_get_position(struct stepper_kinematics *sk)
{
    return sk->commanded_pos;
}

/* ========== Position Calculation ========== */

/**
 * Find the move containing the given time
 */
static struct move *
find_move_at_time(struct trapq *tq, double print_time)
{
    if (tq == NULL) {
        return NULL;
    }
    
    struct move *m;
    list_for_each_entry(m, &tq->moves, struct move, node) {
        double move_end = m->print_time + m->move_t;
        if (print_time >= m->print_time && print_time <= move_end) {
            return m;
        }
    }
    
    return NULL;
}

double
itersolve_calc_position(struct stepper_kinematics *sk, double print_time)
{
    if (sk->calc_position_cb == NULL || sk->tq == NULL) {
        return sk->commanded_pos;
    }
    
    struct move *m = find_move_at_time(sk->tq, print_time);
    if (m == NULL) {
        return sk->commanded_pos;
    }
    
    double move_time = print_time - m->print_time;
    return sk->calc_position_cb(sk, m, move_time);
}

/* ========== Step Generation ========== */

/**
 * Iteratively solve for the time when stepper reaches target position
 * 
 * Uses Newton-Raphson iteration to find the time when the stepper
 * position equals the target step position.
 */
static double
itersolve_find_step_time(struct stepper_kinematics *sk, struct move *m,
                         double target_pos, double low_time, double high_time)
{
    /* Newton-Raphson iteration parameters */
    const int max_iterations = 50;
    const double tolerance = 1e-9;
    
    double time = (low_time + high_time) * 0.5;
    
    for (int i = 0; i < max_iterations; i++) {
        double pos = sk->calc_position_cb(sk, m, time);
        double error = pos - target_pos;
        
        if (fabs(error) < tolerance) {
            return time;
        }
        
        /* Estimate derivative using finite difference */
        double dt = 1e-6;
        double pos_dt = sk->calc_position_cb(sk, m, time + dt);
        double derivative = (pos_dt - pos) / dt;
        
        if (fabs(derivative) < 1e-12) {
            /* Derivative too small, use bisection */
            if (error > 0) {
                high_time = time;
            } else {
                low_time = time;
            }
            time = (low_time + high_time) * 0.5;
        } else {
            /* Newton-Raphson step */
            double new_time = time - error / derivative;
            
            /* Clamp to bounds */
            if (new_time < low_time) {
                new_time = (low_time + time) * 0.5;
            } else if (new_time > high_time) {
                new_time = (time + high_time) * 0.5;
            }
            
            time = new_time;
        }
    }
    
    return time;
}

int
itersolve_generate_steps(struct stepper_kinematics *sk, double flush_time)
{
    if (sk->tq == NULL || sk->calc_position_cb == NULL) {
        return 0;
    }
    
    int steps_generated = 0;
    double current_time = sk->last_flush_time;
    
    struct move *m;
    list_for_each_entry(m, &sk->tq->moves, struct move, node) {
        double move_start = m->print_time;
        double move_end = m->print_time + m->move_t;
        
        /* Skip moves we've already processed */
        if (move_end <= current_time) {
            continue;
        }
        
        /* Stop if move is beyond flush time */
        if (move_start >= flush_time) {
            break;
        }
        
        /* Calculate start and end positions for this move */
        double start_time = (current_time > move_start) ? 
                            (current_time - move_start) : 0.0;
        double end_time = (flush_time < move_end) ? 
                          (flush_time - move_start) : m->move_t;
        
        double start_pos = sk->calc_position_cb(sk, m, start_time);
        double end_pos = sk->calc_position_cb(sk, m, end_time);
        
        /* Determine step direction */
        int8_t dir = (end_pos > start_pos) ? 1 : -1;
        
        /* Calculate target step positions */
        double step_pos = sk->step_pos;
        double target_step = (dir > 0) ? 
                             floor(step_pos) + 1.0 : 
                             ceil(step_pos) - 1.0;
        
        /* Generate steps within this move */
        while (1) {
            /* Check if target is within move range */
            if (dir > 0 && target_step > end_pos) {
                break;
            }
            if (dir < 0 && target_step < end_pos) {
                break;
            }
            
            /* Find time when we reach target position */
            double step_time = itersolve_find_step_time(sk, m, target_step,
                                                        start_time, end_time);
            
            /* Record step (in real implementation, this would queue to stepper) */
            double abs_time = m->print_time + step_time;
            (void)abs_time;  /* TODO: Queue step to stepper driver */
            
            steps_generated++;
            sk->step_pos = target_step;
            
            /* Move to next step */
            target_step += dir;
            start_time = step_time;
        }
        
        current_time = move_end;
    }
    
    sk->last_flush_time = flush_time;
    sk->commanded_pos = sk->step_pos;
    
    return steps_generated;
}

int
itersolve_is_active(struct stepper_kinematics *sk)
{
    if (sk->tq == NULL) {
        return 0;
    }
    return trapq_has_moves(sk->tq);
}

/* ========== Step Queue Functions ========== */

void
step_queue_init(struct step_queue *sq)
{
    sq->head = 0;
    sq->tail = 0;
    sq->count = 0;
}

int
step_queue_push(struct step_queue *sq, double time, int8_t dir)
{
    if (sq->count >= STEP_QUEUE_SIZE) {
        return -1;  /* Queue full */
    }
    
    sq->steps[sq->tail].time = time;
    sq->steps[sq->tail].dir = dir;
    sq->tail = (sq->tail + 1) % STEP_QUEUE_SIZE;
    sq->count++;
    
    return 0;
}

int
step_queue_pop(struct step_queue *sq, struct step_time *step)
{
    if (sq->count == 0) {
        return -1;  /* Queue empty */
    }
    
    *step = sq->steps[sq->head];
    sq->head = (sq->head + 1) % STEP_QUEUE_SIZE;
    sq->count--;
    
    return 0;
}

int
step_queue_empty(const struct step_queue *sq)
{
    return sq->count == 0;
}
