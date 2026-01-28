/**
 * @file    itersolve.h
 * @brief   Iterative solver for stepper timing
 * 
 * Adapted from Klipper klippy/chelper/itersolve.h for MCU use.
 * Provides functions to calculate stepper motor step times from
 * the trapezoidal motion queue.
 * 
 * Key adaptations:
 * - Static memory pools instead of malloc/free
 * - Removed Python FFI markers (__visible)
 * - C99 compatible
 */

#ifndef CHELPER_ITERSOLVE_H
#define CHELPER_ITERSOLVE_H

#include <stdint.h>
#include "trapq.h"

/* Forward declarations */
struct stepper_kinematics;

/**
 * @brief Callback type for calculating stepper position from cartesian coords
 * 
 * @param sk    Stepper kinematics context
 * @param m     Current move
 * @param time  Time offset within move
 * @return Stepper position in steps (as double for interpolation)
 */
typedef double (*sk_calc_callback)(struct stepper_kinematics *sk,
                                   struct move *m, double time);

/**
 * @brief Stepper kinematics structure
 * 
 * Contains the callback and state for converting cartesian motion
 * to stepper positions.
 */
struct stepper_kinematics {
    /* Position calculation callback */
    sk_calc_callback calc_position_cb;
    
    /* Active move tracking */
    struct move *active_move;
    double active_move_start_time;
    
    /* Commanded position tracking */
    double commanded_pos;       /**< Last commanded position (steps) */
    double last_flush_time;     /**< Last time moves were flushed */
    double last_move_time;      /**< End time of last processed move */
    
    /* Step generation state */
    double step_dist;           /**< Distance per step (mm) */
    double step_pos;            /**< Current step position */
    
    /* Associated trapq */
    struct trapq *tq;
    
    /* Kinematics-specific data (e.g., axis index for cartesian) */
    int axis;                   /**< Axis index: 0=X, 1=Y, 2=Z, 3=E */
    double scale;               /**< Scale factor (e.g., steps_per_mm) */
};

/* ========== Memory Pool Configuration ========== */

/**
 * Maximum number of stepper kinematics (typically 4: X, Y, Z, E)
 */
#define ITERSOLVE_MAX_STEPPERS  8

/* ========== Function Prototypes ========== */

/**
 * @brief Initialize the itersolve memory pool
 * 
 * Must be called once at startup.
 */
void itersolve_pool_init(void);

/**
 * @brief Allocate a stepper kinematics structure
 * @return Pointer to stepper_kinematics, or NULL if pool exhausted
 */
struct stepper_kinematics *itersolve_alloc(void);

/**
 * @brief Free a stepper kinematics structure
 * @param sk Stepper kinematics to free
 */
void itersolve_free(struct stepper_kinematics *sk);

/**
 * @brief Set the trapq for a stepper
 * @param sk Stepper kinematics
 * @param tq Trapq to associate
 */
void itersolve_set_trapq(struct stepper_kinematics *sk, struct trapq *tq);

/**
 * @brief Set the position calculation callback
 * @param sk Stepper kinematics
 * @param cb Callback function
 */
void itersolve_set_calc_callback(struct stepper_kinematics *sk,
                                  sk_calc_callback cb);

/**
 * @brief Set the step distance
 * @param sk        Stepper kinematics
 * @param step_dist Distance per step in mm
 */
void itersolve_set_step_dist(struct stepper_kinematics *sk, double step_dist);

/**
 * @brief Set the commanded position
 * @param sk  Stepper kinematics
 * @param pos Position in steps
 */
void itersolve_set_position(struct stepper_kinematics *sk, double pos);

/**
 * @brief Get the commanded position
 * @param sk Stepper kinematics
 * @return Position in steps
 */
double itersolve_get_position(struct stepper_kinematics *sk);

/**
 * @brief Calculate stepper position at a given time
 * @param sk         Stepper kinematics
 * @param print_time Time to query
 * @return Stepper position in steps
 */
double itersolve_calc_position(struct stepper_kinematics *sk, double print_time);

/**
 * @brief Generate step times for moves up to the given time
 * @param sk         Stepper kinematics
 * @param flush_time Time up to which to generate steps
 * @return Number of steps generated
 * 
 * This is the main function that converts motion queue entries
 * into step timing events.
 */
int itersolve_generate_steps(struct stepper_kinematics *sk, double flush_time);

/**
 * @brief Check if stepper is active (has pending moves)
 * @param sk Stepper kinematics
 * @return 1 if active, 0 if idle
 */
int itersolve_is_active(struct stepper_kinematics *sk);

/**
 * @brief Step time entry for step queue
 */
struct step_time {
    double time;                /**< Time of step (seconds) */
    int8_t dir;                 /**< Direction: 1 or -1 */
};

/**
 * @brief Step queue for generated steps
 */
#define STEP_QUEUE_SIZE     256

struct step_queue {
    struct step_time steps[STEP_QUEUE_SIZE];
    int head;                   /**< Read index */
    int tail;                   /**< Write index */
    int count;                  /**< Number of steps in queue */
};

/**
 * @brief Initialize a step queue
 * @param sq Step queue to initialize
 */
void step_queue_init(struct step_queue *sq);

/**
 * @brief Add a step to the queue
 * @param sq   Step queue
 * @param time Step time
 * @param dir  Step direction
 * @return 0 on success, -1 if queue full
 */
int step_queue_push(struct step_queue *sq, double time, int8_t dir);

/**
 * @brief Get next step from queue
 * @param sq   Step queue
 * @param step Output step time entry
 * @return 0 on success, -1 if queue empty
 */
int step_queue_pop(struct step_queue *sq, struct step_time *step);

/**
 * @brief Check if step queue is empty
 * @param sq Step queue
 * @return 1 if empty, 0 if has steps
 */
int step_queue_empty(const struct step_queue *sq);

#endif /* CHELPER_ITERSOLVE_H */
