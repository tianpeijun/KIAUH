/**
 * @file    trapq.h
 * @brief   Trapezoidal motion queue interface
 * 
 * Adapted from Klipper klippy/chelper/trapq.h for MCU use.
 * Provides data structures and functions for trapezoidal motion planning.
 * 
 * Key adaptations for MCU:
 * - No malloc/free - uses static memory pools
 * - Removed __visible macro (Python FFI)
 * - C99 compatible
 */

#ifndef CHELPER_TRAPQ_H
#define CHELPER_TRAPQ_H

#include "list.h"

/**
 * @brief 3D coordinate with extruder position
 */
struct coord {
    double x;
    double y;
    double z;
    double e;
};

/**
 * @brief Motion segment in the trapezoidal queue
 * 
 * Represents a single motion segment with trapezoidal velocity profile.
 * Each move has a start position, direction, and velocity/acceleration params.
 */
struct move {
    /* Timing */
    double print_time;          /**< Start time of this move (seconds) */
    double move_t;              /**< Duration of this move (seconds) */
    
    /* Velocity profile */
    double start_v;             /**< Starting velocity (mm/s) */
    double half_accel;          /**< Half of acceleration (mm/s²) / 2 */
    double cruise_v;            /**< Cruise velocity (mm/s) */
    
    /* Acceleration phase timing */
    double accel_t;             /**< Acceleration phase duration */
    double cruise_t;            /**< Cruise phase duration */
    double decel_t;             /**< Deceleration phase duration */
    
    /* Position and direction */
    struct coord start_pos;     /**< Starting position */
    struct coord axes_r;        /**< Unit direction vector (normalized) */
    
    /* List linkage */
    struct list_node node;      /**< Node for linking in trapq */
};

/**
 * @brief Trapezoidal motion queue
 * 
 * Contains active moves and historical moves for lookahead.
 */
struct trapq {
    struct list_head moves;     /**< Active moves to execute */
    struct list_head history;   /**< Completed moves (for lookahead) */
};

/* ========== Memory Pool Configuration ========== */

/**
 * Maximum number of moves in the queue.
 * Each move is ~256 bytes, so 32 moves = ~8KB
 */
#define TRAPQ_MAX_MOVES     32

/* ========== Function Prototypes ========== */

/**
 * @brief Initialize the trapq memory pool
 * 
 * Must be called once at startup before using any trapq functions.
 */
void trapq_pool_init(void);

/**
 * @brief Create and initialize a new trapq
 * @return Pointer to initialized trapq, or NULL if pool exhausted
 */
struct trapq *trapq_alloc(void);

/**
 * @brief Free a trapq back to the pool
 * @param tq Trapq to free
 */
void trapq_free(struct trapq *tq);

/**
 * @brief Allocate a move from the pool
 * @return Pointer to move, or NULL if pool exhausted
 */
struct move *move_alloc(void);

/**
 * @brief Free a move back to the pool
 * @param m Move to free
 */
void move_free(struct move *m);

/**
 * @brief Add a move to the trapq
 * @param tq        Target trapq
 * @param print_time Start time
 * @param accel_t   Acceleration phase duration
 * @param cruise_t  Cruise phase duration
 * @param decel_t   Deceleration phase duration
 * @param start_pos Starting position
 * @param axes_r    Direction vector (normalized)
 * @param start_v   Starting velocity
 * @param cruise_v  Cruise velocity
 * @param accel     Acceleration value
 */
void trapq_append(struct trapq *tq, double print_time,
                  double accel_t, double cruise_t, double decel_t,
                  const struct coord *start_pos, const struct coord *axes_r,
                  double start_v, double cruise_v, double accel);

/**
 * @brief Finalize moves up to the given time
 * @param tq         Target trapq
 * @param print_time Time up to which moves are finalized
 * 
 * Moves completed moves from the active list to history.
 */
void trapq_finalize_moves(struct trapq *tq, double print_time);

/**
 * @brief Free moves from history older than given time
 * @param tq         Target trapq
 * @param print_time Time before which moves are freed
 */
void trapq_free_moves(struct trapq *tq, double print_time);

/**
 * @brief Get position at a given time
 * @param tq         Target trapq
 * @param print_time Time to query
 * @param pos        Output position
 * @return 0 on success, -1 if no move at that time
 */
int trapq_get_position(const struct trapq *tq, double print_time,
                       struct coord *pos);

/**
 * @brief Calculate position within a move at given time
 * @param m          Move to query
 * @param move_time  Time offset from move start
 * @param pos        Output position
 */
void move_get_coord(const struct move *m, double move_time,
                    struct coord *pos);

/**
 * @brief Get the distance traveled at a given time within a move
 * @param m         Move to query
 * @param move_time Time offset from move start
 * @return Distance traveled (mm)
 */
double move_get_distance(const struct move *m, double move_time);

/**
 * @brief Check if trapq has pending moves
 * @param tq Target trapq
 * @return 1 if moves pending, 0 if empty
 */
int trapq_has_moves(const struct trapq *tq);

/**
 * @brief Get the first move in the queue
 * @param tq Target trapq
 * @return Pointer to first move, or NULL if empty
 */
struct move *trapq_first_move(struct trapq *tq);

/**
 * @brief Get the last move in the queue
 * @param tq Target trapq
 * @return Pointer to last move, or NULL if empty
 */
struct move *trapq_last_move(struct trapq *tq);

#endif /* CHELPER_TRAPQ_H */
