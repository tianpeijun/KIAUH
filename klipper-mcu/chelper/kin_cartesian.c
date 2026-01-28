/**
 * @file    kin_cartesian.c
 * @brief   Cartesian kinematics implementation
 * 
 * Adapted from Klipper klippy/chelper/kin_cartesian.c for MCU use.
 * Provides position calculation callbacks for Cartesian (X/Y/Z) printers.
 * 
 * In Cartesian kinematics, each axis maps directly to one stepper motor:
 * - X axis -> X stepper
 * - Y axis -> Y stepper
 * - Z axis -> Z stepper
 * - E axis -> Extruder stepper
 * 
 * Key adaptations:
 * - Static memory pools instead of malloc/free
 * - Removed Python FFI markers (__visible)
 * - C99 compatible
 */

#include "itersolve.h"
#include "trapq.h"
#include <math.h>

/* ========== Axis Definitions ========== */

#define AXIS_X      0
#define AXIS_Y      1
#define AXIS_Z      2
#define AXIS_E      3

/* ========== Position Calculation Callbacks ========== */

/**
 * @brief Calculate X axis position at given time within a move
 * 
 * @param sk    Stepper kinematics (contains scale factor)
 * @param m     Current move
 * @param time  Time offset within move
 * @return Position in stepper units (steps)
 */
static double
cartesian_x_calc_position(struct stepper_kinematics *sk, struct move *m,
                          double time)
{
    struct coord pos;
    move_get_coord(m, time, &pos);
    return pos.x * sk->scale;
}

/**
 * @brief Calculate Y axis position at given time within a move
 */
static double
cartesian_y_calc_position(struct stepper_kinematics *sk, struct move *m,
                          double time)
{
    struct coord pos;
    move_get_coord(m, time, &pos);
    return pos.y * sk->scale;
}

/**
 * @brief Calculate Z axis position at given time within a move
 */
static double
cartesian_z_calc_position(struct stepper_kinematics *sk, struct move *m,
                          double time)
{
    struct coord pos;
    move_get_coord(m, time, &pos);
    return pos.z * sk->scale;
}

/**
 * @brief Calculate extruder position at given time within a move
 */
static double
cartesian_e_calc_position(struct stepper_kinematics *sk, struct move *m,
                          double time)
{
    struct coord pos;
    move_get_coord(m, time, &pos);
    return pos.e * sk->scale;
}

/* ========== Kinematics Setup Functions ========== */

/**
 * @brief Setup stepper kinematics for X axis
 * 
 * @param sk            Stepper kinematics to configure
 * @param steps_per_mm  Steps per millimeter for this axis
 */
void
cartesian_stepper_x_setup(struct stepper_kinematics *sk, double steps_per_mm)
{
    sk->axis = AXIS_X;
    sk->scale = steps_per_mm;
    sk->step_dist = 1.0 / steps_per_mm;
    sk->calc_position_cb = cartesian_x_calc_position;
}

/**
 * @brief Setup stepper kinematics for Y axis
 */
void
cartesian_stepper_y_setup(struct stepper_kinematics *sk, double steps_per_mm)
{
    sk->axis = AXIS_Y;
    sk->scale = steps_per_mm;
    sk->step_dist = 1.0 / steps_per_mm;
    sk->calc_position_cb = cartesian_y_calc_position;
}

/**
 * @brief Setup stepper kinematics for Z axis
 */
void
cartesian_stepper_z_setup(struct stepper_kinematics *sk, double steps_per_mm)
{
    sk->axis = AXIS_Z;
    sk->scale = steps_per_mm;
    sk->step_dist = 1.0 / steps_per_mm;
    sk->calc_position_cb = cartesian_z_calc_position;
}

/**
 * @brief Setup stepper kinematics for extruder
 */
void
cartesian_stepper_e_setup(struct stepper_kinematics *sk, double steps_per_mm)
{
    sk->axis = AXIS_E;
    sk->scale = steps_per_mm;
    sk->step_dist = 1.0 / steps_per_mm;
    sk->calc_position_cb = cartesian_e_calc_position;
}

/**
 * @brief Generic setup for any Cartesian axis
 * 
 * @param sk            Stepper kinematics to configure
 * @param axis          Axis index (0=X, 1=Y, 2=Z, 3=E)
 * @param steps_per_mm  Steps per millimeter
 */
void
cartesian_stepper_setup(struct stepper_kinematics *sk, int axis,
                        double steps_per_mm)
{
    switch (axis) {
    case AXIS_X:
        cartesian_stepper_x_setup(sk, steps_per_mm);
        break;
    case AXIS_Y:
        cartesian_stepper_y_setup(sk, steps_per_mm);
        break;
    case AXIS_Z:
        cartesian_stepper_z_setup(sk, steps_per_mm);
        break;
    case AXIS_E:
        cartesian_stepper_e_setup(sk, steps_per_mm);
        break;
    default:
        /* Invalid axis - use X as default */
        cartesian_stepper_x_setup(sk, steps_per_mm);
        break;
    }
}

/* ========== Coordinate Transformation ========== */

/**
 * @brief Convert cartesian coordinates to stepper positions
 * 
 * For Cartesian kinematics, this is a simple scaling operation.
 * 
 * @param pos           Input cartesian position (mm)
 * @param steps_per_mm  Array of steps_per_mm for each axis [X, Y, Z, E]
 * @param steps         Output stepper positions (steps)
 */
void
cartesian_coord_to_steps(const struct coord *pos, const double *steps_per_mm,
                         double *steps)
{
    steps[AXIS_X] = pos->x * steps_per_mm[AXIS_X];
    steps[AXIS_Y] = pos->y * steps_per_mm[AXIS_Y];
    steps[AXIS_Z] = pos->z * steps_per_mm[AXIS_Z];
    steps[AXIS_E] = pos->e * steps_per_mm[AXIS_E];
}

/**
 * @brief Convert stepper positions to cartesian coordinates
 * 
 * @param steps         Input stepper positions (steps)
 * @param steps_per_mm  Array of steps_per_mm for each axis [X, Y, Z, E]
 * @param pos           Output cartesian position (mm)
 */
void
cartesian_steps_to_coord(const double *steps, const double *steps_per_mm,
                         struct coord *pos)
{
    pos->x = steps[AXIS_X] / steps_per_mm[AXIS_X];
    pos->y = steps[AXIS_Y] / steps_per_mm[AXIS_Y];
    pos->z = steps[AXIS_Z] / steps_per_mm[AXIS_Z];
    pos->e = steps[AXIS_E] / steps_per_mm[AXIS_E];
}

/* ========== Limits Checking ========== */

/**
 * @brief Check if position is within axis limits
 * 
 * @param pos       Position to check
 * @param min_pos   Minimum position for each axis [X, Y, Z, E]
 * @param max_pos   Maximum position for each axis [X, Y, Z, E]
 * @return 0 if within limits, -1 if out of bounds
 */
int
cartesian_check_limits(const struct coord *pos, const double *min_pos,
                       const double *max_pos)
{
    if (pos->x < min_pos[AXIS_X] || pos->x > max_pos[AXIS_X]) {
        return -1;
    }
    if (pos->y < min_pos[AXIS_Y] || pos->y > max_pos[AXIS_Y]) {
        return -1;
    }
    if (pos->z < min_pos[AXIS_Z] || pos->z > max_pos[AXIS_Z]) {
        return -1;
    }
    /* Extruder typically doesn't have position limits */
    
    return 0;
}

/**
 * @brief Clamp position to axis limits
 * 
 * @param pos       Position to clamp (modified in place)
 * @param min_pos   Minimum position for each axis
 * @param max_pos   Maximum position for each axis
 */
void
cartesian_clamp_to_limits(struct coord *pos, const double *min_pos,
                          const double *max_pos)
{
    if (pos->x < min_pos[AXIS_X]) {
        pos->x = min_pos[AXIS_X];
    } else if (pos->x > max_pos[AXIS_X]) {
        pos->x = max_pos[AXIS_X];
    }
    
    if (pos->y < min_pos[AXIS_Y]) {
        pos->y = min_pos[AXIS_Y];
    } else if (pos->y > max_pos[AXIS_Y]) {
        pos->y = max_pos[AXIS_Y];
    }
    
    if (pos->z < min_pos[AXIS_Z]) {
        pos->z = min_pos[AXIS_Z];
    } else if (pos->z > max_pos[AXIS_Z]) {
        pos->z = max_pos[AXIS_Z];
    }
}

/* ========== Move Distance Calculation ========== */

/**
 * @brief Calculate the Euclidean distance of a move
 * 
 * @param start Start position
 * @param end   End position
 * @return Distance in mm (XYZ only, not including E)
 */
double
cartesian_move_distance(const struct coord *start, const struct coord *end)
{
    double dx = end->x - start->x;
    double dy = end->y - start->y;
    double dz = end->z - start->z;
    
    return sqrt(dx * dx + dy * dy + dz * dz);
}

/**
 * @brief Calculate the direction vector for a move (normalized)
 * 
 * @param start     Start position
 * @param end       End position
 * @param axes_r    Output direction vector (normalized)
 * @return Move distance in mm
 */
double
cartesian_calc_direction(const struct coord *start, const struct coord *end,
                         struct coord *axes_r)
{
    double dx = end->x - start->x;
    double dy = end->y - start->y;
    double dz = end->z - start->z;
    double de = end->e - start->e;
    
    /* Calculate total distance (including extruder for normalization) */
    double dist = sqrt(dx * dx + dy * dy + dz * dz + de * de);
    
    if (dist < 1e-9) {
        /* Zero-length move */
        axes_r->x = 0.0;
        axes_r->y = 0.0;
        axes_r->z = 0.0;
        axes_r->e = 0.0;
        return 0.0;
    }
    
    /* Normalize direction vector */
    double inv_dist = 1.0 / dist;
    axes_r->x = dx * inv_dist;
    axes_r->y = dy * inv_dist;
    axes_r->z = dz * inv_dist;
    axes_r->e = de * inv_dist;
    
    return dist;
}
