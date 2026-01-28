/**
 * @file    kin_cartesian.h
 * @brief   Cartesian kinematics interface
 * 
 * Adapted from Klipper klippy/chelper/kin_cartesian.c for MCU use.
 * Provides position calculation callbacks for Cartesian (X/Y/Z) printers.
 */

#ifndef CHELPER_KIN_CARTESIAN_H
#define CHELPER_KIN_CARTESIAN_H

#include "itersolve.h"
#include "trapq.h"

/* ========== Axis Definitions ========== */

#define CARTESIAN_AXIS_X    0
#define CARTESIAN_AXIS_Y    1
#define CARTESIAN_AXIS_Z    2
#define CARTESIAN_AXIS_E    3
#define CARTESIAN_NUM_AXES  4

/* ========== Kinematics Setup Functions ========== */

/**
 * @brief Setup stepper kinematics for X axis
 * @param sk            Stepper kinematics to configure
 * @param steps_per_mm  Steps per millimeter for this axis
 */
void cartesian_stepper_x_setup(struct stepper_kinematics *sk, double steps_per_mm);

/**
 * @brief Setup stepper kinematics for Y axis
 */
void cartesian_stepper_y_setup(struct stepper_kinematics *sk, double steps_per_mm);

/**
 * @brief Setup stepper kinematics for Z axis
 */
void cartesian_stepper_z_setup(struct stepper_kinematics *sk, double steps_per_mm);

/**
 * @brief Setup stepper kinematics for extruder
 */
void cartesian_stepper_e_setup(struct stepper_kinematics *sk, double steps_per_mm);

/**
 * @brief Generic setup for any Cartesian axis
 * @param sk            Stepper kinematics to configure
 * @param axis          Axis index (0=X, 1=Y, 2=Z, 3=E)
 * @param steps_per_mm  Steps per millimeter
 */
void cartesian_stepper_setup(struct stepper_kinematics *sk, int axis,
                             double steps_per_mm);

/* ========== Coordinate Transformation ========== */

/**
 * @brief Convert cartesian coordinates to stepper positions
 * @param pos           Input cartesian position (mm)
 * @param steps_per_mm  Array of steps_per_mm for each axis [X, Y, Z, E]
 * @param steps         Output stepper positions (steps)
 */
void cartesian_coord_to_steps(const struct coord *pos, const double *steps_per_mm,
                              double *steps);

/**
 * @brief Convert stepper positions to cartesian coordinates
 * @param steps         Input stepper positions (steps)
 * @param steps_per_mm  Array of steps_per_mm for each axis [X, Y, Z, E]
 * @param pos           Output cartesian position (mm)
 */
void cartesian_steps_to_coord(const double *steps, const double *steps_per_mm,
                              struct coord *pos);

/* ========== Limits Checking ========== */

/**
 * @brief Check if position is within axis limits
 * @param pos       Position to check
 * @param min_pos   Minimum position for each axis [X, Y, Z, E]
 * @param max_pos   Maximum position for each axis [X, Y, Z, E]
 * @return 0 if within limits, -1 if out of bounds
 */
int cartesian_check_limits(const struct coord *pos, const double *min_pos,
                           const double *max_pos);

/**
 * @brief Clamp position to axis limits
 * @param pos       Position to clamp (modified in place)
 * @param min_pos   Minimum position for each axis
 * @param max_pos   Maximum position for each axis
 */
void cartesian_clamp_to_limits(struct coord *pos, const double *min_pos,
                               const double *max_pos);

/* ========== Move Distance Calculation ========== */

/**
 * @brief Calculate the Euclidean distance of a move
 * @param start Start position
 * @param end   End position
 * @return Distance in mm (XYZ only, not including E)
 */
double cartesian_move_distance(const struct coord *start, const struct coord *end);

/**
 * @brief Calculate the direction vector for a move (normalized)
 * @param start     Start position
 * @param end       End position
 * @param axes_r    Output direction vector (normalized)
 * @return Move distance in mm
 */
double cartesian_calc_direction(const struct coord *start, const struct coord *end,
                                struct coord *axes_r);

#endif /* CHELPER_KIN_CARTESIAN_H */
