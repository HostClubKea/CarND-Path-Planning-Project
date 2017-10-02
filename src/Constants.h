#ifndef PATH_PLANNING_CONSTANTS_H
#define PATH_PLANNING_CONSTANTS_H

/* Map */
#define MAP_MAX_S 6945.554

/* Car size constraint */
#define CAR_LENGTH 7
#define CAR_WIDTH 3

/* Trajectory generation constraints */
#define LANES 3
#define MIN_SPEED 10
#define MAX_SPEED 49.5* 0.44704  //22.12848
#define STEP_SPEED 2.0

#define MAX_S_T 7
#define MIN_S_T 3
#define STEP_S_T 1

#define MIN_D_T 3
#define MAX_D_T 7
#define STEP_D_T 2

/* Cost function coeffs */
#define S_JERK_WEIGHT 0.5
#define S_MAX_JERK 9.0
#define S_MAX_JERK_WEIGHT 100000

#define D_JERK_WEIGHT 0.5
#define D_MAX_JERK 9.0
#define D_MAX_JERK_WEIGHT 100000

#define COLLISION_WEIGHT 1000000

#define VELOCITY_WEIGHT 1
#define VELOCITY_MAX_WEIGHT 100000

#define SAFETY_WEIGHT 1000

#define COST_HORIZON_T 10
#define COST_DT 0.2

/* */
#define CONTROL_DT 0.02

#endif //PATH_PLANNING_CONSTANTS_H
