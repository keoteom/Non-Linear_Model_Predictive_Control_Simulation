/* This is an auto-generated file made from optimization engine: https://crates.io/crates/optimization_engine */

#pragma once



#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/**
 * Number of decision variables
 */
#define NMPC_UAV_OBSTACLE_NUM_DECISION_VARIABLES 90

/**
 * Number of parameters
 */
#define NMPC_UAV_OBSTACLE_NUM_PARAMETERS 1548

/**
 * Number of parameters associated with augmented Lagrangian
 */
#define NMPC_UAV_OBSTACLE_N1 372

/**
 * Number of penalty constraints
 */
#define NMPC_UAV_OBSTACLE_N2 120

/**
 * nmpc_uav_obstacle version of ExitStatus
 * Structure: `nmpc_uav_obstacleExitStatus`
 */
typedef enum nmpc_uav_obstacleExitStatus {
  /**
   * The algorithm has converged
   *
   * All termination criteria are satisfied and the algorithm
   * converged within the available time and number of iterations
   */
  nmpc_uav_obstacleConverged,
  /**
   * Failed to converge because the maximum number of iterations was reached
   */
  nmpc_uav_obstacleNotConvergedIterations,
  /**
   * Failed to converge because the maximum execution time was reached
   */
  nmpc_uav_obstacleNotConvergedOutOfTime,
  /**
   * If the gradient or cost function cannot be evaluated internally
   */
  nmpc_uav_obstacleNotConvergedCost,
  /**
   * Computation failed and NaN/Infinite value was obtained
   */
  nmpc_uav_obstacleNotConvergedNotFiniteComputation,
} nmpc_uav_obstacleExitStatus;

/**
 * Solver cache (structure `nmpc_uav_obstacleCache`)
 *
 */
typedef struct nmpc_uav_obstacleCache nmpc_uav_obstacleCache;

/**
 * nmpc_uav_obstacle version of AlmOptimizerStatus
 * Structure: `nmpc_uav_obstacleSolverStatus`
 *
 */
typedef struct nmpc_uav_obstacleSolverStatus {
  /**
   * Exit status
   */
  enum nmpc_uav_obstacleExitStatus exit_status;
  /**
   * Number of outer iterations
   */
  unsigned long num_outer_iterations;
  /**
   * Total number of inner iterations
   *
   * This is the sum of the numbers of iterations of
   * inner solvers
   */
  unsigned long num_inner_iterations;
  /**
   * Norm of the fixed-point residual of the the problem
   */
  double last_problem_norm_fpr;
  /**
   * Total solve time
   */
  unsigned long long solve_time_ns;
  /**
   * Penalty value
   */
  double penalty;
  /**
   * Norm of delta y divided by the penalty parameter
   */
  double delta_y_norm_over_c;
  /**
   * Norm of F2(u)
   */
  double f2_norm;
  /**
   * Value of cost function at solution
   */
  double cost;
  /**
   * Lagrange multipliers
   */
  double lagrange[NMPC_UAV_OBSTACLE_N1];
} nmpc_uav_obstacleSolverStatus;

/**
 * Allocate memory and setup the solver
 */
struct nmpc_uav_obstacleCache *nmpc_uav_obstacle_new(void);

/**
 * Solve the parametric optimization problem for a given parameter
 * .
 * .
 * # Arguments:
 * - `instance`: re-useable instance of AlmCache, which should be created using
 *   `nmpc_uav_obstacle_new` (and should be destroyed once it is not
 *   needed using `nmpc_uav_obstacle_free`
 * - `u`: (on entry) initial guess of solution, (on exit) solution
 *   (length: `NMPC_UAV_OBSTACLE_NUM_DECISION_VARIABLES`)
 * - `params`:  static parameters of the optimizer
 *   (length: `NMPC_UAV_OBSTACLE_NUM_PARAMETERS`)
 * - `y0`: Initial guess of Lagrange multipliers (if `0`, the default will
 *   be used; length: `NMPC_UAV_OBSTACLE_N1`)
 * - `c0`: Initial penalty parameter (provide `0` to use the default initial
 *   penalty parameter
 * .
 * .
 * # Returns:
 * Instance of `nmpc_uav_obstacleSolverStatus`, with the solver status
 * (e.g., number of inner/outer iterations, measures of accuracy, solver time,
 * and the array of Lagrange multipliers at the solution).
 * .
 * .
 * .
 * # Safety
 * All arguments must have been properly initialised
 */
struct nmpc_uav_obstacleSolverStatus nmpc_uav_obstacle_solve(struct nmpc_uav_obstacleCache *instance,
                                                             double *u,
                                                             const double *params,
                                                             const double *y0,
                                                             const double *c0);

/**
 * Deallocate the solver's memory, which has been previously allocated
 * using `nmpc_uav_obstacle_new`
 *
 *
 * # Safety
 * All arguments must have been properly initialised
 */
void nmpc_uav_obstacle_free(struct nmpc_uav_obstacleCache *instance);
