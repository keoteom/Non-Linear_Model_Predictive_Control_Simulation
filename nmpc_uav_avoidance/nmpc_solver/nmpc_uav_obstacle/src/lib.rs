//
// Auto-generated file by OptimizationEngine
// See https://alphaville.github.io/optimization-engine/
//
//

use libc::{c_double, c_ulong, c_ulonglong};

use optimization_engine::{constraints::*, panoc::*, alm::*, *};

// ---Private Constants----------------------------------------------------------------------------------

/// Tolerance of inner solver
const EPSILON_TOLERANCE: f64 = 0.0001;

/// Initial tolerance
const INITIAL_EPSILON_TOLERANCE: f64 = 0.0001;

/// Update factor for inner tolerance
const EPSILON_TOLERANCE_UPDATE_FACTOR: f64 = 0.1;

/// Delta tolerance
const DELTA_TOLERANCE: f64 = 0.001;

/// LBFGS memory
const LBFGS_MEMORY: usize = 10;

/// Maximum number of iterations of the inner solver
const MAX_INNER_ITERATIONS: usize = 800;

/// Maximum number of outer iterations
const MAX_OUTER_ITERATIONS: usize = 10;

/// Maximum execution duration in microseconds
const MAX_DURATION_MICROS: u64 = 5000000;

/// Penalty update factor
const PENALTY_UPDATE_FACTOR: f64 = 5.0;

/// Initial penalty
const INITIAL_PENALTY_PARAMETER: Option<f64> = Some(20.0);

/// Sufficient decrease coefficient
const SUFFICIENT_INFEASIBILITY_DECREASE_COEFFICIENT: f64 = 0.7;

/// Whether preconditioning should be applied
const DO_PRECONDITIONING: bool = false;

// ---Public Constants-----------------------------------------------------------------------------------

/// Number of decision variables
pub const NMPC_UAV_OBSTACLE_NUM_DECISION_VARIABLES: usize = 90;

/// Number of parameters
pub const NMPC_UAV_OBSTACLE_NUM_PARAMETERS: usize = 1548;

/// Number of parameters associated with augmented Lagrangian
pub const NMPC_UAV_OBSTACLE_N1: usize = 372;

/// Number of penalty constraints
pub const NMPC_UAV_OBSTACLE_N2: usize = 120;

// ---Export functionality from Rust to C/C++------------------------------------------------------------

/// Solver cache (structure `nmpc_uav_obstacleCache`)
///
#[allow(non_camel_case_types)]
pub struct nmpc_uav_obstacleCache {
    cache: AlmCache,
}

impl nmpc_uav_obstacleCache {
    pub fn new(cache: AlmCache) -> Self {
        nmpc_uav_obstacleCache { cache }
    }
}

/// nmpc_uav_obstacle version of ExitStatus
/// Structure: `nmpc_uav_obstacleExitStatus`
#[allow(non_camel_case_types)]
#[repr(C)]
pub enum nmpc_uav_obstacleExitStatus {
    /// The algorithm has converged
    ///
    /// All termination criteria are satisfied and the algorithm
    /// converged within the available time and number of iterations
    nmpc_uav_obstacleConverged,
    /// Failed to converge because the maximum number of iterations was reached
    nmpc_uav_obstacleNotConvergedIterations,
    /// Failed to converge because the maximum execution time was reached
    nmpc_uav_obstacleNotConvergedOutOfTime,
    /// If the gradient or cost function cannot be evaluated internally
    nmpc_uav_obstacleNotConvergedCost,
    /// Computation failed and NaN/Infinite value was obtained
    nmpc_uav_obstacleNotConvergedNotFiniteComputation,
}

/// nmpc_uav_obstacle version of AlmOptimizerStatus
/// Structure: `nmpc_uav_obstacleSolverStatus`
///
#[repr(C)]
pub struct nmpc_uav_obstacleSolverStatus {
    /// Exit status
    exit_status: nmpc_uav_obstacleExitStatus,
    /// Number of outer iterations
    num_outer_iterations: c_ulong,
    /// Total number of inner iterations
    ///
    /// This is the sum of the numbers of iterations of
    /// inner solvers
    num_inner_iterations: c_ulong,
    /// Norm of the fixed-point residual of the the problem
    last_problem_norm_fpr: c_double,
    /// Total solve time
    solve_time_ns: c_ulonglong,
    /// Penalty value
    penalty: c_double,
    /// Norm of delta y divided by the penalty parameter
    delta_y_norm_over_c: c_double,
    /// Norm of F2(u)
    f2_norm: c_double,
    /// Value of cost function at solution
    cost: c_double,
    /// Lagrange multipliers
    lagrange: [c_double; NMPC_UAV_OBSTACLE_N1]
    }

/// Allocate memory and setup the solver
#[no_mangle]
pub extern "C" fn nmpc_uav_obstacle_new() -> *mut nmpc_uav_obstacleCache {
    Box::into_raw(Box::new(nmpc_uav_obstacleCache::new(initialize_solver())))
}

/// Solve the parametric optimization problem for a given parameter
/// .
/// .
/// # Arguments:
/// - `instance`: re-useable instance of AlmCache, which should be created using
///   `nmpc_uav_obstacle_new` (and should be destroyed once it is not
///   needed using `nmpc_uav_obstacle_free`
/// - `u`: (on entry) initial guess of solution, (on exit) solution
///   (length: `NMPC_UAV_OBSTACLE_NUM_DECISION_VARIABLES`)
/// - `params`:  static parameters of the optimizer
///   (length: `NMPC_UAV_OBSTACLE_NUM_PARAMETERS`)
/// - `y0`: Initial guess of Lagrange multipliers (if `0`, the default will
///   be used; length: `NMPC_UAV_OBSTACLE_N1`)
/// - `c0`: Initial penalty parameter (provide `0` to use the default initial
///   penalty parameter
/// .
/// .
/// # Returns:
/// Instance of `nmpc_uav_obstacleSolverStatus`, with the solver status
/// (e.g., number of inner/outer iterations, measures of accuracy, solver time,
/// and the array of Lagrange multipliers at the solution).
/// .
/// .
/// .
/// # Safety
/// All arguments must have been properly initialised
#[no_mangle]
pub unsafe extern "C" fn nmpc_uav_obstacle_solve(
    instance: *mut nmpc_uav_obstacleCache,
    u: *mut c_double,
    params: *const c_double,
    y0: *const c_double,
    c0: *const c_double,
) -> nmpc_uav_obstacleSolverStatus {

    // Convert all pointers into the required data structures
    let instance: &mut nmpc_uav_obstacleCache = {
        assert!(!instance.is_null());
        &mut *instance
    };

    // "*mut c_double" to "&mut [f64]"
    let u : &mut [f64] = {
        assert!(!u.is_null());
        std::slice::from_raw_parts_mut(u as *mut f64, NMPC_UAV_OBSTACLE_NUM_DECISION_VARIABLES)
    };

    // "*const c_double" to "&[f64]"
    let params : &[f64] = {
        assert!(!params.is_null());
        std::slice::from_raw_parts(params as *const f64, NMPC_UAV_OBSTACLE_NUM_PARAMETERS)
    };

    let c0_option: Option<f64> = if c0.is_null() {
        None::<f64>
    } else {
        Some(*c0)
    };

    let y0_option: Option<Vec<f64>> = if y0.is_null() {
        None::<Vec<f64>>
    } else {
        Some(std::slice::from_raw_parts(y0 as *mut f64, NMPC_UAV_OBSTACLE_N1).to_vec())
    };

    // Invoke `solve`
    let status = solve(params,&mut instance.cache, u, &y0_option, &c0_option);

    // Check solution status and cast it as `nmpc_uav_obstacleSolverStatus`
    match status {
        Ok(status) => nmpc_uav_obstacleSolverStatus {
            exit_status: match status.exit_status() {
                core::ExitStatus::Converged => nmpc_uav_obstacleExitStatus::nmpc_uav_obstacleConverged,
                core::ExitStatus::NotConvergedIterations => nmpc_uav_obstacleExitStatus::nmpc_uav_obstacleNotConvergedIterations,
                core::ExitStatus::NotConvergedOutOfTime => nmpc_uav_obstacleExitStatus::nmpc_uav_obstacleNotConvergedOutOfTime,
            },
            num_outer_iterations: status.num_outer_iterations() as c_ulong,
            num_inner_iterations: status.num_inner_iterations() as c_ulong,
            last_problem_norm_fpr: status.last_problem_norm_fpr(),
            solve_time_ns: status.solve_time().as_nanos() as c_ulonglong,
            penalty: status.penalty() as c_double,
            delta_y_norm_over_c: status.delta_y_norm_over_c() as c_double,
            f2_norm: status.f2_norm() as c_double,
            cost: status.cost() as c_double,
            lagrange: match status.lagrange_multipliers() {
                Some(y) => {
                    let mut y_array : [f64; NMPC_UAV_OBSTACLE_N1] = [0.0; NMPC_UAV_OBSTACLE_N1];
                    y_array.copy_from_slice(y);
                    y_array
                
                },
                None => {
                    [0.0; NMPC_UAV_OBSTACLE_N1]
                }
            }
        },
        Err(e) => nmpc_uav_obstacleSolverStatus {
            exit_status: match e {
                SolverError::Cost => nmpc_uav_obstacleExitStatus::nmpc_uav_obstacleNotConvergedCost,
                SolverError::NotFiniteComputation => nmpc_uav_obstacleExitStatus::nmpc_uav_obstacleNotConvergedNotFiniteComputation,
            },
            num_outer_iterations: std::u64::MAX as c_ulong,
            num_inner_iterations: std::u64::MAX as c_ulong,
            last_problem_norm_fpr: std::f64::INFINITY,
            solve_time_ns: std::u64::MAX as c_ulonglong,
            penalty: std::f64::INFINITY as c_double,
            delta_y_norm_over_c: std::f64::INFINITY as c_double,
            f2_norm: std::f64::INFINITY as c_double,
            cost: std::f64::INFINITY as c_double,
            lagrange:[0.0; NMPC_UAV_OBSTACLE_N1]
        },
    }
}

/// Deallocate the solver's memory, which has been previously allocated
/// using `nmpc_uav_obstacle_new`
/// 
/// 
/// # Safety
/// All arguments must have been properly initialised
#[no_mangle]
pub unsafe extern "C" fn nmpc_uav_obstacle_free(instance: *mut nmpc_uav_obstacleCache) {
    // Add impl
    assert!(!instance.is_null());
    drop(Box::from_raw(instance));
}


// ---Parameters of the constraints----------------------------------------------------------------------



const CONSTRAINTS_XMIN :Option<&[f64]> = Some(&[5.0,-0.35,-0.35,5.0,-0.35,-0.35,5.0,-0.35,-0.35,5.0,-0.35,-0.35,5.0,-0.35,-0.35,5.0,-0.35,-0.35,5.0,-0.35,-0.35,5.0,-0.35,-0.35,5.0,-0.35,-0.35,5.0,-0.35,-0.35,5.0,-0.35,-0.35,5.0,-0.35,-0.35,5.0,-0.35,-0.35,5.0,-0.35,-0.35,5.0,-0.35,-0.35,5.0,-0.35,-0.35,5.0,-0.35,-0.35,5.0,-0.35,-0.35,5.0,-0.35,-0.35,5.0,-0.35,-0.35,5.0,-0.35,-0.35,5.0,-0.35,-0.35,5.0,-0.35,-0.35,5.0,-0.35,-0.35,5.0,-0.35,-0.35,5.0,-0.35,-0.35,5.0,-0.35,-0.35,5.0,-0.35,-0.35,5.0,-0.35,-0.35,5.0,-0.35,-0.35,]);
const CONSTRAINTS_XMAX :Option<&[f64]> = Some(&[13.5,0.35,0.35,13.5,0.35,0.35,13.5,0.35,0.35,13.5,0.35,0.35,13.5,0.35,0.35,13.5,0.35,0.35,13.5,0.35,0.35,13.5,0.35,0.35,13.5,0.35,0.35,13.5,0.35,0.35,13.5,0.35,0.35,13.5,0.35,0.35,13.5,0.35,0.35,13.5,0.35,0.35,13.5,0.35,0.35,13.5,0.35,0.35,13.5,0.35,0.35,13.5,0.35,0.35,13.5,0.35,0.35,13.5,0.35,0.35,13.5,0.35,0.35,13.5,0.35,0.35,13.5,0.35,0.35,13.5,0.35,0.35,13.5,0.35,0.35,13.5,0.35,0.35,13.5,0.35,0.35,13.5,0.35,0.35,13.5,0.35,0.35,13.5,0.35,0.35,]);




// ---Parameters of ALM-type constraints (Set C)---------------------------------------------------------
const SET_C_XMIN :Option<&[f64]> = Some(&[std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,std::f64::NEG_INFINITY,]);
const SET_C_XMAX :Option<&[f64]> = Some(&[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,]);




// ---Parameters of ALM-type constraints (Set Y)---------------------------------------------------------
/// Constraints: Centre of Euclidean Ball
const SET_Y_BALL_XC: Option<&[f64]> = None;

/// Constraints: Radius of Euclidean Ball
const SET_Y_BALL_RADIUS : f64 = 1000000.0;




// ---Internal private helper functions------------------------------------------------------------------

/// Make constraints U
fn make_constraints() -> impl Constraint {
    // - Rectangle:
    Rectangle::new(CONSTRAINTS_XMIN, CONSTRAINTS_XMAX)
    }

/// Make set C
fn make_set_c() -> impl Constraint {
    Rectangle::new(SET_C_XMIN, SET_C_XMAX)
    }


/// Make set Y
fn make_set_y() -> impl Constraint {
    BallInf::new(SET_Y_BALL_XC, SET_Y_BALL_RADIUS)
    }


// ---Main public API functions--------------------------------------------------------------------------


/// Initialisation of the solver
pub fn initialize_solver() -> AlmCache {
    let panoc_cache = PANOCCache::new(NMPC_UAV_OBSTACLE_NUM_DECISION_VARIABLES, EPSILON_TOLERANCE, LBFGS_MEMORY);
    AlmCache::new(panoc_cache, NMPC_UAV_OBSTACLE_N1, NMPC_UAV_OBSTACLE_N2)
}

/// If preconditioning has been applied, then at the end (after a solution has been obtained)
/// we need to undo the scaling and update the cost function
fn unscale_result(solver_status: &mut Result<AlmOptimizerStatus, SolverError>) {
    if let Ok(sss) = solver_status {
        let w_cost : f64 = icasadi_nmpc_uav_obstacle::get_w_cost();
        sss.update_cost(sss.cost() / w_cost);
    }
}

/// Solver interface
///
/// ## Arguments
/// - `p`: static parameter vector of the optimization problem
/// - `alm_cache`: Instance of AlmCache
/// - `u`: Initial guess
/// - `y0` (optional) initial vector of Lagrange multipliers
/// - `c0` (optional) initial penalty
///
/// ## Returns
/// This function returns either an instance of AlmOptimizerStatus with information about the
/// solution, or a SolverError object if something goes wrong
pub fn solve(
    p: &[f64],
    alm_cache: &mut AlmCache,
    u: &mut [f64],
    y0: &Option<Vec<f64>>,
    c0: &Option<f64>,
) -> Result<AlmOptimizerStatus, SolverError> {

    assert_eq!(p.len(), NMPC_UAV_OBSTACLE_NUM_PARAMETERS, "Wrong number of parameters (p)");
    assert_eq!(u.len(), NMPC_UAV_OBSTACLE_NUM_DECISION_VARIABLES, "Wrong number of decision variables (u)");

    // Start by initialising the optimiser interface (e.g., set w=1)
    icasadi_nmpc_uav_obstacle::init_nmpc_uav_obstacle();

    let mut rho_init : f64 = 1.0;
    if DO_PRECONDITIONING {
        // Compute the preconditioning parameters (w's)
        // The scaling parameters will be stored internally in `interface.c`
        icasadi_nmpc_uav_obstacle::precondition(u, p);

        // Compute initial penalty
        icasadi_nmpc_uav_obstacle::initial_penalty(u, p, & mut rho_init);
    }

    let psi = |u: &[f64], xi: &[f64], cost: &mut f64| -> Result<(), SolverError> {
        icasadi_nmpc_uav_obstacle::cost(u, xi, p, cost);
        Ok(())
    };
    let grad_psi = |u: &[f64], xi: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
        icasadi_nmpc_uav_obstacle::grad(u, xi, p, grad);
        Ok(())
    };
    
    let f1 = |u: &[f64], res: &mut [f64]| -> Result<(), SolverError> {
        icasadi_nmpc_uav_obstacle::mapping_f1(u, p, res);
        Ok(())
    };
    let f2 = |u: &[f64], res: &mut [f64]| -> Result<(), SolverError> {
        icasadi_nmpc_uav_obstacle::mapping_f2(u, p, res);
        Ok(())
    };let bounds = make_constraints();

    let set_y = make_set_y();
    let set_c = make_set_c();
    let alm_problem = AlmProblem::new(
        bounds,
        Some(set_c),
        Some(set_y),
        psi,
        grad_psi,
        Some(f1),
        Some(f2),
        NMPC_UAV_OBSTACLE_N1,
        NMPC_UAV_OBSTACLE_N2,
    );

    let mut alm_optimizer = AlmOptimizer::new(alm_cache, alm_problem)
        .with_delta_tolerance(DELTA_TOLERANCE)
        .with_epsilon_tolerance(EPSILON_TOLERANCE)
        .with_initial_inner_tolerance(INITIAL_EPSILON_TOLERANCE)
        .with_inner_tolerance_update_factor(EPSILON_TOLERANCE_UPDATE_FACTOR)
        .with_max_duration(std::time::Duration::from_micros(MAX_DURATION_MICROS))
        .with_max_outer_iterations(MAX_OUTER_ITERATIONS)
        .with_max_inner_iterations(MAX_INNER_ITERATIONS)
        .with_initial_penalty(c0.unwrap_or(INITIAL_PENALTY_PARAMETER.unwrap_or(rho_init)))
        .with_penalty_update_factor(PENALTY_UPDATE_FACTOR)
        .with_sufficient_decrease_coefficient(SUFFICIENT_INFEASIBILITY_DECREASE_COEFFICIENT);

    // solve the problem using `u`, the initial condition `u`, and
    // initial vector of Lagrange multipliers, if provided;
    // returns the problem status (instance of `AlmOptimizerStatus`)
    if let Some(y0_) = y0 {
        let mut alm_optimizer = alm_optimizer.with_initial_lagrange_multipliers(y0_);
        let mut solution_status = alm_optimizer.solve(u);
        unscale_result(&mut solution_status);
        solution_status
    } else {
        let mut solution_status = alm_optimizer.solve(u);
        unscale_result(&mut solution_status);
        solution_status
    }

}