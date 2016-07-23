#ifndef G2O_OPTIMIZE_H
#define G2O_OPTIMIZE_H

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"

#include "g2o/solvers/cholmod/linear_solver_cholmod.h"

#include "g2o/types/sclam2d/odometry_measurement.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/data/data_queue.h"
#include "g2o/types/data/robot_odom.h"

#include "edge_calib.h"

using namespace g2o;

/**
 * \brief graph optimization of the calibration problem.
 */

void initOptimizer(SparseOptimizer &, bool _verbose = false);
void addVertexSE2(SparseOptimizer &, const SE2 &, const int , bool _fixed = false);
void addEdgeCalib(SparseOptimizer &, int , const OdomAndStereoMotion &, const Matrix3D &);

// build structure and optimize
void optimize(SparseOptimizer &, int );

// get x, y, theta of offset.
Vector3D getEstimation(SparseOptimizer &, int _id = 0);

#endif
