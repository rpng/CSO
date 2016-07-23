#include "g2o_optimize.h"

void initOptimizer(SparseOptimizer & _opt, bool _verbose)
{
    typedef BlockSolver< BlockSolverTraits<-1, -1> >  SclamBlockSolver;
    typedef LinearSolverCholmod<SclamBlockSolver::PoseMatrixType> SclamLinearSolver;

    SclamLinearSolver* linearSolver = new SclamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SclamBlockSolver* blockSolver = new SclamBlockSolver(linearSolver);
    OptimizationAlgorithm* solver = 0;
    solver = new OptimizationAlgorithmLevenberg(blockSolver);

    _opt.setAlgorithm(solver);
    _opt.setVerbose(_verbose);
}

void addVertexSE2(SparseOptimizer & _opt, const SE2 & _initv, const int _id, bool _fixed)
{
    VertexSE2* stereoOffset = new VertexSE2;
    stereoOffset->setId(_id);
    stereoOffset->setEstimate(_initv);
    stereoOffset->setFixed(_fixed);
    _opt.addVertex(stereoOffset);
}

void addEdgeCalib(SparseOptimizer & _opt, int _id, const OdomAndStereoMotion & _measure, const Matrix3D & _infomat)
{
    EdgeCalib* calibEdge = new EdgeCalib;
    calibEdge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(_opt.vertex(_id)));
    calibEdge->setInformation(_infomat);
    calibEdge->setMeasurement(_measure);
    _opt.addEdge(calibEdge);
}

// build structure and optimize
void optimize(SparseOptimizer & _opt, int _numiter)
{
    _opt.initializeOptimization(0);
    _opt.optimize(_numiter);
}

// get x, y, theta of offset.
Vector3D getEstimation(SparseOptimizer & _opt, int _id)
{
    VertexSE2* temp = static_cast<VertexSE2*>(_opt.vertex(_id));
    return temp->estimate().toVector().transpose();
}
