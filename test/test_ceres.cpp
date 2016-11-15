
#include "gmock/gmock.h"  // already contain gtest

#include "ceres_wrapper/ceres_manager.h"
#include "constraint_analytic.h"
#include "problem.h"
#include "wolf.h"

#include "angle_local_parameterization.h"
#include "normalize_angle.h"
#include "pose_graph_2d_error_term.h"
#include "read_g2o.h"

#include <Eigen/Core>
#include <iostream>
#include <map>
#include <utility>  // for std::pair
#include <vector>
using namespace wolf;
using namespace ceres::examples;
using namespace Eigen;

class LocalParametrizationConstrainedAngle : public LocalParametrizationBase {
   public:
    LocalParametrizationConstrainedAngle() : LocalParametrizationBase(1, 1) {}
    virtual ~LocalParametrizationConstrainedAngle();

    virtual bool plus(const Eigen::Map<const Eigen::VectorXs>& _x, const Eigen::Map<const Eigen::VectorXs>& _delta,
                      Eigen::Map<Eigen::VectorXs>& _x_plus_delta) {
        _x_plus_delta = NormalizeAngle(_x + _delta);
        return true;
    }
    virtual bool computeJacobian(const Eigen::Map<const Eigen::VectorXs>& _x,
                                 Eigen::Map<Eigen::MatrixXs>& _jacobian) const = 0;
};

TEST(ceres, justtry) {
    // 1. init stuf
    Problem p(FRM_PO_2D);
    CeresManager ceres_mgr(&p);

    // 2. load vertice and edge data from g2o file
    std::map<int, Pose2d> poses;
    std::vector<Constraint2d> constraints;
    ReadG2oFile("/home/kaihong/workspace/wolf/test/input_M3500_g2o.g2o", &poses, &constraints);
    std::cout << poses.size() << " vertice " << constraints.size() << " edges loaded" << std::endl;

    // 3. create stateblock from vertice
    std::map<int, StateBlock*> vertice_stateblock_map;
    for (auto const& ent : poses) {
        int id = ent.first;
        const Pose2d& pos = ent.second;

        // a. prepair state block
        Eigen::VectorXd var(3);
        var << pos.x, pos.y, pos.yaw_radians;
        StateBlock* sb_ptr = new StateBlock(var);

        sb_ptr->setLocalParametrizationPtr();
        // b. insert
        p.addStateBlockPtr(sb_ptr);

        // c. keep a recored
        vertice_stateblock_map[id] = sb_ptr;
    }
    ASSERT_EQ(p.getStateListPtr()->size(), poses.size());

    // 4. create constraints from edges
    for (auto const& ent : constraints) {
        ent.id_begin;
        new ConstraintAnalytic();
    }

    // 5. add to the problem

    // 6. run the solver
    //     ceres_mgr.solve();
    // plot the result
}
