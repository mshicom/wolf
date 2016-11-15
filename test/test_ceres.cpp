
#include "gmock/gmock.h"  // already contain gtest

#include "ceres_wrapper/ceres_manager.h"
#include "problem.h"
#include "wolf.h"

using namespace wolf;

TEST(ceres, justtry) {
  //
  Problem p(FRM_PO_2D);
  CeresManager ceres_mgr(&p);

  p.addStateBlockPtr()
}
