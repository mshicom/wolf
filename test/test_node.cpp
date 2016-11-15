
#include "gmock/gmock.h"  // already contain gtest

#include "node_base.h"
#include "node_linked.h"
#include "wolf.h"

using namespace testing;
namespace wolf {

TEST(TestNodeBase, Create) {
  NodeBase p("Class", "Type", "Name");
  ASSERT_EQ(p.nodeId(), 1);
  ASSERT_EQ(p.getClass(), "Class");
  ASSERT_EQ(p.getType(), "Type");
  ASSERT_EQ(p.getName(), "Name");

  p.setName("Rename");
  ASSERT_EQ(p.getName(), "Rename");
  p.setType("Retype");
  ASSERT_EQ(p.getType(), "Retype");

  wolf::NodeBase p2("", "", "");
  ASSERT_EQ(p2.nodeId(), 2);
}

template <typename T0, typename T1>
class MockNodeLinked : public NodeLinked<T0, T1> {
 public:
    MockNodeLinked(NodeLocation _loc, std::string _class, std::string _type, std::string _name)
        : NodeLinked<T0, T1>(_loc, _class, _type, _name) {}
//  MOCK_METHOD0_T(destruct, void());
//  MOCK_CONST_METHOD0_T(isDeleting, const bool());
  MOCK_METHOD0_T(die, void());
  virtual ~MockNodeLinked() {die();}
};

class C;
class C : public MockNodeLinked<C,C>{
public:
    C(): MockNodeLinked<C,C>(MID,"C","","") {}
    virtual ~C();
};

//TEST(TestNodeLinked, destruct) {
//  C* robot = new C();
//  C* sensor= new C();

//  EXPECT_CALL(*sensor, die());
//  EXPECT_CALL(*robot, die());

//  robot->addDownNode(sensor);
//  robot->destruct();
//}

// TEST(TestNodeLinked, addDownNode) {
//  Robot* rob_ptr = new Robot();
//  Sensor* sen_ptr_array = new Sensor[2];

//  rob_ptr->addDownNode(&sen_ptr_array[0]);
//  rob_ptr->addDownNode(&sen_ptr_array[1]);

//  ASSERT_EQ(rob_ptr->downNodeList().front(), &sen_ptr_array[0]);
//  ASSERT_EQ(rob_ptr->downNodeList().back(), &sen_ptr_array[1]);

//  ASSERT_EQ(sen_ptr_array[0].upperNodePtr(), rob_ptr);
//  rob_ptr->destruct();
//}

}  // namespace wolf
