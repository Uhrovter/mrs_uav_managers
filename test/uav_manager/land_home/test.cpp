#include <gtest/gtest.h>

#include <mrs_uav_testing/test_generic.h>

class Tester : public mrs_uav_testing::TestGeneric {

public:
  bool test();
};

bool Tester::test() {

  // | ------------- wait for the system to be ready ------------ |

  while (true) {

    if (!ros::ok()) {
      return false;
    }

    if (this->mrsSystemReady()) {
      break;
    }
  }

  // | ---------------- save the current position --------------- |

  auto takeoff_pos = this->sh_uav_state_.getMsg()->pose.position;
  auto takeoff_hdg = mrs_lib::AttitudeConverter(this->sh_uav_state_.getMsg()->pose.orientation).getHeading();

  // | ------------------------ take off ------------------------ |

  {
    auto [success, message] = takeoff();

    if (!success) {
      ROS_ERROR("[%s]: takeoff failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  sleep(1.0);

  // | --------------------- goto somewhere --------------------- |

  {
    auto [success, message] = gotoRel(8, 1, 2, 1.2);

    if (!success) {
      ROS_ERROR("[%s]: goto failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  // | ------------------------ land home ----------------------- |

  {
    auto [success, message] = landHome();

    if (!success) {
      ROS_ERROR("[%s]: land home failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  // | ---------------- check the final position ---------------- |

  if (this->isAtPosition(takeoff_pos.x, takeoff_pos.y, takeoff_pos.z, takeoff_hdg, 0.5)) {
    return true;
  } else {
    ROS_ERROR("[%s]: land home did end in wrong place", ros::this_node::getName().c_str());
    return false;
  }
}


TEST(TESTSuite, test) {

  Tester tester;

  bool result = tester.test();

  if (result) {
    GTEST_SUCCEED();
  } else {
    GTEST_FAIL();
  }
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {

  ros::init(argc, argv, "test");

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
