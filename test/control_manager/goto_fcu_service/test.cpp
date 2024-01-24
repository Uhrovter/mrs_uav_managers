#include <gtest/gtest.h>
#include <tuple>
#include <string>
#include <mrs_uav_testing/test_generic.h>

class Tester : public mrs_uav_testing::TestGeneric {

public:
  bool test();
  mrs_lib::ServiceClientHandler<mrs_msgs::Vec4> sch_goto_fcu_;
  bool gotoFcu(const double &x, const double &y, const double &z, const double &hdg);

};

bool Tester::test() {
  sch_goto_fcu_ = mrs_lib::ServiceClientHandler<mrs_msgs::Vec4>(nh_, "/" + _uav_name_ + "/control_manager/goto_fcu");

  {
    auto [success, message] = activateMidAir();

    if (!success) {
      ROS_ERROR("[%s]: midair activation failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  {
    bool success = this->gotoFcu(0.5, 0, 2.0, 0);

    if (!success) {
      ROS_ERROR("[%s]: goto failed", ros::this_node::getName().c_str());
      return false;
    }

  }

  this->sleep(5.0);

  if (this->isFlyingNormally()) {
    return true;
  } else {
    ROS_ERROR("[%s]: not flying normally", ros::this_node::getName().c_str());
    return false;
  }
}

  bool Tester::gotoFcu(const double &x, const double &y, const double &z, const double &hdg) {
    mrs_msgs::Vec4 srv;
    auto start_pose = sh_estim_manager_diag_.getMsg()->pose.position;
    auto start_hdg  = mrs_lib::AttitudeConverter(sh_estim_manager_diag_.getMsg()->pose.orientation).getHeading();

      srv.request.goal[0] = x;
      srv.request.goal[1] = y;
      srv.request.goal[2] = z;
      srv.request.goal[3] = hdg;

      {
        bool service_call = sch_goto_fcu_.call(srv);

        if (!service_call || !srv.response.success) {
          return false;
        }
      }
      // | -------------------- check for result -------------------- |

    while (true) {

      if (!ros::ok()) {
        return false;
      }

      if (!isFlyingNormally()) {
        return false;
      }

      if (isAtPosition(start_pose.x + x, start_pose.y + y, start_pose.z + z, start_hdg + hdg, 0.1)) {
        return true;
      }

      sleep(0.01);
    }

    return false;
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
