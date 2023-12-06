#include <gtest/gtest.h>

#include <mrs_uav_testing/test_generic.h>

class Tester : public mrs_uav_testing::TestGeneric {

public:
  bool test();

  Tester();

  mrs_lib::ServiceClientHandler<std_srvs::Trigger> sch_failsafe_;

  std::string _modality_;
};

Tester::Tester() : mrs_uav_testing::TestGeneric() {

  pl_->loadParam("modality", _modality_, std::string());

  sch_failsafe_ = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh_, "/" + _uav_name_ + "/control_manager/failsafe");
}

bool Tester::test() {

  {
    auto [success, message] = activateMidAir();

    if (!success) {
      ROS_ERROR("[%s]: midair activation failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  // | -------------------- trigger failsafe -------------------- |

  {
    std_srvs::Trigger srv;

    {
      bool service_call = sch_failsafe_.call(srv);

      if (!service_call || !srv.response.success) {
        return false;
      }
    }
  }

  // | -------- wait till the right controller is active -------- |

  while (true) {

    if (!ros::ok()) {
      return false;
    }

    if (_modality_ == "position") {
      if (sh_control_manager_diag_.getMsg()->active_controller == "EmergencyController") {
        break;
      }
    } else {
      if (sh_control_manager_diag_.getMsg()->active_controller == "FailsafeController") {
        break;
      }
    }

    sleep(0.1);
  }

  // | ----------------- wait till the UAV lands ---------------- |

  while (true) {

    if (!ros::ok()) {
      return false;
    }

    if (!isOutputEnabled()) {
      return true;
    }

    sleep(0.1);
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

  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "test");

  return RUN_ALL_TESTS();
}
