#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <mrs_uav_testing/test_generic.h>

using namespace std::chrono_literals;

class Tester : public mrs_uav_testing::TestGeneric {

public:
  Tester() : mrs_uav_testing::TestGeneric() {
    pl_->loadParam("modality", _modality_, std::string());
  }

  bool test(void);

  std::string _modality_;
};

bool Tester::test(void) {

  const std::string uav_name = "uav1";

  std::shared_ptr<mrs_uav_testing::UAVHandler> uh;

  {
    auto [uhopt, message] = getUAVHandler(uav_name);

    if (!uhopt) {
      RCLCPP_ERROR(node_->get_logger(), "Failed obtain handler for '%s': '%s'", uav_name.c_str(), message.c_str());
      return false;
    }

    uh = uhopt.value();
  }

  {
    auto [success, message] = uh->activateMidAir();

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "midair activation failed with message: '%s'", message.c_str());
      return false;
    }
  }

  sleep(3.0);

  // | -------------------- trigger failsafe -------------------- |

  {
    auto [success, message] = uh->failsafe();

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "failsafe activation failed with message: '%s'", message.c_str());
      return false;
    }
  }

  // | -------- wait till the right controller is active -------- |

  while (true) {

    if (!rclcpp::ok()) {
      return false;
    }

    if (_modality_ == "position") {
      if (uh->sh_control_manager_diag_.getMsg()->active_controller == "EmergencyController") {
        break;
      }
    } else {
      if (uh->sh_control_manager_diag_.getMsg()->active_controller == "FailsafeController") {
        break;
      }
    }

    sleep(0.01);
  }

  // | ----------------- wait till the UAV lands ---------------- |

  while (true) {

    if (!rclcpp::ok()) {
      return false;
    }

    if (!uh->isOutputEnabled()) {
      return true;
    }

    sleep(0.01);
  }

  return false;
}

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);

  bool test_result = true;

  Tester tester;

  test_result &= tester.test();

  tester.sleep(2.0);

  std::cout << "Test: reporting test results" << std::endl;

  tester.reportTestResult(test_result);

  tester.join();
}
