#include "iostream"
#include "motion_control_system/spin_motion_controller.hpp"

//命名空间
namespace motion_control_system
{
  void SpinMotionController::start()
  {
    //实现旋转运动控制逻辑
    std::cout << "SpinMotionController started." << std::endl;
  }

  void SpinMotionController::stop()
  {
    //停止运动控制
    std::cout << "SpinMotionController stopped." << std::endl;
  }
} // namespace motion_control_system

//提供PLUGINLIB_EXPORT_CLASS宏定义，一个类，一个基类；
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(motion_control_system::SpinMotionController, motion_control_system::MotionController)