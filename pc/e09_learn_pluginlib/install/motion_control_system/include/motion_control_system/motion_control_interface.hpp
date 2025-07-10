#ifndef MOTION_CONTROL_INTERFACE_HPP
#define MOTION_CONTROL_INTERFACE_HPP

namespace motion_control_system
{
  class MotionController
  {
  private:
  public:

    // Start the motion control system
    virtual void start() = 0;

    // Stop the motion control system
    virtual void stop() = 0;

  };
}

#endif