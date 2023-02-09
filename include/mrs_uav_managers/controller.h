#ifndef MRS_UAV_CONTROLLER_H
#define MRS_UAV_CONTROLLER_H

/* includes //{ */

#include <ros/ros.h>

#include <mrs_uav_managers/common_handlers.h>

#include <mrs_msgs/HwApiActuatorCmd.h>
#include <mrs_msgs/HwApiControlGroupCmd.h>
#include <mrs_msgs/HwApiAttitudeRateCmd.h>
#include <mrs_msgs/HwApiAttitudeCmd.h>
#include <mrs_msgs/HwApiAccelerationCmd.h>
#include <mrs_msgs/HwApiVelocityCmd.h>
#include <mrs_msgs/HwApiPositionCmd.h>

#include <mrs_msgs/ControllerDiagnostics.h>
#include <mrs_msgs/ControllerStatus.h>
#include <mrs_msgs/TrackerCommand.h>
#include <mrs_msgs/UavState.h>

#include <mrs_msgs/DynamicsConstraintsSrv.h>
#include <mrs_msgs/DynamicsConstraintsSrvRequest.h>
#include <mrs_msgs/DynamicsConstraintsSrvResponse.h>

//}

namespace mrs_uav_managers
{

class Controller {
public:
  typedef struct ControllerOutputs
  {
    bool position      = false;
    bool velocity      = false;
    bool acceleration  = false;
    bool attitude      = false;
    bool attitude_rate = false;
    bool control_group = false;
    bool actuators     = false;
  } ControllerOutputs;

  typedef std::variant<mrs_msgs::HwApiActuatorCmd, mrs_msgs::HwApiControlGroupCmd, mrs_msgs::HwApiAttitudeRateCmd, mrs_msgs::HwApiAttitudeCmd,
                       mrs_msgs::HwApiAccelerationCmd, mrs_msgs::HwApiVelocityCmd, mrs_msgs::HwApiPositionCmd>
      HwApiOutputVariant;

  typedef struct
  {
    std::optional<HwApiOutputVariant> control_output;
    std::optional<Eigen::Quaterniond> desired_orientation;
    mrs_msgs::ControllerDiagnostics   diagnostics;
  } ControlOutput;

  virtual ~Controller() = 0;

  /**
   * @brief Initializes the controller. It is called once for every controller. The runtime is not limited.
   *
   * @param parent_nh the node handle of the ControlManager
   * @param name of the controller for distinguishing multiple running instances of the same code
   * @param name_space the parameter namespace of the controller, can be used during initialization of the private node handle
   * @param uav_mass the net mass of the UAV
   * @param common_handlers handlers shared between trackers and controllers
   */
  virtual void initialize(const ros::NodeHandle &parent_nh, const std::string name, const std::string name_space, const double uav_mass,
                          std::shared_ptr<mrs_uav_managers::CommonHandlers_t>    common_handlers,
                          const mrs_uav_managers::Controller::ControllerOutputs &output_modalities) = 0;

  /**
   * @brief It is called before the controller output will be required and used. Should not take much time (within miliseconds).
   *
   * @param last_attitude_cmd the last command produced by the last active controller. Should be used as an initial condition, e.g., for re-initializing
   * integrators and estimators.
   *
   * @return true if success
   */
  virtual bool activate(const mrs_msgs::ControllerDiagnostics &last_attitude_cmd) = 0;

  /**
   * @brief is called when this controller's output is no longer needed. However, it can be activated later.
   */
  virtual void deactivate(void) = 0;

  /**
   * @brief It may be called to reset the controllers disturbance estimators.
   */
  virtual void resetDisturbanceEstimators(void) = 0;

  /**
   * @brief The most important routine. It is called with every odometry update and it should produce a new control command.
   *
   * TODO
   */
  virtual ControlOutput update(const mrs_msgs::UavState::ConstPtr &uav_state, const mrs_msgs::TrackerCommand::ConstPtr &tracker_command) = 0;
  /**
   * @brief A request for the controller's status.
   *
   * @return the controller's status
   */
  virtual const mrs_msgs::ControllerStatus getStatus() = 0;

  /**
   * @brief It is called during every switch of reference frames of the UAV state estimate.
   * The controller should recalculate its internal states from old the frame to the new one.
   *
   * @param new_uav_state the new UavState which will come in the next update()
   */
  virtual void switchOdometrySource(const mrs_msgs::UavState::ConstPtr &new_uav_state) = 0;

  /**
   * @brief Request for setting new constraints.
   *
   * @param constraints to be set
   *
   * @return a service response
   */
  virtual const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr setConstraints(const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr &constraints) = 0;
};

// A pure virtual destructor requires a function body.
Controller::~Controller(){};

}  // namespace mrs_uav_managers

#endif
