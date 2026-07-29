/*
 * DirectJointPositionPlugin — Minimal Gazebo system plugin
 *
 * Subscribes to a Gazebo transport topic for a joint position command
 * and directly resets the joint position + zeros velocity — no PID,
 * no spring, no physics dynamics. Just: command position → joint is there.
 *
 * Usage in model.sdf:
 *   <plugin filename="libDirectJointPositionPlugin.so"
 *           name="my::DirectJointPositionPlugin">
 *     <joint_name>base_to_link0</joint_name>
 *   </plugin>
 *
 * The plugin auto-derives the Gazebo topic:
 *   /model/<model_name>/joint/<joint_name>/0/cmd_pos
 */

#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointVelocityReset.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/double.pb.h>
#include <gz/plugin/Register.hh>
#include <mutex>
#include <string>

namespace my {

class DirectJointPositionPlugin
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate {
 public:
  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager & /*_eventMgr*/) override {
    this->model = gz::sim::Model(_entity);

    // Read joint name from SDF
    auto jointNameElem = _sdf->FindElement("joint_name");
    if (!jointNameElem) {
      gzerr << "DirectJointPositionPlugin: <joint_name> is required\n";
      return;
    }
    this->jointName = jointNameElem->Get<std::string>();

    // Resolve joint entity
    this->jointEntity = this->model.JointByName(_ecm, this->jointName);

    // Read optional topic override
    this->topic = "/model/" + this->model.Name(_ecm) +
                  "/joint/" + this->jointName + "/0/cmd_pos";
    auto topicElem = _sdf->FindElement("topic");
    if (topicElem) {
      this->topic = topicElem->Get<std::string>();
    }

    // Subscribe to Gazebo transport position commands
    this->node.Subscribe(this->topic, &DirectJointPositionPlugin::OnCmd, this);

    gzmsg << "DirectJointPositionPlugin: [" << this->jointName
          << "] listening on [" << this->topic << "]\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo & /*_info*/,
                 gz::sim::EntityComponentManager &_ecm) override {
    if (this->jointEntity == gz::sim::kNullEntity) {
      return;
    }

    double target;
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      target = this->cmd;
    }

    // Reset joint position + zero velocity EVERY physics step.
    // This locks the joint at the commanded position — no PID,
    // no spring, no drift from gravity. Instant, absolute position hold.
    _ecm.SetComponentData<gz::sim::components::JointPositionReset>(
        this->jointEntity, {target});
    _ecm.SetComponentData<gz::sim::components::JointVelocityReset>(
        this->jointEntity, {0.0});
  }

 private:
  void OnCmd(const gz::msgs::Double &_msg) {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->cmd = _msg.data();
  }

  gz::sim::Model model{gz::sim::kNullEntity};
  gz::sim::Entity jointEntity{gz::sim::kNullEntity};
  std::string jointName;
  std::string topic;
  gz::transport::Node node;
  double cmd{0.0};
  std::mutex mutex;
};

}  // namespace my

GZ_ADD_PLUGIN(my::DirectJointPositionPlugin,
              gz::sim::System,
              my::DirectJointPositionPlugin::ISystemConfigure,
              my::DirectJointPositionPlugin::ISystemPreUpdate)
