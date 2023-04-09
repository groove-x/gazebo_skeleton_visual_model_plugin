#include <gazebo/common/Mesh.hh>
#include <gazebo/common/MeshManager.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Skeleton.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
class SkeletonModelVisual : public ModelPlugin
{
public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  // Called by the world update start event
  void OnUpdate();

private:
  // Pointer to the node
  transport::NodePtr node;

  // Pointer to the model
  physics::ModelPtr model;

  // Pointer to the world
  physics::WorldPtr world;

  // Pointer to the update event connection
  event::ConnectionPtr updateConnection;

  /// \brief Filename for the skin.
  std::string skinFile;

  /// \brief Scaling factor to apply to the skin.
  double skinScale;

  /// \brief Pointer to the actor's mesh.
  const common::Mesh* mesh = nullptr;

  /// \brief The actor's skeleton.
  common::Skeleton* skeleton = nullptr;

  /// \brief Name of the visual representing the skin.
  std::string visualName;

  /// \brief ID for the visual representing the skin.
  uint32_t visualId;

  /// \brief Publisher to send bone info.
  transport::PublisherPtr bonePosePub;

  bool LoadSkin(sdf::ElementPtr _skinSdf, sdf::ElementPtr _targetSdf);

  void LoadSkeletonLinks(const sdf::ElementPtr _skeletonSdf);

  void SetPose(const double _time);

  void AddSphereInertia(const sdf::ElementPtr& _linkSdf, const ignition::math::Pose3d& _pose, const double _mass,
                        const double _radius);

  void AddSphereCollision(const sdf::ElementPtr& _linkSdf, const std::string& _name,
                          const ignition::math::Pose3d& _pose, const double _radius);

  void AddSphereVisual(const sdf::ElementPtr& _linkSdf, const std::string& _name, const ignition::math::Pose3d& _pose,
                       const double _radius, const std::string& _material, const ignition::math::Color& _ambient);

  void AddBoxVisual(const sdf::ElementPtr& _linkSdf, const std::string& _name, const ignition::math::Pose3d& _pose,
                    const ignition::math::Vector3d& _size, const std::string& _material,
                    const ignition::math::Color& _ambient);

  void AddBoxCollision(const sdf::ElementPtr& _linkSdf, const std::string& _name, const ignition::math::Pose3d& _pose,
                       const ignition::math::Vector3d& _size);

  void AddActorVisual(const sdf::ElementPtr& _linkSdf, const std::string& _name, const ignition::math::Pose3d& _pose);

  void PrintTransform(const std::string& _parentFrame, const std::string& _childFrame,
                      const ignition::math::Matrix4d& _transform);
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(SkeletonModelVisual)
}  // namespace gazebo