#include "gazebo_skeleton_visual_model_plugin/skeleton_model_plugin.hpp"

namespace gazebo
{
void SkeletonModelVisual::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  this->model = _parent;
  this->world = this->model->GetWorld();
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->world->Name());

  // Parse skin
  if (_sdf->HasElement("skin"))
  {
    gzmsg << "SkeletonModelVisual::Load has skin." << std::endl;
    // Only load skeleton animations if we get a skeleton from the skin
    auto skeletonLinksSdf = sdf::ElementPtr(new sdf::Element());
    sdf::initFile("model.sdf", skeletonLinksSdf);
    if (this->LoadSkin(_sdf->GetElement("skin"), skeletonLinksSdf) && this->skeleton)
    {
      gzmsg << "SkeletonModelVisual::Load has skeleton." << std::endl;
      gzmsg << "NumNodes: " << this->skeleton->GetNumNodes() << std::endl;
      gzmsg << "NumJoints: " << this->skeleton->GetNumJoints() << std::endl;
      this->LoadSkeletonLinks(skeletonLinksSdf);
    }
  }
  else
  {
    gzerr << "SkeletonModelVisual::Load has no skin." << std::endl;
  }

  // If there is a skin, check that the skin visual was created and save its
  // id
  std::string modelName = this->model->GetName();
  std::string modelLinkName = modelName + "_pose";
  gzmsg << "ModelLinkName: " << modelLinkName << std::endl;
  physics::LinkPtr modelLinkPtr = this->model->GetLink(modelLinkName);
  if (modelLinkPtr)
  {
    msgs::Visual actorVisualMsg = modelLinkPtr->GetVisualMessage(this->visualName);
    if (actorVisualMsg.has_id())
    {
      this->visualId = actorVisualMsg.id();
      gzmsg << "VisualId: " << this->visualId << std::endl;
    }
    else
    {
      gzerr << "Message for actor visual [" << modelLinkName << "] not found." << std::endl;
    }
  }
  else
  {
    gzerr << "Model link [" << modelLinkName << "] not found." << std::endl;
  }

  this->bonePosePub = this->node->Advertise<msgs::PoseAnimation>("~/skeleton_pose/info", 10);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&SkeletonModelVisual::OnUpdate, this));
}

// Called by the world update start event
void SkeletonModelVisual::OnUpdate()
{
  common::Time currentTime = this->world->SimTime();
  SetPose(currentTime.Double());
}

bool SkeletonModelVisual::LoadSkin(sdf::ElementPtr _skinSdf, sdf::ElementPtr _targetSdf)
{
  this->skinFile = _skinSdf->Get<std::string>("filename");
  this->skinScale = _skinSdf->Get<double>("scale");
  this->printTransforms = _skinSdf->Get<bool>("printTransforms");

  // Create unordered map of bones
  // <bone name="Bone1" parent="body" link="bone1" />
  // <bone name="Bone2" parent="bone1" link="bone2" />
  // std::unordered_map<std::string, BoneLinks> boneLinks;
  for (auto boneSdf = _skinSdf->GetElement("bone"); boneSdf; boneSdf = boneSdf->GetNextElement("bone"))
  {
    std::string boneName = boneSdf->Get<std::string>("name");
    std::string boneParent = boneSdf->Get<std::string>("parent");
    std::string boneLink = boneSdf->Get<std::string>("link");
    this->boneLinks.insert(std::make_pair(boneName, BoneLinks(boneParent, boneLink)));
    gzmsg << "Bone: " << boneName << " " << boneParent << " " << boneLink << std::endl;
  }

  common::MeshManager::Instance()->Load(this->skinFile);
  if (!common::MeshManager::Instance()->HasMesh(this->skinFile))
  {
    gzwarn << "Couldn't find mesh [" << this->skinFile << "]. "
           << "Not loading skin." << std::endl;
    return false;
  }

  this->mesh = common::MeshManager::Instance()->GetMesh(this->skinFile);
  if (!this->mesh || !this->mesh->HasSkeleton())
  {
    gzwarn << "Collada file [" << this->skinFile << "] does not contain skeletal animation." << std::endl;
    return false;
  }

  this->skeleton = this->mesh->GetSkeleton();

  auto modelName = this->model->GetName();
  auto linkSdf = _targetSdf->GetElement("link");
  linkSdf->GetAttribute("name")->Set(modelName + "_pose");
  linkSdf->GetElement("gravity")->Set(false);
  linkSdf->GetElement("self_collide")->Set(false);

  std::string modelLinkName = modelName + "::" + modelName + "_pose";
  this->visualName = modelLinkName + "::" + modelName + "_visual";
  this->AddActorVisual(linkSdf, modelName + "_visual", ignition::math::Pose3d::Zero);

  // Create spherical links for each skeleton node
  auto nodes = this->skeleton->GetNodes();
  for (auto iter : nodes)
  {
    gzmsg << "Loading node: " << iter.first << " " << iter.second->GetName() << std::endl;
    common::SkeletonNode* bone = iter.second;

    // Add link element
    linkSdf = _targetSdf->AddElement("link");

    // Set default properties
    linkSdf->GetAttribute("name")->Set(bone->GetName());
    linkSdf->GetElement("gravity")->Set(false);
    linkSdf->GetElement("self_collide")->Set(false);

    // Set pose
    ignition::math::Pose3d pose(bone->ModelTransform().Translation(), bone->ModelTransform().Rotation());
    if (bone->IsRootNode())
      pose = ignition::math::Pose3d::Zero;

    linkSdf->GetElement("pose")->Set(pose);

    // FIXME hardcoded inertia of a sphere with mass 1.0 and radius 0.01
    // Do we even need inertial info in an actor?
    // this->AddSphereInertia(linkSdf, ignition::math::Pose3d::Zero, 1.0, 0.01);

    // FIXME hardcoded visual to red sphere with radius 0.02
    if (bone->IsRootNode())
    {
      this->AddSphereVisual(linkSdf, bone->GetName() + "__SKELETON_VISUAL__", ignition::math::Pose3d::Zero, 0.02,
                            "Gazebo/Blue", ignition::math::Color::Blue);
    }
    else if (bone->GetChildCount() == 0)
    {
      this->AddSphereVisual(linkSdf, bone->GetName() + "__SKELETON_VISUAL__", ignition::math::Pose3d::Zero, 0.02,
                            "Gazebo/Yellow", ignition::math::Color::Yellow);
    }
    else
    {
      this->AddSphereVisual(linkSdf, bone->GetName() + "__SKELETON_VISUAL__", ignition::math::Pose3d::Zero, 0.02,
                            "Gazebo/Red", ignition::math::Color::Red);
    }

    // Create a box visual representing each bone
    for (unsigned int i = 0; i < bone->GetChildCount(); ++i)
    {
      common::SkeletonNode* curChild = bone->GetChild(i);
      gzmsg << "Loading child node: " << curChild->GetName() << std::endl;

      ignition::math::Vector3d dir = curChild->ModelTransform().Translation() - bone->ModelTransform().Translation();
      double length = dir.Length();

      if (!ignition::math::equal(length, 0.0))
      {
        ignition::math::Vector3d r = curChild->Transform().Translation();
        ignition::math::Vector3d linkPos = ignition::math::Vector3d(r.X() / 2.0, r.Y() / 2.0, r.Z() / 2.0);
        double theta = atan2(dir.Y(), dir.X());
        double phi = acos(dir.Z() / length);

        ignition::math::Pose3d bonePose(linkPos, ignition::math::Quaterniond(0.0, phi, theta));
        bonePose.Rot() = pose.Rot().Inverse() * bonePose.Rot();

        this->AddBoxVisual(linkSdf, bone->GetName() + "_" + curChild->GetName() + "__SKELETON_VISUAL__", bonePose,
                           ignition::math::Vector3d(0.02, 0.02, length), "Gazebo/Green", ignition::math::Color::Green);
        // this->AddBoxCollision(
        //     linkSdf,
        //     bone->GetName() + "_" + curChild->GetName() + "_collision",
        //     bonePose, ignition::math::Vector3d(0.02, 0.02, length));
      }
    }
  }
  return true;
}

void SkeletonModelVisual::LoadSkeletonLinks(const sdf::ElementPtr _skeletonSdf)
{
  for (auto elem = _skeletonSdf->GetFirstElement(); elem; elem = elem->GetNextElement())
  {
    assert(elem->GetName() == "link");
    auto linkName = elem->Get<std::string>("name");
    auto linkPtr = this->model->CreateLink(linkName);
    gzmsg << "Attaching link: " << linkName << std::endl;
    linkPtr->Load(elem);
  }
}

void SkeletonModelVisual::PrintTransform(const std::string& _parentFrame, const std::string& _childFrame,
                                         const ignition::math::Matrix4d& _transform)
{
  ignition::math::Pose3d pose;
  pose.Pos() = _transform.Translation();
  pose.Rot() = _transform.Rotation();
  gzmsg << "Transform from " << _parentFrame << " to " << _childFrame << ": " << pose << std::endl;
}

void SkeletonModelVisual::SetPose(const double _time)
{
  msgs::PoseAnimation msg;
  msg.set_model_name(this->visualName);
  msg.set_model_id(this->visualId);

  ignition::math::Pose3d mainLinkPose = this->model->WorldPose();

  if (this->printTransforms)
  {
    gzmsg << "====" << std::endl;
  }

  for (unsigned int i = 0; i < this->skeleton->GetNumNodes(); ++i)
  {
    common::SkeletonNode* bone = this->skeleton->GetNodeByHandle(i);
    common::SkeletonNode* parentBone = bone->GetParent();
    ignition::math::Matrix4d transform(ignition::math::Matrix4d::Identity);
    transform = bone->Transform();

    // Copy relative transform between parent and child link
    // from Gazebo physics model to skeleton
    auto it = this->boneLinks.find(bone->GetName());
    if (it != this->boneLinks.end())
    {
      const auto& linkNames = it->second;
      auto parentLink = this->model->GetLink(linkNames.parent);
      auto boneLink = this->model->GetLink(linkNames.child);
      if (parentLink && boneLink)
      {
        auto t = boneLink->WorldPose() - parentLink->WorldPose();
        transform = ignition::math::Matrix4d(t);
      }
    }

    physics::LinkPtr currentLink = this->model->GetChildLink(bone->GetName());
    ignition::math::Pose3d bonePose = transform.Pose();
    if (!bonePose.IsFinite())
    {
      bonePose.Correct();
    }

    msgs::Pose* bone_pose = msg.add_pose();
    bone_pose->set_name(bone->GetName());
    bone_pose->mutable_position()->CopyFrom(msgs::Convert(bonePose.Pos()));
    bone_pose->mutable_orientation()->CopyFrom(msgs::Convert(bonePose.Rot()));

    // Propagate the pose from parent
    if (!parentBone)
    {
      ignition::math::Matrix4d parentTrans(mainLinkPose);
      transform = parentTrans * transform;
    }
    else
    {
      physics::LinkPtr parentLink = this->model->GetChildLink(parentBone->GetName());
      if (this->printTransforms)
      {
        PrintTransform(parentLink->GetName(), currentLink->GetName(), transform);
      }
      auto parentPose = parentLink->WorldPose();
      ignition::math::Matrix4d parentTrans(parentPose);
      transform = parentTrans * transform;
    }

    msgs::Pose* link_pose = msg.add_pose();
    link_pose->set_name(currentLink->GetScopedName());
    link_pose->set_id(currentLink->GetId());
    ignition::math::Pose3d linkPose = transform.Pose() - mainLinkPose;
    link_pose->mutable_position()->CopyFrom(msgs::Convert(linkPose.Pos()));
    link_pose->mutable_orientation()->CopyFrom(msgs::Convert(linkPose.Rot()));
    currentLink->SetWorldPose(transform.Pose(), true, false);
  }

  msgs::Time* stamp = msg.add_time();
  stamp->CopyFrom(msgs::Convert(_time));

  if (this->bonePosePub && this->bonePosePub->HasConnections())
    this->bonePosePub->Publish(msg);

  std::string modelName = this->model->GetName();
  std::string modelLinkName = modelName + "_pose";
  this->model->GetLink(modelLinkName)->SetWorldPose(mainLinkPose, true, false);
}

void SkeletonModelVisual::AddSphereInertia(const sdf::ElementPtr& _linkSdf, const ignition::math::Pose3d& _pose,
                                           const double _mass, const double _radius)
{
  double ixx = 2.0 * _mass * _radius * _radius / 5.0;
  sdf::ElementPtr inertialSdf = _linkSdf->GetElement("inertial");
  sdf::ElementPtr inertialPoseSdf = inertialSdf->GetElement("pose");
  inertialPoseSdf->Set(_pose);
  inertialSdf->GetElement("mass")->Set(_mass);
  sdf::ElementPtr tensorSdf = inertialSdf->GetElement("inertia");
  tensorSdf->GetElement("ixx")->Set(ixx);
  tensorSdf->GetElement("ixy")->Set(0.00);
  tensorSdf->GetElement("ixz")->Set(0.00);
  tensorSdf->GetElement("iyy")->Set(ixx);
  tensorSdf->GetElement("iyz")->Set(0.00);
  tensorSdf->GetElement("izz")->Set(ixx);
}

void SkeletonModelVisual::AddSphereCollision(const sdf::ElementPtr& _linkSdf, const std::string& _name,
                                             const ignition::math::Pose3d& _pose, const double _radius)
{
  sdf::ElementPtr collisionSdf = _linkSdf->GetElement("collision");
  collisionSdf->GetAttribute("name")->Set(_name);
  sdf::ElementPtr collPoseSdf = collisionSdf->GetElement("pose");
  collPoseSdf->Set(_pose);
  sdf::ElementPtr geomColSdf = collisionSdf->GetElement("geometry");
  sdf::ElementPtr sphereColSdf = geomColSdf->GetElement("sphere");
  sphereColSdf->GetElement("radius")->Set(_radius);
}

void SkeletonModelVisual::AddSphereVisual(const sdf::ElementPtr& _linkSdf, const std::string& _name,
                                          const ignition::math::Pose3d& _pose, const double _radius,
                                          const std::string& _material, const ignition::math::Color& _ambient)
{
  sdf::ElementPtr visualSdf = _linkSdf->GetElement("visual");
  visualSdf->GetAttribute("name")->Set(_name);
  sdf::ElementPtr visualPoseSdf = visualSdf->GetElement("pose");
  visualPoseSdf->Set(_pose);
  sdf::ElementPtr geomVisSdf = visualSdf->GetElement("geometry");
  sdf::ElementPtr sphereVisSdf = geomVisSdf->GetElement("sphere");
  sphereVisSdf->GetElement("radius")->Set(_radius);
  sdf::ElementPtr matSdf = visualSdf->GetElement("material");
  matSdf->GetElement("script")->Set(_material);
  sdf::ElementPtr colorSdf = matSdf->GetElement("ambient");
  colorSdf->Set(_ambient);
}

void SkeletonModelVisual::AddBoxVisual(const sdf::ElementPtr& _linkSdf, const std::string& _name,
                                       const ignition::math::Pose3d& _pose, const ignition::math::Vector3d& _size,
                                       const std::string& _material, const ignition::math::Color& _ambient)
{
  sdf::ElementPtr visualSdf = _linkSdf->AddElement("visual");
  visualSdf->GetAttribute("name")->Set(_name);
  sdf::ElementPtr visualPoseSdf = visualSdf->GetElement("pose");
  visualPoseSdf->Set(_pose);
  sdf::ElementPtr geomVisSdf = visualSdf->GetElement("geometry");
  sdf::ElementPtr boxSdf = geomVisSdf->GetElement("box");
  boxSdf->GetElement("size")->Set(_size);
  sdf::ElementPtr matSdf = visualSdf->GetElement("material");
  matSdf->GetElement("script")->Set(_material);
  sdf::ElementPtr colorSdf = matSdf->GetElement("ambient");
  colorSdf->Set(_ambient);
}

void SkeletonModelVisual::AddBoxCollision(const sdf::ElementPtr& _linkSdf, const std::string& _name,
                                          const ignition::math::Pose3d& _pose, const ignition::math::Vector3d& _size)
{
  sdf::ElementPtr collisionSdf = _linkSdf->AddElement("collision");
  collisionSdf->GetAttribute("name")->Set(_name);
  sdf::ElementPtr collisionPoseSdf = collisionSdf->GetElement("pose");
  collisionPoseSdf->Set(_pose);
  sdf::ElementPtr geomCollSdf = collisionSdf->GetElement("geometry");
  sdf::ElementPtr boxSdf = geomCollSdf->GetElement("box");
  boxSdf->GetElement("size")->Set(_size);
}

void SkeletonModelVisual::AddActorVisual(const sdf::ElementPtr& _linkSdf, const std::string& _name,
                                         const ignition::math::Pose3d& _pose)
{
  if (this->skinFile.empty())
  {
    gzerr << "Can't add an actor visual without a skin file." << std::endl;
    return;
  }

  // Add visual
  sdf::ElementPtr visualSdf = _linkSdf->AddElement("visual");

  // Set name
  visualSdf->GetAttribute("name")->Set(_name);

  // Set pose
  sdf::ElementPtr visualPoseSdf = visualSdf->GetElement("pose");
  visualPoseSdf->Set(_pose);

  // Set mesh geometry (skin file)
  sdf::ElementPtr geomVisSdf = visualSdf->GetElement("geometry");
  sdf::ElementPtr meshSdf = geomVisSdf->GetElement("mesh");
  meshSdf->GetElement("uri")->Set(this->skinFile);
  meshSdf->GetElement("scale")->Set(ignition::math::Vector3d(this->skinScale, this->skinScale, this->skinScale));
}

}  // namespace gazebo