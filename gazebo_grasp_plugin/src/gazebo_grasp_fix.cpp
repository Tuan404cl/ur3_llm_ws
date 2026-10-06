#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <string>
#include <vector>
#include <set>
#include <map>
#include <mutex>
#include <algorithm>

namespace gazebo
{

class GazeboGraspFix : public ModelPlugin
{
public:
  GazeboGraspFix() : ModelPlugin() {}
  virtual ~GazeboGraspFix()
  {
    this->updateConnection.reset();
    if (this->contactSub)
    {
      this->contactSub.reset();
    }
    if (this->node)
    {
      this->node->Fini();
      this->node.reset();
    }
  }

  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override
  {
    this->model = _parent;
    this->world = this->model->GetWorld();

    // 1. Parse parameters
    this->armName = "ur3_gripper";
    this->palmLinkName = "wrist_3_link";
    this->gripCountThreshold = 2;

    if (_sdf->HasElement("grip_count_threshold"))
      this->gripCountThreshold = _sdf->Get<int>("grip_count_threshold");

    // Look for <arm> element or direct elements
    sdf::ElementPtr armElem = _sdf->HasElement("arm") ? _sdf->GetElement("arm") : _sdf;

    if (armElem->HasElement("arm_name"))
      this->armName = armElem->Get<std::string>("arm_name");
    else if (_sdf->HasElement("arm_name"))
      this->armName = _sdf->Get<std::string>("arm_name");

    if (armElem->HasElement("palm_link"))
      this->palmLinkName = armElem->Get<std::string>("palm_link");
    else if (_sdf->HasElement("palm_link"))
      this->palmLinkName = _sdf->Get<std::string>("palm_link");

    this->palmLink = this->model->GetLink(this->palmLinkName);
    if (!this->palmLink)
    {
      gzerr << "[GazeboGraspFix] Palm link [" << this->palmLinkName << "] not found in model!\n";
    }
    else
    {
      gzmsg << "[GazeboGraspFix] Using palm link: " << this->palmLinkName << "\n";
    }

    // Collect all gripper_link tags (from both <arm> children and direct <plugin> children)
    std::set<std::string> linkNames;
    for (sdf::ElementPtr curArm = _sdf->GetElement("arm"); curArm != nullptr; curArm = curArm->GetNextElement("arm"))
    {
      for (sdf::ElementPtr elem = curArm->GetElement("gripper_link"); elem != nullptr; elem = elem->GetNextElement("gripper_link"))
      {
        linkNames.insert(elem->Get<std::string>());
      }
    }
    for (sdf::ElementPtr elem = _sdf->GetElement("gripper_link"); elem != nullptr; elem = elem->GetNextElement("gripper_link"))
    {
      linkNames.insert(elem->Get<std::string>());
    }

    // Register links and their collision entities
    for (const auto &lName : linkNames)
    {
      physics::LinkPtr link = this->model->GetLink(lName);
      if (link)
      {
        this->gripperLinks.push_back(link);
        gzmsg << "[GazeboGraspFix] Registered gripper link: " << lName << "\n";
        for (unsigned int c = 0; c < link->GetCollisions().size(); ++c)
        {
          physics::CollisionPtr coll = link->GetCollisions()[c];
          if (coll)
          {
            std::string cName = coll->GetScopedName();
            this->monitoredCollisions.insert(cName);
            this->collisionNamesList.push_back(cName);
            gzmsg << "[GazeboGraspFix]   Monitoring collision: " << cName << "\n";
          }
        }
      }
      else
      {
        gzerr << "[GazeboGraspFix] Gripper link [" << lName << "] not found in model!\n";
      }
    }

    // Find gripper joint for release detection
    this->knuckleJoint = this->model->GetJoint("robotiq_85_left_knuckle_joint");
    if (!this->knuckleJoint)
    {
      // Fallback search
      for (auto &j : this->model->GetJoints())
      {
        if (j->GetName().find("knuckle") != std::string::npos)
        {
          this->knuckleJoint = j;
          break;
        }
      }
    }

    // 2. Setup ContactManager and Transport filter
    this->contactManager = this->world->Physics()->GetContactManager();
    if (this->contactManager)
    {
      this->contactManager->SetNeverDropContacts(true);
      this->contactManager->PublishContacts();

      if (!this->collisionNamesList.empty())
      {
        std::string filterTopic = this->contactManager->CreateFilter(
            this->model->GetScopedName() + "_grasp_filter", this->collisionNamesList);
        this->node = transport::NodePtr(new transport::Node());
        this->node->Init(this->world->Name());
        this->contactSub = this->node->Subscribe(filterTopic, &GazeboGraspFix::OnContact, this);
        gzmsg << "[GazeboGraspFix] Subscribed to contact filter topic: " << filterTopic << "\n";
      }
    }

    // 3. Setup ROS 2 node & publishers
    try
    {
      this->rosNode = gazebo_ros::Node::Get(_sdf);
      this->graspPub = this->rosNode->create_publisher<std_msgs::msg::Bool>("/ur3_gripper/grasping", 10);
      this->vacuumPub = this->rosNode->create_publisher<std_msgs::msg::Bool>("/ur3_gripper/vacuum_grasping", 10);
      gzmsg << "[GazeboGraspFix] ROS 2 publishers initialized on /ur3_gripper/grasping and /ur3_gripper/vacuum_grasping\n";
    }
    catch (const std::exception &e)
    {
      gzerr << "[GazeboGraspFix] Failed to initialize ROS 2 node: " << e.what() << "\n";
    }

    // 4. Hook into world update loop
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&GazeboGraspFix::OnUpdate, this));

    gzmsg << "[GazeboGraspFix] Plugin loaded successfully for arm [" << this->armName << "].\n";
  }

  void OnContact(ConstContactsPtr &_msg)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    common::Time now = this->world->SimTime();

    for (int i = 0; i < _msg->contact_size(); ++i)
    {
      const gazebo::msgs::Contact &contact = _msg->contact(i);
      std::string c1 = contact.collision1();
      std::string c2 = contact.collision2();

      std::string gripperColl = "";
      std::string otherColl = "";

      if (this->monitoredCollisions.count(c1) && !this->monitoredCollisions.count(c2))
      {
        gripperColl = c1;
        otherColl = c2;
      }
      else if (this->monitoredCollisions.count(c2) && !this->monitoredCollisions.count(c1))
      {
        gripperColl = c2;
        otherColl = c1;
      }
      else
      {
        continue;
      }

      size_t delim = otherColl.find("::");
      if (delim == std::string::npos)
        continue;

      std::string modelName = otherColl.substr(0, delim);
      if (modelName == this->model->GetName())
        continue;

      physics::ModelPtr targetModel = this->world->ModelByName(modelName);
      if (!targetModel || targetModel->IsStatic())
        continue;

      bool isLeft = (gripperColl.find("left") != std::string::npos);
      bool isRight = (gripperColl.find("right") != std::string::npos);

      if (isLeft)
        this->lastLeftContactTime[modelName] = now;
      if (isRight)
        this->lastRightContactTime[modelName] = now;
    }
  }

  void OnUpdate()
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    common::Time now = this->world->SimTime();

    // Direct ContactManager query as backup to ensure zero missed contacts
    if (this->contactManager)
    {
      unsigned int cCount = this->contactManager->GetContactCount();
      const std::vector<physics::Contact *> &contacts = this->contactManager->GetContacts();
      for (unsigned int i = 0; i < cCount && i < contacts.size(); ++i)
      {
        physics::Contact *c = contacts[i];
        if (!c || !c->collision1 || !c->collision2)
          continue;

        std::string c1 = c->collision1->GetScopedName();
        std::string c2 = c->collision2->GetScopedName();

        std::string gripperColl = "";
        physics::LinkPtr otherLink = nullptr;

        if (this->monitoredCollisions.count(c1) && !this->monitoredCollisions.count(c2))
        {
          gripperColl = c1;
          otherLink = c->collision2->GetLink();
        }
        else if (this->monitoredCollisions.count(c2) && !this->monitoredCollisions.count(c1))
        {
          gripperColl = c2;
          otherLink = c->collision1->GetLink();
        }
        else
        {
          continue;
        }

        if (!otherLink || !otherLink->GetModel() || otherLink->GetModel()->IsStatic())
          continue;

        std::string mName = otherLink->GetModel()->GetName();
        if (mName == this->model->GetName())
          continue;

        if (gripperColl.find("left") != std::string::npos)
          this->lastLeftContactTime[mName] = now;
        if (gripperColl.find("right") != std::string::npos)
          this->lastRightContactTime[mName] = now;
      }
    }

    // Check which movable model has active contact with BOTH left and right fingers
    std::string candidateModelName = "";
    physics::ModelPtr candidateModel = nullptr;

    for (const auto &pair : this->lastLeftContactTime)
    {
      const std::string &mName = pair.first;
      double leftDt = (now - pair.second).Double();
      double rightDt = 999.0;
      if (this->lastRightContactTime.count(mName))
      {
        rightDt = (now - this->lastRightContactTime[mName]).Double();
      }

      // Contact is fresh if within 100 ms
      if (leftDt < 0.10 && rightDt < 0.10)
      {
        physics::ModelPtr m = this->world->ModelByName(mName);
        if (m && !m->IsStatic())
        {
          candidateModelName = mName;
          candidateModel = m;
          break;
        }
      }
    }

    // Get knuckle joint angle
    double knuckleAngle = 0.0;
    if (this->knuckleJoint)
    {
      knuckleAngle = this->knuckleJoint->Position(0);
    }

    // Logic: Attach (Grasp)
    if (!this->fixedJoint && candidateModel)
    {
      this->gripCounter++;
      if (this->gripCounter >= this->gripCountThreshold)
      {
        physics::LinkPtr targetLink = candidateModel->GetLink();
        if (!targetLink)
        {
          auto links = candidateModel->GetLinks();
          if (!links.empty())
            targetLink = links[0];
        }

        if (targetLink && this->palmLink)
        {
          gzmsg << "[GazeboGraspFix] Grasping object [" << candidateModelName 
                << "] (knuckle joint: " << knuckleAngle << " rad)\n";

          this->fixedJoint = this->world->Physics()->CreateJoint("fixed", this->model);
          this->fixedJoint->SetName(this->model->GetName() + "_grasp_joint");
          this->fixedJoint->Attach(this->palmLink, targetLink);
          this->fixedJoint->Load(this->palmLink, targetLink, ignition::math::Pose3d::Zero);
          this->fixedJoint->Init();

          this->graspedModelName = candidateModelName;
          this->graspedLink = targetLink;
          this->releaseCounter = 0;
          this->PublishGraspState(true);
        }
      }
    }
    // Logic: Detach (Release)
    else if (this->fixedJoint && this->graspedLink)
    {
      // Gripper opened (knuckle angle < 0.15 rad when commanded to 0.0)
      bool gripperOpened = (this->knuckleJoint && knuckleAngle < 0.15);

      double leftDt = 999.0;
      double rightDt = 999.0;
      if (this->lastLeftContactTime.count(this->graspedModelName))
        leftDt = (now - this->lastLeftContactTime[this->graspedModelName]).Double();
      if (this->lastRightContactTime.count(this->graspedModelName))
        rightDt = (now - this->lastRightContactTime[this->graspedModelName]).Double();

      bool contactsLost = (leftDt > 0.15 && rightDt > 0.15);

      if (gripperOpened || contactsLost)
      {
        this->releaseCounter++;
        // If commanded open, release immediately; if contact lost, release after 15 updates (~0.3s)
        if (gripperOpened || this->releaseCounter > 15)
        {
          gzmsg << "[GazeboGraspFix] Releasing object [" << this->graspedModelName 
                << "] (gripperOpened=" << gripperOpened << ", knuckle=" << knuckleAngle << " rad)\n";

          this->fixedJoint->Detach();
          this->fixedJoint.reset();
          this->graspedLink = nullptr;
          this->graspedModelName = "";
          this->gripCounter = 0;
          this->releaseCounter = 0;
          this->PublishGraspState(false);
        }
      }
      else
      {
        this->releaseCounter = 0;
      }
    }
  }

  void PublishGraspState(bool state)
  {
    if (this->graspPub)
    {
      std_msgs::msg::Bool msg;
      msg.data = state;
      this->graspPub->publish(msg);
    }
    if (this->vacuumPub)
    {
      std_msgs::msg::Bool msg;
      msg.data = state;
      this->vacuumPub->publish(msg);
    }
  }

private:
  physics::ModelPtr model;
  physics::WorldPtr world;
  physics::LinkPtr palmLink;
  physics::LinkPtr graspedLink;
  physics::JointPtr fixedJoint;
  physics::JointPtr knuckleJoint;
  physics::ContactManager *contactManager{nullptr};

  std::vector<physics::LinkPtr> gripperLinks;
  std::set<std::string> monitoredCollisions;
  std::vector<std::string> collisionNamesList;

  std::string armName;
  std::string palmLinkName;
  std::string graspedModelName{""};

  std::map<std::string, common::Time> lastLeftContactTime;
  std::map<std::string, common::Time> lastRightContactTime;

  int gripCountThreshold{2};
  int gripCounter{0};
  int releaseCounter{0};

  transport::NodePtr node;
  transport::SubscriberPtr contactSub;
  event::ConnectionPtr updateConnection;
  std::mutex mutex;

  gazebo_ros::Node::SharedPtr rosNode;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr graspPub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vacuumPub;
};

GZ_REGISTER_MODEL_PLUGIN(GazeboGraspFix)

} // namespace gazebo
