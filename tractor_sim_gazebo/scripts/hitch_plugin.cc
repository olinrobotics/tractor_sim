//http://gazebosim.org/tutorials?tut=guided_i5
#ifndef _HITCH_PLUGIN_HH_
#define _HITCH_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

namespace gazebo
{
  class HitchPlugin : public ModelPlugin
  {
    public: HitchPlugin() {}

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      if (_model->GetJointCount() == 0)
      {
        std::cerr << "Invalid joint count, hitch plugin not loaded\n";
        return;
      }
      std::cerr << "\nThe hitch plugin is attached to model[" <<
              _model->GetName() << "]\n";

      this->model = _model;
      this->joint = model->GetJoint("base_link_to_hitch");

      // // Apply the P-controller to the joint.
      // this->model->GetJointController()->SetVelocityPID(
      //     this->joint->GetScopedName(), this->pid);
      //
      // // Set the joint's target velocity. This target velocity is just
      // // for demonstration purposes.
      // this->model->GetJointController()->SetVelocityTarget(
      //     this->joint->GetScopedName(), 100.0);

      // Create the node
      this->node = transport::NodePtr(new transport::Node());
      #if GAZEBO_MAJOR_VERSION < 8
      this->node->Init(this->model->GetWorld()->GetName());
      #else
      this->node->Init(this->model->GetWorld()->Name());
      #endif

      // Create a topic name
      std::string topicName = "/hitch_force";

      // Subscribe to the topic, and register a callback
      this->sub = this->node->Subscribe(topicName,
         &HitchPlugin::OnMsg, this);


     // Initialize ros, if it has not already bee initialized.
     if (!ros::isInitialized())
     {
       int argc = 0;
       char **argv = NULL;
       ros::init(argc, argv, "gazebo_client",
           ros::init_options::NoSigintHandler);
     }

     // Create our ROS node. This acts in a similar manner to
     // the Gazebo node
     this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

     // Create a named topic, and subscribe to it.
     ros::SubscribeOptions so =
       ros::SubscribeOptions::create<std_msgs::Float32>(
           "/hitch_force",
           1,
           boost::bind(&HitchPlugin::OnRosMsg, this, _1),
           ros::VoidPtr(), &this->rosQueue);
     this->rosSub = this->rosNode->subscribe(so);

     // Spin up the queue helper thread.
     this->rosQueueThread =
       std::thread(std::bind(&HitchPlugin::QueueThread, this));
    }

    /// \brief Handle an incoming message from ROS
    /// \param[in] _msg A float value that is used to set the velocity
    /// of the Velodyne.
    public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
    {
      this->SetHitchForce(_msg->data);
    }

    /// \brief ROS helper function that processes messages
    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

    public: void SetHitchForce(const double &_vel)
    {
      this->joint->SetForce(0,_vel); // axis 0, 10000 force
    }

    /// \brief Handle incoming message
    /// \param[in] _msg Repurpose a vector3 message. This function will
    /// only use the x component.
    private: void OnMsg(ConstVector3dPtr &_msg)
    {
      this->SetHitchForce(_msg->x());
    }

    /// \brief Pointer to the model.
    private: physics::ModelPtr model;

    /// \brief Pointer to the joint.
    private: physics::JointPtr joint;

    /// \brief A PID controller for the joint.
    private: common::PID pid;

    /// \brief A node used for transport
    private: transport::NodePtr node;

    /// \brief A subscriber to a named topic.
    private: transport::SubscriberPtr sub;

    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub;

    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;

    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;

  };
  GZ_REGISTER_MODEL_PLUGIN(HitchPlugin)
}

#endif
