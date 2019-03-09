#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>


int main(int _argc, char **_argv)
{
  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Publish to the  velodyne topic
  gazebo::transport::PublisherPtr pub =
    node->Advertise<gazebo::msgs::Vector3d>("/hitch_force");

  // Wait for a subscriber to connect to this publisher
  pub->WaitForConnection();

  // Create a a vector3 message
  gazebo::msgs::Vector3d msg;

  // Send the message
  pub->Publish(msg);
}
