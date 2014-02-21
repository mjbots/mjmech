// Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

#include <iostream>

#include <boost/bind.hpp>

#include <gazebo/gazebo.hh>

namespace gazebo {
class ServoPlugin : public ModelPlugin {
 public:
  ServoPlugin() : ModelPlugin() {
    std::cout << "model plugin created!\n";
  }

  virtual void Load(physics::ModelPtr parent, sdf::ElementPtr) {
    model_ = parent;

    std::cout << "loaded model!\n";
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&ServoPlugin::OnUpdate, this, _1));
  }

  virtual void OnUpdate(const common::UpdateInfo& update_info) {
  }

 private:
  physics::ModelPtr model_;
  event::ConnectionPtr update_connection_;
};

GZ_REGISTER_MODEL_PLUGIN(ServoPlugin);

}
