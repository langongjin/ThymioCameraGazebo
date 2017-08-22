#include <iostream>
#include <fstream>
#include <boost/bind.hpp>
#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"

namespace gazebo
{
    class CameraMove : public ModelPlugin
    {
    public: CameraMove() : ModelPlugin() {}

    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            // Get a pointer to the model
            this->model = _model;

            // Listen to the update event. This event is broadcast every simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    boost::bind(&CameraMove::OnUpdate, this));
        }

        // Called by the world update start event
    public: void OnUpdate()
        {
            //usleep(200000); //200ms
            int d;
            std::ifstream data;
            data.open("/Users/lan/.gazebo/plugins/cameraSingle_plugin/data.txt");
            data >> d ;
            data.close();

            if (d == 2000)
            {   // no target, searching
                std::cout << "d = " << d << " ---searching--- " << std::endl;
                this->model->GetJoint("left_wheel_hinge")->SetVelocity(0,0);
                this->model->GetJoint("right_wheel_hinge")->SetVelocity(0,0.1);
            }
            else
            {
                if (d == 1000)
                {   // stop
                    std::cout << "d = " << d << " ---stop--- " << std::endl;
                    this->model->GetJoint("left_wheel_hinge")->SetVelocity(0,0);
                    this->model->GetJoint("right_wheel_hinge")->SetVelocity(0,0);
                }
                else
                {
                    if (d ==0)
                    {   // go straight forward
                        std::cout << "d = " << d << " ---go straight forward--- " << std::endl;
                        this->model->GetJoint("left_wheel_hinge")->SetVelocity(0,0.2);
                        this->model->GetJoint("right_wheel_hinge")->SetVelocity(0,0.2);
                    }
                    if (d > 0)
                    {   // turn left
                        std::cout << "d = " << d << " ---turn left--- " << std::endl;
                        this->model->GetJoint("left_wheel_hinge")->SetVelocity(0,0.1);
                        this->model->GetJoint("right_wheel_hinge")->SetVelocity(0,0.2);
                    }
                    if (d < 0)
                    {   // turn right
                        std::cout << "d = " << d << " ---turn right--- " << std::endl;
                        this->model->GetJoint("left_wheel_hinge")->SetVelocity(0,0.2);
                        this->model->GetJoint("right_wheel_hinge")->SetVelocity(0,0.1);
                    }
                }
            }
        }

        // Pointer to the model
    private: physics::ModelPtr model;

        // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(CameraMove)
}