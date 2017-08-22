#include <boost/bind.hpp>
#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"

namespace gazebo
{
    class robRanVel : public ModelPlugin
    {
    public: robRanVel() : ModelPlugin() {}

    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            // Get a pointer to the model
            this->model = _model;

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    boost::bind(&robRanVel::OnUpdate, this));
        }

        // Called by the world update start event
    public: void OnUpdate()
        {
            math::Vector3 v(-0.01, 0.02, 0);
            math::Vector3 a(0, 0, 0);
            // Apply a small linear velocity to the model.
            this->model->SetLinearVel(v);
            this->model->SetAngularVel(a);

        }

        // Pointer to the model
    private: physics::ModelPtr model;

        // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(robRanVel)
}