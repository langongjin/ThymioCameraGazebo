#include "gazebo/gazebo.hh"
#include "gazebo/plugins/CameraPlugin.hh"

namespace gazebo
{
    class CameraDump : public CameraPlugin
    {
    public: CameraDump() : CameraPlugin() {}

    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
        {
            // Don't forget to load the camera plugin
            CameraPlugin::Load(_parent, _sdf);
        }

        // Update the controller
    public: void OnNewFrame(const unsigned char *_image,
                            unsigned int _width, unsigned int _height, unsigned int _depth,
                            const std::string &_format)
        {
            this->parentSensor->GetCamera()->SaveFrame(_image, _width, _height, _depth, _format, "/Users/lan/.gazebo/plugins/cameraSingle_plugin/images/updating.jpg");
            //sleep(1);
            usleep(200000);     //2fps
        }

    };

    // Register this plugin with the simulator
    GZ_REGISTER_SENSOR_PLUGIN(CameraDump)
}