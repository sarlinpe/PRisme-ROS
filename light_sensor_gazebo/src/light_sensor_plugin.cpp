#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include "light_sensor_plugin.h"

#include "gazebo_plugins/gazebo_ros_camera.h"

#include <string>

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>

#include <sensor_msgs/Illuminance.h>

namespace gazebo
{
  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(GazeboRosLight)

  ////////////////////////////////////////////////////////////////////////////////
  // Constructor
  GazeboRosLight::GazeboRosLight():
	_fov(6),
	_robotNamespace("/"),
	_illuminanceTopic("light_sensor"),
	_frameName("")
  {}

  ////////////////////////////////////////////////////////////////////////////////
  // Destructor
  GazeboRosLight::~GazeboRosLight()
  {
		this->parentSensor_->SetActive(false);
		this->_nh->shutdown();
		delete this->_nh;
    ROS_DEBUG_STREAM_NAMED("camera","Unloaded");
  }

  void GazeboRosLight::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
  {
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
		}

		CameraPlugin::Load(_parent, _sdf);
		// copying from CameraPlugin into GazeboRosCameraUtils
		this->parentSensor_ = this->parentSensor;
		this->width_ = this->width;
		this->height_ = this->height;
		this->depth_ = this->depth;
		this->format_ = this->format;
		this->camera_ = this->camera;

		// only keep the useful lines of GazeboRosCameraUtils::Load
		std::string world_name = _parent->GetWorldName();
		this->world_ = physics::get_world(world_name);

		// extract values from sdf
		if (!_sdf->HasElement("updateRate"))
		{
		 	ROS_DEBUG("Camera plugin missing <updateRate>, defaults to unlimited (0).");
		 	this->update_rate_ = 0;
			this->update_period_ = 0.0;
			this->parentSensor_->SetUpdateRate(0);
		}
		else
		{
		 	this->update_rate_ = _sdf->Get<double>("updateRate");
			this->update_period_ = 1.0/this->update_rate_;
			this->parentSensor_->SetUpdateRate(this->update_rate_);
		}

		if (_sdf->HasElement("fov")) // maybe useless ?
			this->_fov = _sdf->Get<unsigned>("fov");

		if (_sdf->HasElement("robotNamespace"))
			this->_robotNamespace = _sdf->Get<std::string>("robotNamespace");
		this->_nh = new ros::NodeHandle(this->_robotNamespace);

		if (_sdf->HasElement("illuminanceTopicName"))
			this->_illuminanceTopic = _sdf->Get<std::string>("illuminanceTopicName");
		_sensorPublisher = this->_nh->advertise<sensor_msgs::Illuminance>(this->_illuminanceTopic, 1);

		if (_sdf->HasElement("frameName"))
			this->_frameName = _sdf->Get<std::string>("frameName");

		this->parentSensor->SetActive(true);
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Update the controller
  void GazeboRosLight::OnNewFrame(const unsigned char *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
  {
    static int seq=0;
    this->sensor_update_time_ = this->parentSensor_->GetLastUpdateTime();

		common::Time cur_time = this->world_->GetSimTime();
    if (cur_time - this->last_update_time_ >= this->update_period_)
    {
      sensor_msgs::Illuminance msg;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = this->_frameName;
      msg.header.seq = seq;

      int startingPix = _width * ( (int)(_height/2) - (int)( _fov/2)) - (int)(_fov/2);

      double illum = 0;
      for (int i=0; i<_fov ; ++i)
      {
        int index = startingPix + i*_width;
        for (int j=0; j<_fov ; ++j)
          illum += _image[index+j];
      }

      msg.illuminance = illum/(_fov*_fov);
      msg.variance = 0.0;

      _sensorPublisher.publish(msg);

      seq++;
      this->last_update_time_ = cur_time;
    }
  }
}


