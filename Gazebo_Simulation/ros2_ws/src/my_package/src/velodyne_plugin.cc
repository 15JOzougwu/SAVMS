#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/sensors/GpuRaySensor.hh>
#include <gazebo/transport/Node.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <functional>
#include <iostream>

namespace gazebo
{
  
  /// \brief A plugin to control a Velodyne sensor.
  class VelodynePlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: VelodynePlugin() : ModelPlugin() {}
    
    private: event::ConnectionPtr updateConnection;

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
    
       //this->link = this->model->GetLink("top");
    
    	// Safety check
    	//if (_model->GetJointCount() == 0)
    	//{
    	//  std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
    	//  return;
    	//}
    
    	// Store the model pointer for convenience.
    	this->model = _model;
    	
    	this->joint = _model->GetJoints()[0];
    	
    	this->pid = common::PID(0.1, 0, 0);
    	
    	this->node = transport::NodePtr(new transport::Node());
        this->node->Init();
      
        std::string rayTopic = "~/my_model/top/sensor/scan";
                                           
       this->raySub = this->node->Subscribe(rayTopic, &VelodynePlugin::OnUpdate, this);
      
      std::cout << "GpuRayPlugin loaded and subscribed to " << rayTopic << std::endl;

      // Subscribe to the contact sensor topic
    	
    	//this->sensor = this->model->GetSensor("sensor");
    	
    	//sensors::SensorManager* sensorManager = sensors::SensorManager::Instance();
    	
    	 //std::string sensorName = this->model->GetName() + "::sensor";

      // Get the Velodyne sensor using the sensor name
       //this->sensor = std::dynamic_pointer_cast<sensors::GpuRaySensor>(
         // sensorManager->GetSensor(sensorName));
          
       //if (!sensor)
       //{
      //    gzerr << "VelodyneHDL32Plugin: GpuRaySensor with name '" << sensorName << "' not found." << std::endl;
        //  return;
       //}
    	
    	//this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          //std::bind(&VelodynePlugin::SensorLoad, this));
    	
    	this->model->GetJointController()->SetVelocityPID(
    	  this->joint->GetScopedName(), this->pid);
    	  
    	this->model->GetJointController()->SetVelocityTarget(
    	  this->joint->GetScopedName(), 10.0);
    	
    	//this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          //std::bind(&VelodynePlugin::OnUpdate, this));
         
         //OnUpdate();
    	
    	//Connect to the sensor update event
        
        //const float *ranges = this->sensor->RangeCount();
        
        // Initialize Gazebo transport


      //std::cout << "ContactPlugin loaded and subscribed to contact topics." << std::endl;
    }

    // Called on sensor update
    void OnUpdate(ConstLaserScanStampedPtr &msg)
    {
      //const float *ranges = _sensor->LaserShape()->Ranges();
      //int numPoints = sensor->RangeCount() * sensor->VerticalRangeCount();

      //for (int i = 0; i < numPoints; ++i)
      //{
        //double distance = ranges[i];
        //std::cout << "Point[" << i << "]: Distance: " << distance << std::endl;
      //}
      //std::cout << "Working\n\n";
      
      for (int i = 0; i < msg->scan().ranges_size(); ++i)
      {
        //const double ranges = msg->ranges(i);
        //std::cout << "Max Dist: " << msg->scan().MaxRange() << "\n";
        std::cout << "Range: " << msg->scan().ranges(i) << " m" << std::endl;
      }
      
      std::cout << "\n\n";
      
        //std::cout << "\n\nRange[" << i << "]: " << msg->scan().ranges(i) << std::endl;
    }
    
    
    //public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
    //{
    //	this->sensor = std::dynamic_pointer_cast<sensors::GpuRaySensor>(_sensor);
    	
    //	this->updateConnection = this->sensor->ConnectUpdated(
    //      std::bind(&VelodynePlugin::OnUpdate, this));
    //}
    
    //public: void EndConnection()
    //{
    //   std::cout << "Paused";
    //}
    
    private: physics::ModelPtr model;
    
    private: physics::JointPtr joint;
    
    private: physics::LinkPtr link;
    
    private: common::PID pid;
    
    private: sensors::GpuRaySensorPtr sensor;
    
    transport::NodePtr node;
    transport::SubscriberPtr raySub;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  //GZ_REGISTER_SENSOR_PLUGIN(LaserPlugin);
  GZ_REGISTER_MODEL_PLUGIN(VelodynePlugin)
}
#endif
