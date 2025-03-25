#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/gazebo.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/sensors/GpuRaySensor.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <functional>
#include <iostream>
#include <string>
#include <cmath>
#include <Eigen/Dense>
#include <chrono>
#include <ctime>

namespace gazebo
{
    class ModelPush : public ModelPlugin
    {
    
    #define PI 3.1415926535897932384626433832795028841971
    
    float front_right = 0;
    float front_left = 0;
    float back_right = 0;
    float back_left = 0;
    std::string mode = "AUTO";
    int sector = 0;
    float default_speed = 0;
    int start = 0;
    int forward = 0;
    int backward = 0;
    int clockwise = 0; // right
    int anticlockwise = 0; // left
    float velocity;
    float refined_angle;
    
    int vertical_samples;
    int h_resolution;
    int v_resolution;
    
    private: ignition::math::Quaterniond modelOrientation_sensor;
    private: ignition::math::Quaterniond modelOrientation_vehicle;
    private: event::ConnectionPtr updateConnection;
    private: ignition::math::Vector3d eulerAngles_sensor;
    private: ignition::math::Vector3d eulerAngles_vehicle;
    public:
        ModelPush() {}

        // Load function (called when the plugin is loaded into Gazebo)
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
        {
            // Initialize ROS 2 if it hasn't been initialized yet
            if (!rclcpp::ok()) {
                rclcpp::init(0, nullptr);
            }

            // Create a ROS 2 node
            ros_node_ = rclcpp::Node::make_shared("model_push_node");
            
        Shutdown_subscription = ros_node_->create_subscription<std_msgs::msg::Int32>(
            "control/safety/emergency_shutdown", 10, std::bind(&ModelPush::Shutdown_callback, this, std::placeholders::_1));
            
        StartStop_subscription = ros_node_->create_subscription<std_msgs::msg::Int32>(
            "control/navigation/start_stop", 10, std::bind(&ModelPush::StartStop_callback, this, std::placeholders::_1));
            
         Mode_subscription = ros_node_->create_subscription<std_msgs::msg::Int32>(
            "control/mode_control", 10, std::bind(&ModelPush::Mode_callback, this, std::placeholders::_1));
            
         Forward_subscription = ros_node_->create_subscription<std_msgs::msg::Int32>(
            "control/navigation/move_forward", 10, std::bind(&ModelPush::Forward_callback, this, std::placeholders::_1));

         Backward_subscription = ros_node_->create_subscription<std_msgs::msg::Int32>(
            "control/navigation/move_backward", 10, std::bind(&ModelPush::Backward_callback, this, std::placeholders::_1));
            
         Velocity_subscription = ros_node_->create_subscription<std_msgs::msg::Float64>(
            "control/navigation/vel_control", 10, std::bind(&ModelPush::Velocity_callback, this, std::placeholders::_1));
            
         TurnLeft_subscription = ros_node_->create_subscription<std_msgs::msg::Int32>(
            "control/navigation/turn_left", 10, std::bind(&ModelPush::TurnLeft_callback, this, std::placeholders::_1));
            
         TurnRight_subscription = ros_node_->create_subscription<std_msgs::msg::Int32>(
            "control/navigation/turn_right", 10, std::bind(&ModelPush::TurnRight_callback, this, std::placeholders::_1));

            // Create the ROS 2 publisher to publish velocity
            posex_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("vehicle_status/navigation/x_position", 10);

            posey_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("vehicle_status/navigation/y_position", 10);

            MinRange_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/diagnostics/min_range", 10);

            MaxRange_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/diagnostics/max_range", 10);

            AngleRotation_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("vehicle_status/navigation/angle", 10);
            
            Velocity_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("vehicle_status/navigation/velocity", 10);
            
            HSamples_publisher = ros_node_->create_publisher<std_msgs::msg::Int32>("sensors/Velodyne/diagnostics/hsamples", 10);
            
            VSamples_publisher = ros_node_->create_publisher<std_msgs::msg::Int32>("sensors/Velodyne/diagnostics/vsamples", 10);
            
            HResolution_publisher = ros_node_->create_publisher<std_msgs::msg::Int32>("sensors/Velodyne/diagnostics/hresolution", 10);
            
            VResolution_publisher = ros_node_->create_publisher<std_msgs::msg::Int32>("sensors/Velodyne/diagnostics/vresolution", 10);
            
            Distance1_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/distance1", 10);
            
            Distance2_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/distance2", 10);
            
            Distance3_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/distance3", 10);
            
            Distance4_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/distance4", 10);
            
            Distance5_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/distance5", 10);
            
            Distance6_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/distance6", 10);
            
            Distance7_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/distance7", 10);
            
            Distance8_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/distance8", 10);
            
            Distance9_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/distance9", 10);
            
            Distance10_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/distance10", 10);
            
            Distance11_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/distance11", 10);
            
            Distance12_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/distance12", 10);
            
            Distance13_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/distance13", 10);
            
            Distance14_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/distance14", 10);
            
            Distance15_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/distance15", 10);
            
            Distance16_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/distance16", 10);
            
            Distance17_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/distance17", 10);
            
            Distance18_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/distance18", 10);
            
            Distance19_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/distance19", 10);
            
            Distance20_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/distance20", 10);
            
            Distance21_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/distance21", 10);
            
            Distance22_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/distance22", 10);
            
            Distance23_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/distance23", 10);
            
            Distance24_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/distance24", 10);
            
            Distance25_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/distance25", 10);
            
            Distance26_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/distance26", 10);
            
            Distance27_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/distance27", 10);
            
            Distance28_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/distance28", 10);
            
            Distance29_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/distance29", 10);
            
            Distance30_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/distance30", 10);
            
            Distance31_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/distance31", 10);
            
            Distance32_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/distance32", 10);
            
            Angle1_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/angle1", 10);
            
            Angle2_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/angle2", 10);
            
            Angle3_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/angle3", 10);
            
            Angle4_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/angle4", 10);
            
            Angle5_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/angle5", 10);
            
            Angle6_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/angle6", 10);
            
            Angle7_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/angle7", 10);
            
            Angle8_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/angle8", 10);
            
            Angle9_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/angle9", 10);
            
            Angle10_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/angle10", 10);
            
            Angle11_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/angle11", 10);
            
            Angle12_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/angle12", 10);
            
            Angle13_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/angle13", 10);
            
            Angle14_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/angle14", 10);
            
            Angle15_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/angle15", 10);
            
            Angle16_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/angle16", 10);
            
            Angle17_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/angle17", 10);
            
            Angle18_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/angle18", 10);
            
            Angle19_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/angle19", 10);
            
            Angle20_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/angle20", 10);
            
            Angle21_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/angle21", 10);
            
            Angle22_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/angle22", 10);
            
            Angle23_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/angle23", 10);
            
            Angle24_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/angle24", 10);
            
            Angle25_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/angle25", 10);
            
            Angle26_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/angle26", 10);
            
            Angle27_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/angle27", 10);
            
            Angle28_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/angle28", 10);
            
            Angle29_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/angle29", 10);
            
            Angle30_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/angle30", 10);
            
            Angle31_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/angle31", 10);
            
            Angle32_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("sensors/Velodyne/navigation/angle32", 10);

       // Store the model pointer for further use
       this->model_ = _model;
       
       this->joint = _model->GetJoints()[0];
       
       this->pid = common::PID(0.1, 0, 0);
       
       this->node = transport::NodePtr(new transport::Node());
       this->node->Init();

        std::string rayTopic = "~/my_model/top/sensor/scan";
                                           
        this->raySub = this->node->Subscribe(rayTopic, &ModelPush::Update, this);
            
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelPush::OnUpdate, this));
          
         this->model_->GetJointController()->SetVelocityPID(
    	  this->joint->GetScopedName(), this->pid);
    	  
    	this->model_->GetJointController()->SetVelocityTarget(
    	  this->joint->GetScopedName(), 5.0);
    	  
    	  sdf::ElementPtr frontrightElement = this->model_->GetLink("right_wheel")->GetSDF();
    	  
    	  sdf::ElementPtr frontleftElement = this->model_->GetLink("left_wheel")->GetSDF();
    	  
    	  sdf::ElementPtr backleftElement = this->model_->GetLink("left_wheel_2")->GetSDF();
    	  
    	  sdf::ElementPtr backrightElement = this->model_->GetLink("right_wheel_2")->GetSDF();
    	  
    	  sdf::ElementPtr velodyne_top = this->model_->GetLink("top")->GetSDF();
    	  
    	  vertical_samples = velodyne_top->GetElement("sensor")->GetElement("ray")
                    ->GetElement("scan")->GetElement("vertical")->Get<int>("samples");
          
         h_resolution = velodyne_top->GetElement("sensor")->GetElement("ray")
                    ->GetElement("scan")->GetElement("horizontal")->Get<int>("resolution");
                    
         v_resolution = velodyne_top->GetElement("sensor")->GetElement("ray")
                    ->GetElement("scan")->GetElement("vertical")->Get<int>("resolution");
         executor_thread_ = std::thread([this]() {
    rclcpp::spin(ros_node_);
});

    }
    
    // Calculate angle interpretable in the Gazebo world
    public: float RefineAngle(float initial_angle) {
    float refined_angle;
    if (initial_angle < 0)
       {
       	refined_angle = abs(initial_angle * (180/PI));
       }
       
       else
       {
       	refined_angle = 360 - (initial_angle * (180/PI));
       }
       
      return refined_angle;
    }

    public: void Update(ConstLaserScanStampedPtr &msg)
    {
    
    	ignition::math::Pose3d modelPose_vehicle = model_->WorldPose();
    	ignition::math::Vector3d modelPosition_vehicle = modelPose_vehicle.Pos();
             
    	ignition::math::Pose3d modelPose_sensor = joint->WorldPose();
    	ignition::math::Vector3d modelPosition_sensor = modelPose_sensor.Pos();
    
    	modelOrientation_vehicle = modelPose_vehicle.Rot();
    	eulerAngles_vehicle = modelOrientation_vehicle.Euler();
   
    	modelOrientation_sensor = modelPose_sensor.Rot();
    	eulerAngles_sensor = modelOrientation_sensor.Euler();
    
       auto minRange = std_msgs::msg::Float64();
       auto maxRange = std_msgs::msg::Float64();
       minRange.data = msg->scan().range_min();
       maxRange.data = msg->scan().range_max();
       
       MinRange_publisher->publish(minRange);
       MaxRange_publisher->publish(maxRange);
      
      // Get the min and max angles of the laser
      double angle_min = msg->scan().angle_min();
      double angle_max = msg->scan().angle_max();
      
      auto h_samples = std_msgs::msg::Int32();
      h_samples.data = msg->scan().ranges_size();
      HSamples_publisher->publish(h_samples);
      
      auto vsamples = std_msgs::msg::Int32();
      vsamples.data = vertical_samples;
      VSamples_publisher->publish(vsamples);
      
      auto hresolution = std_msgs::msg::Int32();
      hresolution.data = h_resolution;
      HResolution_publisher->publish(hresolution);
      
      auto vresolution = std_msgs::msg::Int32();
      vresolution.data = v_resolution;
      VResolution_publisher->publish(vresolution);      
      
      int count = msg->scan().count();

      // Calculate the angle increment
      double angle_increment = (angle_max - angle_min) / (count - 1);
      double resolution = (angle_max - angle_min) / angle_increment;
      
      double rotate;
      
      if((eulerAngles_vehicle.Z() * (180/PI)) < 0)
       {
       	rotate = 180 + (-1 * eulerAngles_vehicle.Z() * (180/PI));
       }
       	
       else 
       {
       	rotate = 180 - (eulerAngles_vehicle.Z() * (180/PI));
       }
      //double angle_increment = (msg->scan().angle_max() - msg->scan().angle_min()) / (count - 1)

     for (int i = 10; i < count - 10; ++i)
      {
        double range = msg->scan().ranges(i);
        double angle = angle_min + i * angle_increment;
        double total = (eulerAngles_sensor.Z() + angle) * (180/PI);
        double refine;
        
        if(total < 0)
        {
        	refine = 180 + (-1 * total);
        }
        
        else
        {
        	refine = 180 - (total);
        }

        // Convert polar coordinates (angle, range) to Cartesian (x, y)
        double x = range * cos(angle);
        double y = range * sin(angle);
        
        if (((eulerAngles_sensor.Z() + angle_min + 0 * angle_increment) * (180/PI)) < 0)
       {
       	refined_angle = abs((eulerAngles_sensor.Z() + angle_min + 0 * angle_increment) * (180/PI));
       }
       
       else
       {
       	refined_angle = 360 - ((eulerAngles_sensor.Z() + angle_min + 0 * angle_increment) * (180/PI));
       }
      
      auto distance1 = std_msgs::msg::Float64();
      distance1.data = msg->scan().ranges(0);
      Distance1_publisher->publish(distance1);
      
      auto distance2 = std_msgs::msg::Float64();
      distance2.data = msg->scan().ranges(1);
      Distance2_publisher->publish(distance2);
      
      auto distance3 = std_msgs::msg::Float64();
      distance3.data = msg->scan().ranges(2);
      Distance3_publisher->publish(distance3);
      
      auto distance4 = std_msgs::msg::Float64();
      distance4.data = msg->scan().ranges(3);
      Distance4_publisher->publish(distance4);
      
      auto distance5 = std_msgs::msg::Float64();
      distance5.data = msg->scan().ranges(4);
      Distance5_publisher->publish(distance5);
      
      auto distance6 = std_msgs::msg::Float64();
      distance6.data = msg->scan().ranges(5);
      Distance6_publisher->publish(distance6);
      
      auto distance7 = std_msgs::msg::Float64();
      distance7.data = msg->scan().ranges(6);
      Distance7_publisher->publish(distance7);
      
      auto distance8 = std_msgs::msg::Float64();
      distance8.data = msg->scan().ranges(7);
      Distance8_publisher->publish(distance8);
      
      auto distance9 = std_msgs::msg::Float64();
      distance9.data = msg->scan().ranges(8);
      Distance9_publisher->publish(distance9);
      
      auto distance10 = std_msgs::msg::Float64();
      distance10.data = msg->scan().ranges(9);
      Distance10_publisher->publish(distance10);
      
      auto distance11 = std_msgs::msg::Float64();
      distance11.data = msg->scan().ranges(10);
      Distance11_publisher->publish(distance11);
      
      auto distance12 = std_msgs::msg::Float64();
      distance12.data = msg->scan().ranges(11);
      Distance12_publisher->publish(distance12);
      
      auto distance13 = std_msgs::msg::Float64();
      distance13.data = msg->scan().ranges(12);
      Distance13_publisher->publish(distance13);
      
      auto distance14 = std_msgs::msg::Float64();
      distance14.data = msg->scan().ranges(13);
      Distance14_publisher->publish(distance14);
      
      auto distance15 = std_msgs::msg::Float64();
      distance15.data = msg->scan().ranges(14);
      Distance15_publisher->publish(distance15);
      
      auto distance16 = std_msgs::msg::Float64();
      distance16.data = msg->scan().ranges(15);
      Distance16_publisher->publish(distance16);
      
      auto distance17 = std_msgs::msg::Float64();
      distance17.data = msg->scan().ranges(16);
      Distance17_publisher->publish(distance17);
      
      auto distance18 = std_msgs::msg::Float64();
      distance18.data = msg->scan().ranges(17);
      Distance18_publisher->publish(distance18);
      
      auto distance19 = std_msgs::msg::Float64();
      distance19.data = msg->scan().ranges(18);
      Distance19_publisher->publish(distance19);
      
      auto distance20 = std_msgs::msg::Float64();
      distance20.data = msg->scan().ranges(19);
      Distance20_publisher->publish(distance20);
      
      auto distance21 = std_msgs::msg::Float64();
      distance21.data = msg->scan().ranges(20);
      Distance21_publisher->publish(distance21);
      
      auto distance22 = std_msgs::msg::Float64();
      distance22.data = msg->scan().ranges(21);
      Distance22_publisher->publish(distance22);
      
      auto distance23 = std_msgs::msg::Float64();
      distance23.data = msg->scan().ranges(22);
      Distance23_publisher->publish(distance23);
      
      auto distance24 = std_msgs::msg::Float64();
      distance24.data = msg->scan().ranges(23);
      Distance24_publisher->publish(distance24);
      
      auto distance25 = std_msgs::msg::Float64();
      distance25.data = msg->scan().ranges(24);
      Distance25_publisher->publish(distance25);
      
      auto distance26 = std_msgs::msg::Float64();
      distance26.data = msg->scan().ranges(25);
      Distance26_publisher->publish(distance26);
      
      auto distance27 = std_msgs::msg::Float64();
      distance27.data = msg->scan().ranges(26);
      Distance27_publisher->publish(distance27);
      
      auto distance28 = std_msgs::msg::Float64();
      distance28.data = msg->scan().ranges(27);
      Distance28_publisher->publish(distance28);
      
      auto distance29 = std_msgs::msg::Float64();
      distance29.data = msg->scan().ranges(28);
      Distance29_publisher->publish(distance29);
      
      auto distance30 = std_msgs::msg::Float64();
      distance30.data = msg->scan().ranges(29);
      Distance30_publisher->publish(distance30);
      
      auto distance31 = std_msgs::msg::Float64();
      distance31.data = msg->scan().ranges(30);
      Distance31_publisher->publish(distance31);
      
      auto distance32 = std_msgs::msg::Float64();
      distance32.data = msg->scan().ranges(31);
      Distance32_publisher->publish(distance32);
      
      auto angle1 = std_msgs::msg::Float64();
      angle1.data = RefineAngle(eulerAngles_sensor.Z() + angle_min + 0 * angle_increment);
      Angle1_publisher->publish(angle1);
      
      auto angle2 = std_msgs::msg::Float64();
      angle2.data = RefineAngle(eulerAngles_sensor.Z() + angle_min + 1 * angle_increment);
      Angle2_publisher->publish(angle2);
      
      auto angle3 = std_msgs::msg::Float64();
      angle3.data = RefineAngle(eulerAngles_sensor.Z() + angle_min + 2 * angle_increment);
      Angle3_publisher->publish(angle3);
      
      auto angle4 = std_msgs::msg::Float64();
      angle4.data = RefineAngle(eulerAngles_sensor.Z() + angle_min + 3 * angle_increment);
      Angle4_publisher->publish(angle4);
      
      auto angle5 = std_msgs::msg::Float64();
      angle5.data = RefineAngle(eulerAngles_sensor.Z() + angle_min + 4 * angle_increment);
      Angle5_publisher->publish(angle5);
      
      auto angle6 = std_msgs::msg::Float64();
      angle6.data = RefineAngle(eulerAngles_sensor.Z() + angle_min + 5 * angle_increment);
      Angle6_publisher->publish(angle6);
      
      auto angle7 = std_msgs::msg::Float64();
      angle7.data = RefineAngle(eulerAngles_sensor.Z() + angle_min + 6 * angle_increment);
      Angle7_publisher->publish(angle7);
      
      auto angle8 = std_msgs::msg::Float64();
      angle8.data = RefineAngle(eulerAngles_sensor.Z() + angle_min + 7 * angle_increment);
      Angle8_publisher->publish(angle8);
      
      auto angle9 = std_msgs::msg::Float64();
      angle9.data = RefineAngle(eulerAngles_sensor.Z() + angle_min + 8 * angle_increment);
      Angle9_publisher->publish(angle9);
      
      auto angle10 = std_msgs::msg::Float64();
      angle10.data = RefineAngle(eulerAngles_sensor.Z() + angle_min + 9 * angle_increment);
      Angle10_publisher->publish(angle10);
      
      auto angle11 = std_msgs::msg::Float64();
      angle11.data = RefineAngle(eulerAngles_sensor.Z() + angle_min + 10 * angle_increment);
      Angle11_publisher->publish(angle11);
      
      auto angle12 = std_msgs::msg::Float64();
      angle12.data = RefineAngle(eulerAngles_sensor.Z() + angle_min + 11 * angle_increment);
      Angle12_publisher->publish(angle12);
      
      auto angle13 = std_msgs::msg::Float64();
      angle13.data = RefineAngle(eulerAngles_sensor.Z() + angle_min + 12 * angle_increment);
      Angle13_publisher->publish(angle13);
      
      auto angle14 = std_msgs::msg::Float64();
      angle14.data = RefineAngle(eulerAngles_sensor.Z() + angle_min + 13 * angle_increment);
      Angle14_publisher->publish(angle14);
      
      auto angle15 = std_msgs::msg::Float64();
      angle15.data = RefineAngle(eulerAngles_sensor.Z() + angle_min + 14 * angle_increment);
      Angle15_publisher->publish(angle15);
      
      auto angle16 = std_msgs::msg::Float64();
      angle16.data = RefineAngle(eulerAngles_sensor.Z() + angle_min + 15 * angle_increment);
      Angle16_publisher->publish(angle16);
      
      auto angle17 = std_msgs::msg::Float64();
      angle17.data = RefineAngle(eulerAngles_sensor.Z() + angle_min + 16 * angle_increment);
      Angle17_publisher->publish(angle17);
      
      auto angle18 = std_msgs::msg::Float64();
      angle18.data = RefineAngle(eulerAngles_sensor.Z() + angle_min + 17 * angle_increment);
      Angle18_publisher->publish(angle18);
      
      auto angle19 = std_msgs::msg::Float64();
      angle19.data = RefineAngle(eulerAngles_sensor.Z() + angle_min + 18 * angle_increment);
      Angle19_publisher->publish(angle19);
      
      auto angle20 = std_msgs::msg::Float64();
      angle20.data = RefineAngle(eulerAngles_sensor.Z() + angle_min + 19 * angle_increment);
      Angle20_publisher->publish(angle20);
      
      auto angle21 = std_msgs::msg::Float64();
      angle21.data = RefineAngle(eulerAngles_sensor.Z() + angle_min + 20 * angle_increment);
      Angle21_publisher->publish(angle21);
      
      auto angle22 = std_msgs::msg::Float64();
      angle22.data = RefineAngle(eulerAngles_sensor.Z() + angle_min + 21 * angle_increment);
      Angle22_publisher->publish(angle22);
      
      auto angle23 = std_msgs::msg::Float64();
      angle23.data = RefineAngle(eulerAngles_sensor.Z() + angle_min + 22 * angle_increment);
      Angle23_publisher->publish(angle23);
      
      auto angle24 = std_msgs::msg::Float64();
      angle24.data = RefineAngle(eulerAngles_sensor.Z() + angle_min + 23 * angle_increment);
      Angle24_publisher->publish(angle24);
      
      auto angle25 = std_msgs::msg::Float64();
      angle25.data = RefineAngle(eulerAngles_sensor.Z() + angle_min + 24 * angle_increment);
      Angle25_publisher->publish(angle25);
      
      auto angle26 = std_msgs::msg::Float64();
      angle26.data = RefineAngle(eulerAngles_sensor.Z() + angle_min + 25 * angle_increment);
      Angle26_publisher->publish(angle26);
      
      auto angle27 = std_msgs::msg::Float64();
      angle27.data = RefineAngle(eulerAngles_sensor.Z() + angle_min + 26 * angle_increment);
      Angle27_publisher->publish(angle27);
      
      auto angle28 = std_msgs::msg::Float64();
      angle28.data = RefineAngle(eulerAngles_sensor.Z() + angle_min + 27 * angle_increment);
      Angle28_publisher->publish(angle28);
      
      auto angle29 = std_msgs::msg::Float64();
      angle29.data = RefineAngle(eulerAngles_sensor.Z() + angle_min + 28 * angle_increment);
      Angle29_publisher->publish(angle29);
      
      auto angle30 = std_msgs::msg::Float64();
      angle30.data = RefineAngle(eulerAngles_sensor.Z() + angle_min + 29 * angle_increment);
      Angle30_publisher->publish(angle30);
      
      auto angle31 = std_msgs::msg::Float64();
      angle31.data = RefineAngle(eulerAngles_sensor.Z() + angle_min + 30 * angle_increment);
      Angle31_publisher->publish(angle31);
      
      auto angle32 = std_msgs::msg::Float64();
      angle32.data = RefineAngle(eulerAngles_sensor.Z() + angle_min + 31 * angle_increment);
      Angle32_publisher->publish(angle32);
      
      if(mode == "MANUAL")
      {
      	
      	if(start == 0)
      	{
      	   Stop();
      	}
      	
      	else if (start == 1)
      	{
      	   front_right = default_speed;
    	   front_left = default_speed;
      	   back_right = default_speed;
    	   back_left = default_speed;
    	   
    	   if(forward == 1)
    	   {
    	   	front_right = default_speed;
    	   	front_left = default_speed;
      	   	back_right = default_speed;
    	   	back_left = default_speed;
    	   	
    	   	if(anticlockwise == 1)
    	   	{
    	   		front_right = -3;
	    	 	front_left = 7;
    			back_right = -3;
      			back_left = 7;
    	   	}
    	   
    	   	else if(clockwise == 1)
	    	{
    	   		front_right = 7;
	    	 	front_left = -3;
	    		back_right = 7;
	      		back_left = -3;
		}
    	   }
    	   
    	   else if(backward == 1)
    	   {
    	   	front_right = -1 * default_speed;
    	   	front_left = -1 * default_speed;
	      	back_right = -1 * default_speed;
    	   	back_left = -1 * default_speed;
    	   	
    	   	if(anticlockwise == 1)
    	   	{
    	   		front_right = -3;
	    	 	front_left = 7;
    			back_right = -3;
      			back_left = 7;
    	   	}
    	   
    	   	else if(clockwise == 1)
	    	{
    	   		front_right = 7;
	    	 	front_left = -3;
	    		back_right = 7;
	      		back_left = -3;
		}
    	   }    	   
      	}
      }
       
       else if (mode == "AUTO")
       {
       default_speed = 2;
       switch(sector)
       {
         case 0:
       	if((((msg->scan().ranges(22) <= 4) || (msg->scan().ranges(23) <= 4) || (msg->scan().ranges(24) <= 4) || (msg->scan().ranges(25) <= 4) || (msg->scan().ranges(26) <= 4) || (msg->scan().ranges(27) <= 4) || (msg->scan().ranges(28) <= 4)) && (refine >= 135 && refine <= 179)))
      		{
         		Stop();
         		sector = 1;
         	}
         	
         	else if(((msg->scan().ranges(22) <= 4) || (msg->scan().ranges(23) <= 4) || (msg->scan().ranges(24) <= 4) || (msg->scan().ranges(25) <= 4) || (msg->scan().ranges(26) <= 4) || (msg->scan().ranges(27) <= 4) || (msg->scan().ranges(28) <= 4)) && (refine >= 90 && refine <= 134))
         	{
         		Stop();
         		sector = 3;
         	}
         	
         	else if(((msg->scan().ranges(22) <= 4) || (msg->scan().ranges(23) <= 4) || (msg->scan().ranges(24) <= 4) || (msg->scan().ranges(25) <= 4) || (msg->scan().ranges(26) <= 4) || (msg->scan().ranges(27) <= 4) || (msg->scan().ranges(28) <= 4)) && (refine >= 45 && refine <= 89))
         	{
         		Stop();
         		sector = 4;
         	}
         	
         	else if(((msg->scan().ranges(22) <= 4) || (msg->scan().ranges(23) <= 4) || (msg->scan().ranges(24) <= 4) || (msg->scan().ranges(25) <= 4) || (msg->scan().ranges(26) <= 4) || (msg->scan().ranges(27) <= 4) || (msg->scan().ranges(28) <= 4)) && (refine >= 180 && refine <= 224))
         	{
         		Stop();
         		sector = 5;
         	}
         	
         	else if(((msg->scan().ranges(22) <= 4) || (msg->scan().ranges(23) <= 4) || (msg->scan().ranges(24) <= 4) || (msg->scan().ranges(25) <= 4) || (msg->scan().ranges(26) <= 4) || (msg->scan().ranges(27) <= 4) || (msg->scan().ranges(28) <= 4)) && (refine >= 225 && refine <= 269))
         	{
         		Stop();
         		sector = 6;
         	}
         	
         	else if(((msg->scan().ranges(22) <= 4) || (msg->scan().ranges(23) <= 4) || (msg->scan().ranges(24) <= 4) || (msg->scan().ranges(25) <= 4) || (msg->scan().ranges(26) <= 4) || (msg->scan().ranges(27) <= 4) || (msg->scan().ranges(28) <= 4)) && (refine >= 270 && refine <= 314))
         	{
         		Stop();
         		sector = 2;
         	}
         	
         	else if(((msg->scan().ranges(22) <= 4) || (msg->scan().ranges(23) <= 4) || (msg->scan().ranges(24) <= 4) || (msg->scan().ranges(25) <= 4) || (msg->scan().ranges(26) <= 4) || (msg->scan().ranges(27) <= 4) || (msg->scan().ranges(28) <= 4)) && (refine >= 315 && refine <= 360))
         	{
         		Stop();
         		sector = 7;
         	}
         	
         	else if(((msg->scan().ranges(22) <= 4) || (msg->scan().ranges(23) <= 4) || (msg->scan().ranges(24) <= 4) || (msg->scan().ranges(25) <= 4) || (msg->scan().ranges(26) <= 4) || (msg->scan().ranges(27) <= 4) || (msg->scan().ranges(28) <= 4)) && (refine >= 0 && refine <= 44))
         	{
         		Stop();
         		sector = 8;
         	}

         	else
         	{
         		front_right = default_speed;
    		front_left = default_speed;
    		back_right = default_speed;
    		back_left = default_speed;
    		sector = 0;
         	}

         	break;
         
         case 1:
         //std::cout << eulerAngles_vehicle.Z() * (180/PI) << std::endl;
             if(rotate >= 135 && rotate < 180)
             {
		front_right = 3;
    	 	front_left = -7;
    		back_right = 3;
      		back_left = -7;
      	    }
      	
      	else
      	{
    		front_right = default_speed;
    		front_left = default_speed;
    		back_right = default_speed;
    		back_left = default_speed;
    		sector = 0;
      	}
           break;
           
           
        case 2:
         //std::cout << eulerAngles_vehicle.Z() * (180/PI) << std::endl;
             if(rotate >= 270 && rotate < 315)
             {
             front_right = 3;
    	 	front_left = -7;
    		back_right = 3;
      		back_left = -7;
      	    }
      	
      	else
      	{
	    	front_right = default_speed;
    		front_left = default_speed;
    		back_right = default_speed;
    		back_left = default_speed;
    		sector = 0;
      	}
           break;
           
           case 3:
         //std::cout << eulerAngles_vehicle.Z() * (180/PI) << std::endl;
             if(rotate >= 90 && rotate < 135)
             {
		front_right = 3;
    	 	front_left = -7;
    		back_right = 3;
      		back_left = -7;
      	    }
      	
      	else
      	{
    		front_right = default_speed;
    		front_left = default_speed;
    		back_right = default_speed;
    		back_left = default_speed;
    		sector = 0;
      	}
           break;
           
           case 4:
         //std::cout << eulerAngles_vehicle.Z() * (180/PI) << std::endl;
             if(rotate >= 45 && rotate < 90)
             {
		front_right = 3;
    	 	front_left = -7;
    		back_right = 3;
      		back_left = -7;
      	    }
      	
      	else
      	{
    		front_right = default_speed;
    			front_left = default_speed;
    			back_right = default_speed;
    			back_left = default_speed;
    			sector = 0;
      	}
           break;
           
           case 5:
         //std::cout << eulerAngles_vehicle.Z() * (180/PI) << std::endl;
             if(rotate >= 180 && rotate < 225)
             {
		front_right = 3;
    	 	front_left = -7;
    		back_right = 3;
      		back_left = -7;
      	    }
      	
      	else
      	{
      		front_right = default_speed;
    			front_left = default_speed;
    			back_right = default_speed;
    			back_left = default_speed;
    			sector = 0;
      	}
           break;
           
           case 6:
         //std::cout << eulerAngles_vehicle.Z() * (180/PI) << std::endl;
             if(rotate >= 225 && rotate < 270)
             {
		front_right = 3;
    	 	front_left = -7;
    		back_right = 3;
      		back_left = -7;
      	    }
      	
      	else
      	{
    		front_right = default_speed;
    			front_left = default_speed;
    			back_right = default_speed;
    			back_left = default_speed;
    			sector = 0;
      	}
           break;
           
           case 7:
         //std::cout << eulerAngles_vehicle.Z() * (180/PI) << std::endl;
             if(rotate >= 315 && rotate < 360)
             {
		front_right = 3;
    	 	front_left = -7;
    		back_right = 3;
      		back_left = -7;
      	    }
      	
      	else
      	{
    			front_right = default_speed;
    			front_left = default_speed;
    			back_right = default_speed;
    			back_left = default_speed;
    			sector = 0;
      	}
           break;
           
           case 8:
         //std::cout << eulerAngles_vehicle.Z() * (180/PI) << std::endl;
             if(rotate >= 0 && rotate < 45)
             {
		front_right = 3;
    	 	front_left = -7;
    		back_right = 3;
      		back_left = -7;
      	    }
      	
      	else
      	{
    		front_right = default_speed;
    			front_left = default_speed;
    			back_right = default_speed;
    			back_left = default_speed;
    			sector = 0;
      	}
           break;
       	
       }
       }
      }
    }
    
    public: void Shutdown_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
    	this->worldPtr = gazebo::physics::get_world("default");
    	
    	if(msg->data == 1)
    	{
        	this->worldPtr->SetPaused(true);
        }
    }
    
    public: void StartStop_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
    	if(msg->data == 0)
    	{
    		start = 0;
        }
        else if(msg->data == 1)
        {
        	start = 1;
        }
        //std::cout << std::endl << "Start/Stop: " << std::endl << msg->data;
    }
    
    public: void Mode_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
    	
    	if(msg->data == 1)
    	{
    		mode = "MANUAL";
    	}
    	
    	else if(msg->data == 2)
    	{
    		mode = "AUTO";
    	}
    }
    
    public: void Forward_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
    	backward = 0;
    	forward = 1;
    }
    
    public: void Backward_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
    	backward = 1;
    	forward = 0;
    }
    
    public: void Velocity_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
    	default_speed = msg->data;
    }
    
    public: void TurnLeft_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
    	clockwise = 0;
    	
    	if(msg->data == 0)
    	{
    		anticlockwise = 0;
    	}
    	
    	else if(msg->data == 1)
    	{
    		anticlockwise = 1;
    	}
    }
    
    public: void TurnRight_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
    	anticlockwise = 0;
    	if(msg->data == 0)
    	{
    		clockwise = 0;
    	}
    	
    	else if(msg->data == 1)
    	{
    		clockwise = 1;
    	}
    }
    
    public: void OnUpdate()
    {
    
    ignition::math::Pose3d modelPose_body = model_->WorldPose();
    ignition::math::Vector3d modelPosition_body = modelPose_body.Pos();
    ignition::math::Vector3d linearVel = this->model_->WorldLinearVel();
    
    auto velocity = std_msgs::msg::Float64();
    
    velocity.data = linearVel.Length();
    Velocity_publisher->publish(velocity);
    
    	//this->model_->SetLinearVel(ignition::math::Vector3d(val, yal, 0));
    	this->model_->GetJoint("left_wheel_hinge_2")->SetVelocity(0, front_left); // left wheel
        this->model_->GetJoint("right_wheel_hinge_2")->SetVelocity(0, front_right);
       this->model_->GetJoint("left_wheel_hinge")->SetVelocity(0, back_left); // left wheel
        this->model_->GetJoint("right_wheel_hinge")->SetVelocity(0, back_right);
       auto posex = std_msgs::msg::Float64();
       auto posey = std_msgs::msg::Float64();
       posex.data = modelPose_body.X();
       posey.data = modelPose_body.Y();
       posex_publisher->publish(posex);
       posey_publisher->publish(posey);
       
       auto now = std::chrono::system_clock::now();
       auto now_time_t = std::chrono::system_clock::to_time_t(now);
       auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
       
       std::tm* local_tm = std::localtime(&now_time_t);
       
       auto angle_rotation = std_msgs::msg::Float64();
       
       if((eulerAngles_vehicle.Z() * (180/PI)) < 0)
       {
       	angle_rotation.data = abs(eulerAngles_vehicle.Z() * (180/PI));
       }
       	
       else 
       {
       	angle_rotation.data = 360 - (eulerAngles_vehicle.Z() * (180/PI));
       }
       	
       AngleRotation_publisher->publish(angle_rotation);
    }
    
    void Stop()
    {
    	front_left = 0;
    	front_right = 0;
    	back_left = 0;
    	back_right = 0;
    }

    private:
        // Pointer to the model
        physics::ModelPtr model_;
        physics::WorldPtr worldPtr;
        
        // ROS 2 node and publisher
        rclcpp::Node::SharedPtr ros_node_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr posex_publisher;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr posey_publisher;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr MinRange_publisher;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr MaxRange_publisher;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr AngleRotation_publisher;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr Velocity_publisher;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr HSamples_publisher;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr VSamples_publisher;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr HResolution_publisher;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr VResolution_publisher;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr Distance1_publisher, Distance2_publisher, Distance3_publisher, Distance4_publisher, Distance5_publisher, Distance6_publisher, Distance7_publisher, Distance8_publisher, Distance9_publisher, Distance10_publisher, Distance11_publisher, Distance12_publisher, Distance13_publisher, Distance14_publisher, Distance15_publisher, Distance16_publisher, Distance17_publisher, Distance18_publisher, Distance19_publisher, Distance20_publisher, Distance21_publisher, Distance22_publisher, Distance23_publisher, Distance24_publisher, Distance25_publisher, Distance26_publisher, Distance27_publisher, Distance28_publisher, Distance29_publisher, Distance30_publisher, Distance31_publisher, Distance32_publisher;
        
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr Angle1_publisher, Angle2_publisher, Angle3_publisher, Angle4_publisher, Angle5_publisher, Angle6_publisher, Angle7_publisher, Angle8_publisher, Angle9_publisher, Angle10_publisher, Angle11_publisher, Angle12_publisher, Angle13_publisher, Angle14_publisher, Angle15_publisher, Angle16_publisher, Angle17_publisher, Angle18_publisher, Angle19_publisher, Angle20_publisher, Angle21_publisher, Angle22_publisher, Angle23_publisher, Angle24_publisher, Angle25_publisher, Angle26_publisher, Angle27_publisher, Angle28_publisher, Angle29_publisher, Angle30_publisher, Angle31_publisher, Angle32_publisher;
     rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr Shutdown_subscription, StartStop_subscription, Mode_subscription, Forward_subscription, Backward_subscription, TurnLeft_subscription, TurnRight_subscription;
     rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr Velocity_subscription;
     
     private: std::thread executor_thread_;

        // Timer to periodically publish velocity
        rclcpp::TimerBase::SharedPtr timer_;
        
        private: physics::JointPtr joint;
    
    private: physics::LinkPtr link;
    
    private: common::PID pid;
    
    private: sensors::GpuRaySensorPtr sensor;
    
    private: std::string status = "UP";
    
    transport::NodePtr node;
    transport::SubscriberPtr raySub;
    };

    // Register the plugin with Gazebo
    GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
#endif
