#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/gazebo.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
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

namespace gazebo
{
    class ModelPush : public ModelPlugin
    {
    
    #define PI 3.1415926535897932384626433832795028841971
    
    float front_right = 1.2;
    float front_left = 1.2;
    float back_right = 1.2;
    float back_left = 1.2;
    
    private: event::ConnectionPtr updateConnection;
    public:
        ModelPush() {}

        // Load function (called when the plugin is loaded into Gazebo)
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
        {
            // Initialize ROS 2 if it hasn't been initialized yet
            if (!rclcpp::ok()) {
                rclcpp::init(0, nullptr);
            }

            // Create a ROS 2 node (do not inherit from Node)
            ros_node_ = rclcpp::Node::make_shared("model_push_node");

            // Create the ROS 2 publisher to publish velocity
            posex_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("x_position", 10);
            
            posey_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("y_position", 10);
            
            MinRange_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("min_range", 10);
            
            MaxRange_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("max_range", 10);

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
    	  this->joint->GetScopedName(), 10.0);
          
        }
        

    
    public: void Update(ConstLaserScanStampedPtr &msg)
    {
       auto minRange = std_msgs::msg::Float64();
       auto maxRange = std_msgs::msg::Float64();
       minRange.data = msg->scan().range_min();
       maxRange.data = msg->scan().range_max();
       
       MinRange_publisher->publish(minRange);
       MaxRange_publisher->publish(maxRange);
       
        //int count = this->raySensor->RangeCount();

      // Get the min and max angles of the laser
      double angle_min = msg->scan().angle_min();
      double angle_max = msg->scan().angle_max();
      
      int count = msg->scan().count();

      // Calculate the angle increment
      double angle_increment = (angle_max - angle_min) / (count - 1);
      //double angle_increment = (msg->scan().angle_max() - msg->scan().angle_min()) / (count - 1)
       
     for (int i = 0; i < count; ++i)
      {
        double range = msg->scan().ranges(i);
        double angle = angle_min + i * angle_increment;

        // Convert polar coordinates (angle, range) to Cartesian (x, y)
        double x = range * cos(angle);
        double y = range * sin(angle);

        // Print the point data
        std::cout << "Point " << (i + 1) << ": Range = " << range
                  << ", \t Angle = " << angle * (180/3.14159265358979)
                  << ", (X, Y) = (" << x << ", " << y << ")" << std::endl << std::endl;
        //std::cout << msg->scan().ranges(22) << std::endl << std::endl; 
        
        //std::cout << "Angle: " << angle_min + 0 * angle_increment * (180/3.14159265358979) << "     Range: " << msg->scan().ranges(0) << std::endl << std::endl;
        
       // std::cout << "Angle: " << angle_min + 11 * angle_increment * (180/3.14159265358979) << "     Range: " << msg->scan().ranges(11) << std::endl << std::endl;
        
//        std::cout << "Angle: " << angle_min + 12 * angle_increment * (180/3.14159265358979) << "     Range: " << msg->scan().ranges(12) << std::endl << std::endl;
        
  //      std::cout << "Angle: " << angle_min + 13 * angle_increment * (180/3.14159265358979) << "     Range: " << msg->scan().ranges(13) << std::endl << std::endl;
        
    //    std::cout << "Angle: " << angle_min + 14 * angle_increment * (180/3.14159265358979) << "     Range: " << msg->scan().ranges(14) << std::endl << std::endl;
        
      //  std::cout << "Angle: " << angle_min + 15 * angle_increment * (180/3.14159265358979) << "     Range: " << msg->scan().ranges(15) << std::endl << std::endl;
        
        //std::cout << "Angle: " << angle_min + 16 * angle_increment * (180/3.14159265358979) << "     Range: " << msg->scan().ranges(16) << std::endl << std::endl;
        
 //       std::cout << "Angle: " << angle_min + 17 * angle_increment * (180/3.14159265358979) << "     Range: " << msg->scan().ranges(17) << std::endl << std::endl;
        
   //     std::cout << "Angle: " << angle_min + 18 * angle_increment * (180/3.14159265358979) << "     Range: " << msg->scan().ranges(18) << std::endl << std::endl;
        
     //   std::cout << "Angle: " << angle_min + 19 * angle_increment * (180/3.14159265358979) << "     Range: " << msg->scan().ranges(19) << std::endl << std::endl;
        
       // std::cout << "Angle: " << angle_min + 20 * angle_increment * (180/3.14159265358979) << "     Range: " << msg->scan().ranges(20) << std::endl << std::endl;
        
       // std::cout << "Angle: " << angle_min + 21 * angle_increment * (180/3.14159265358979) << "     Range: " << msg->scan().ranges(21) << std::endl << std::endl;
              
      }
      
      if(msg->scan().ranges(22) <= 3.7)
      {
         front_right = 3.6;
    	 front_left = -4.4;
    	 back_right = 3.6;
         back_left = -4.4;
          
 //         ignition::math::Pose3d modelPose = model_->WorldPose();
   //       ignition::math::Vector3d modelPosition = modelPose.Pos();
     //     ignition::math::Quaterniond newOrientation(0.0, 0.0, 3.5); // Example: Identity quaternion
    //  ignition::math::Vector3d newPosition(modelPose.Y(), modelPose.X(), 0.0); // Example: Keep current position
      //this->model_->SetWorldPose(ignition::math::Pose3d(newPosition, newOrientation));
       }
       
       else if(msg->scan().ranges(22) > 3.7)
       {
     		front_right = 1.2;
            front_left = 1.2;
    	 back_right = 1.2;
    	 back_left = 1.2;
       }
      
      
        //const double ranges = msg->ranges(i);
        //std::cout << "Max Dist: " << msg->scan().MaxRange() << "\n";
        //std::cout << "Range: " << msg->scan().ranges(22) << " m" << std::endl;
        
    //for (size_t i = 0; i < msg->scan().ranges_size(); i++) {
    //double range = msg->scan().ranges(i);
    //int count = msg->scan().count();
    //double angle_increment = (msg->scan().angle_max() - msg->scan().angle_min()) / (count - 1);
    //double angle = msg->scan().angle_min() + i * angle_increment;
    
    //std::cout << "Angle: " << (msg->scan().angle_min() + i * angle_increment) * (180/3.14159265358979) << "\t\t" << msg->scan().ranges(i) << std::endl;

    // Now you have both 'angle' and 'range' for the i-th reading.
    // Process them as needed
//}

    }
    
    public: void OnUpdate()
    {
    
    ignition::math::Pose3d modelPose = model_->WorldPose();
    ignition::math::Vector3d modelPosition = modelPose.Pos();
    	//this->model_->SetLinearVel(ignition::math::Vector3d(val, yal, 0));
    	this->model_->GetJoint("left_wheel_hinge_2")->SetVelocity(0, front_left); // left wheel
        this->model_->GetJoint("right_wheel_hinge_2")->SetVelocity(0, front_right);
       this->model_->GetJoint("left_wheel_hinge")->SetVelocity(0, back_left); // left wheel
        this->model_->GetJoint("right_wheel_hinge")->SetVelocity(0, back_right);
       auto posex = std_msgs::msg::Float64();
       auto posey = std_msgs::msg::Float64();
       posex.data = modelPose.X();
       posey.data = modelPose.Y();
       posex_publisher->publish(posex);
       posey_publisher->publish(posey);
       
       //std::cout << msg->scan().count() << std::endl;
       
       
    }
    
    //public: void Stop()
    //{
    //	front_left = 0;
    //	front_right = 0;
    //	back_left = 0;
    //	back_right = 0;
    //}

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
