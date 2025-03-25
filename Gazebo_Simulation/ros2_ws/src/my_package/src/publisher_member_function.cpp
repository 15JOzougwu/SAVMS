#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

// Include Paho MQTT libraries
#include <mqtt/async_client.h>

using namespace std::chrono_literals;

class Talker : public rclcpp::Node, public virtual mqtt::callback
{
public:
    Talker() : Node("talker"), count_(0), mqtt_client_("tcp://localhost:1883", "ros2_mqtt_client")
    {
    	ros_node_ = rclcpp::Node::make_shared("model_push_node");
    
        posex_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "vehicle_status/navigation/x_position", 10, std::bind(&Talker::xposition_callback, this, std::placeholders::_1));
            
        posey_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "vehicle_status/navigation/y_position", 10, std::bind(&Talker::yposition_callback, this, std::placeholders::_1));
            
        MinRange_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/diagnostics/min_range", 10, std::bind(&Talker::MinRange_callback, this, std::placeholders::_1));
            
       MaxRange_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/diagnostics/max_range", 10, std::bind(&Talker::MaxRange_callback, this, std::placeholders::_1));
            
       AngleRotation_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "vehicle_status/navigation/angle", 10, std::bind(&Talker::angle_callback, this, std::placeholders::_1));
            
       Velocity_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "vehicle_status/navigation/velocity", 10, std::bind(&Talker::velocity_callback, this, std::placeholders::_1));
            
       HSamples_subscription = this->create_subscription<std_msgs::msg::Int32>(
            "sensors/Velodyne/diagnostics/hsamples", 10, std::bind(&Talker::HSamples_callback, this, std::placeholders::_1));
            
       VSamples_subscription = this->create_subscription<std_msgs::msg::Int32>(
            "sensors/Velodyne/diagnostics/vsamples", 10, std::bind(&Talker::VSamples_callback, this, std::placeholders::_1));
            
       HResolution_subscription = this->create_subscription<std_msgs::msg::Int32>(
            "sensors/Velodyne/diagnostics/hresolution", 10, std::bind(&Talker::HResolution_callback, this, std::placeholders::_1));
       
       VResolution_subscription = this->create_subscription<std_msgs::msg::Int32>(
            "sensors/Velodyne/diagnostics/vresolution", 10, std::bind(&Talker::VResolution_callback, this, std::placeholders::_1));
            
       Distance1_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/distance1", 10, std::bind(&Talker::Distance1_callback, this, std::placeholders::_1));
            
       Distance2_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/distance2", 10, std::bind(&Talker::Distance2_callback, this, std::placeholders::_1));
            
       Distance3_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/distance3", 10, std::bind(&Talker::Distance3_callback, this, std::placeholders::_1));
            
       Distance4_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/distance4", 10, std::bind(&Talker::Distance4_callback, this, std::placeholders::_1));
            
       Distance5_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/distance5", 10, std::bind(&Talker::Distance5_callback, this, std::placeholders::_1));
            
       Distance6_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/distance6", 10, std::bind(&Talker::Distance6_callback, this, std::placeholders::_1));
            
       Distance7_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/distance7", 10, std::bind(&Talker::Distance7_callback, this, std::placeholders::_1));
            
       Distance8_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/distance8", 10, std::bind(&Talker::Distance8_callback, this, std::placeholders::_1));
            
       Distance9_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/distance9", 10, std::bind(&Talker::Distance9_callback, this, std::placeholders::_1));
            
       Distance10_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/distance10", 10, std::bind(&Talker::Distance10_callback, this, std::placeholders::_1));
            
       Distance11_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/distance11", 10, std::bind(&Talker::Distance11_callback, this, std::placeholders::_1));
            
       Distance12_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/distance12", 10, std::bind(&Talker::Distance12_callback, this, std::placeholders::_1));
            
       Distance13_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/distance13", 10, std::bind(&Talker::Distance13_callback, this, std::placeholders::_1));
            
       Distance14_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/distance14", 10, std::bind(&Talker::Distance14_callback, this, std::placeholders::_1));
            
       Distance15_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/distance15", 10, std::bind(&Talker::Distance15_callback, this, std::placeholders::_1));
            
       Distance16_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/distance16", 10, std::bind(&Talker::Distance16_callback, this, std::placeholders::_1));
            
       Distance17_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/distance17", 10, std::bind(&Talker::Distance17_callback, this, std::placeholders::_1));
            
       Distance18_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/distance18", 10, std::bind(&Talker::Distance18_callback, this, std::placeholders::_1));
            
       Distance19_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/distance19", 10, std::bind(&Talker::Distance19_callback, this, std::placeholders::_1));
            
       Distance20_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/distance20", 10, std::bind(&Talker::Distance20_callback, this, std::placeholders::_1));
            
       Distance21_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/distance21", 10, std::bind(&Talker::Distance21_callback, this, std::placeholders::_1));
            
       Distance22_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/distance22", 10, std::bind(&Talker::Distance22_callback, this, std::placeholders::_1));
            
       Distance23_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/distance23", 10, std::bind(&Talker::Distance23_callback, this, std::placeholders::_1));
            
        Distance24_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/distance24", 10, std::bind(&Talker::Distance24_callback, this, std::placeholders::_1));
            
        Distance25_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/distance25", 10, std::bind(&Talker::Distance25_callback, this, std::placeholders::_1));
            
        Distance26_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/distance26", 10, std::bind(&Talker::Distance26_callback, this, std::placeholders::_1));
            
        Distance27_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/distance27", 10, std::bind(&Talker::Distance27_callback, this, std::placeholders::_1));
            
        Distance28_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/distance28", 10, std::bind(&Talker::Distance28_callback, this, std::placeholders::_1));
            
        Distance29_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/distance29", 10, std::bind(&Talker::Distance29_callback, this, std::placeholders::_1));
            
        Distance30_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/distance30", 10, std::bind(&Talker::Distance30_callback, this, std::placeholders::_1));
            
        Distance31_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/distance31", 10, std::bind(&Talker::Distance31_callback, this, std::placeholders::_1));
            
        Distance32_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/distance32", 10, std::bind(&Talker::Distance32_callback, this, std::placeholders::_1));
            
        Angle1_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/angle1", 10, std::bind(&Talker::Angle1_callback, this, std::placeholders::_1));
            
        Angle2_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/angle2", 10, std::bind(&Talker::Angle2_callback, this, std::placeholders::_1));
            
        Angle3_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/angle3", 10, std::bind(&Talker::Angle3_callback, this, std::placeholders::_1));
            
        Angle4_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/angle4", 10, std::bind(&Talker::Angle4_callback, this, std::placeholders::_1));
            
        Angle5_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/angle5", 10, std::bind(&Talker::Angle5_callback, this, std::placeholders::_1));
            
        Angle6_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/angle6", 10, std::bind(&Talker::Angle6_callback, this, std::placeholders::_1));
            
        Angle7_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/angle7", 10, std::bind(&Talker::Angle7_callback, this, std::placeholders::_1));
            
        Angle8_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/angle8", 10, std::bind(&Talker::Angle8_callback, this, std::placeholders::_1));
            
        Angle9_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/angle9", 10, std::bind(&Talker::Angle9_callback, this, std::placeholders::_1));
            
        Angle10_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/angle10", 10, std::bind(&Talker::Angle10_callback, this, std::placeholders::_1));
            
        Angle11_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/angle11", 10, std::bind(&Talker::Angle11_callback, this, std::placeholders::_1));
            
        Angle12_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/angle12", 10, std::bind(&Talker::Angle12_callback, this, std::placeholders::_1));
            
        Angle13_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/angle13", 10, std::bind(&Talker::Angle13_callback, this, std::placeholders::_1));
            
        Angle14_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/angle14", 10, std::bind(&Talker::Angle14_callback, this, std::placeholders::_1));
            
        Angle15_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/angle15", 10, std::bind(&Talker::Angle15_callback, this, std::placeholders::_1));
            
        Angle16_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/angle16", 10, std::bind(&Talker::Angle16_callback, this, std::placeholders::_1));
            
        Angle17_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/angle17", 10, std::bind(&Talker::Angle17_callback, this, std::placeholders::_1));
            
        Angle18_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/angle18", 10, std::bind(&Talker::Angle18_callback, this, std::placeholders::_1));
            
        Angle19_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/angle19", 10, std::bind(&Talker::Angle19_callback, this, std::placeholders::_1));
            
        Angle20_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/angle20", 10, std::bind(&Talker::Angle20_callback, this, std::placeholders::_1));
            
        Angle21_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/angle21", 10, std::bind(&Talker::Angle21_callback, this, std::placeholders::_1));
            
        Angle22_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/angle22", 10, std::bind(&Talker::Angle22_callback, this, std::placeholders::_1));
            
        Angle23_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/angle23", 10, std::bind(&Talker::Angle23_callback, this, std::placeholders::_1));
            
        Angle24_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/angle24", 10, std::bind(&Talker::Angle24_callback, this, std::placeholders::_1));
            
        Angle25_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/angle25", 10, std::bind(&Talker::Angle25_callback, this, std::placeholders::_1));
            
        Angle26_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/angle26", 10, std::bind(&Talker::Angle26_callback, this, std::placeholders::_1));
            
        Angle27_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/angle27", 10, std::bind(&Talker::Angle27_callback, this, std::placeholders::_1));
            
        Angle28_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/angle28", 10, std::bind(&Talker::Angle28_callback, this, std::placeholders::_1));
            
        Angle29_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/angle29", 10, std::bind(&Talker::Angle29_callback, this, std::placeholders::_1));
            
        Angle30_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/angle30", 10, std::bind(&Talker::Angle30_callback, this, std::placeholders::_1));
            
        Angle31_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/angle31", 10, std::bind(&Talker::Angle31_callback, this, std::placeholders::_1));
            
        Angle32_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/Velodyne/navigation/angle32", 10, std::bind(&Talker::Angle32_callback, this, std::placeholders::_1));
        
        StartStop_publisher = ros_node_->create_publisher<std_msgs::msg::Int32>("control/navigation/start_stop", 10);
            
        Shutdown_publisher = ros_node_->create_publisher<std_msgs::msg::Int32>("control/safety/emergency_shutdown", 10);
        
        Mode_publisher = ros_node_->create_publisher<std_msgs::msg::Int32>("control/mode_control", 10);
        
        Forward_publisher = ros_node_->create_publisher<std_msgs::msg::Int32>("control/navigation/move_forward", 10);
        
        Backward_publisher = ros_node_->create_publisher<std_msgs::msg::Int32>("control/navigation/move_backward", 10);
        
        Velocity_publisher = ros_node_->create_publisher<std_msgs::msg::Float64>("control/navigation/vel_control", 10);
        
        Left_publisher = ros_node_->create_publisher<std_msgs::msg::Int32>("control/navigation/turn_left", 10);
        
        Right_publisher = ros_node_->create_publisher<std_msgs::msg::Int32>("control/navigation/turn_right", 10);
        
    // MQTT client setup
        mqtt::connect_options connOpts;
        mqtt_client_.set_callback(*this);
        
        try {
            mqtt_client_.connect(connOpts)->wait();
            mqtt_client_.subscribe("control/safety/emergency_shutdown", 1)->wait();
            mqtt_client_.subscribe("control/navigation/start_stop", 1)->wait();
            mqtt_client_.subscribe("control/mode_control", 1)->wait();
            mqtt_client_.subscribe("control/navigation/move_forward", 1)->wait();
            mqtt_client_.subscribe("control/navigation/move_backward", 1)->wait();
            mqtt_client_.subscribe("control/navigation/turn_left", 1)->wait();
            mqtt_client_.subscribe("control/navigation/turn_right", 1)->wait();
            mqtt_client_.subscribe("control/navigation/vel_control", 1)->wait();
            RCLCPP_INFO(this->get_logger(), "Connected to MQTT broker");
        } catch (const mqtt::exception& exc) {
            RCLCPP_ERROR(this->get_logger(), "Error connecting to MQTT broker: %s", exc.what());
        }
    }
    
    ~Talker() {
        try {
            mqtt_client_.disconnect()->wait();
            RCLCPP_INFO(this->get_logger(), "Disconnected from MQTT broker");
        } catch (const mqtt::exception& exc) {
            RCLCPP_ERROR(this->get_logger(), "Error disconnecting from MQTT broker: %s", exc.what());
        }
    }

private:
    
  void xposition_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("vehicle_status/navigation/x_position", message_str);
        mqtt_message->set_qos(0);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
  void yposition_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("vehicle_status/navigation/y_position", message_str);
        mqtt_message->set_qos(0);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }  
    
  void MinRange_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/diagnostics/min_range", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
   void MaxRange_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/diagnostics/max_range", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
           
        }
    }
    
  void angle_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("vehicle_status/navigation/angle", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
  void velocity_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("vehicle_status/navigation/velocity", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void HSamples_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/diagnostics/hsamples", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void VSamples_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/diagnostics/vsamples", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void HResolution_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/diagnostics/hresolution", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void VResolution_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/diagnostics/vresolution", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Distance1_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/distance1", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Distance2_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/distance2", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Distance3_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/distance3", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Distance4_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/distance4", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Distance5_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/distance5", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Distance6_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/distance6", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Distance7_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/distance7", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Distance8_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/distance8", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Distance9_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/distance9", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Distance10_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/distance10", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Distance11_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/distance11", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Distance12_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/distance12", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Distance13_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/distance13", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
           
        }
    }
    
    void Distance14_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/distance14", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Distance15_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/distance15", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Distance16_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/distance16", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Distance17_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/distance17", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Distance18_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/distance18", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Distance19_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/distance19", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Distance20_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/distance20", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Distance21_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/distance21", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Distance22_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/distance22", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Distance23_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/distance23", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Distance24_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/distance24", message_str);
        mqtt_message->set_qos(1);
        
        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Distance25_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/distance25", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Distance26_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/distance26", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Distance27_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/distance27", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Distance28_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/distance28", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Distance29_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/distance29", message_str);
        mqtt_message->set_qos(1);
        
        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Distance30_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/distance30", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Distance31_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/distance31", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
           
        }
    }
    
    void Distance32_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/distance32", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Angle1_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/angle1", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Angle2_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/angle2", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Angle3_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/angle3", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Angle4_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/angle4", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
           
        }
    }
    
    void Angle5_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/angle5", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Angle6_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/angle6", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Angle7_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/angle7", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Angle8_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/angle8", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Angle9_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/angle9", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
           
        }
    }
    
    void Angle10_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/angle10", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Angle11_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/angle11", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Angle12_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/angle12", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Angle13_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/angle13", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
           
        }
    }
    
    void Angle14_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/angle14", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Angle15_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/angle15", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Angle16_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/angle16", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Angle17_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/angle17", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Angle18_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/angle18", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Angle19_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/angle19", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Angle20_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/angle20", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Angle21_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/angle21", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Angle22_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/angle22", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Angle23_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/angle23", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Angle24_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/angle24", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Angle25_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/angle25", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Angle26_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/angle26", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Angle27_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/angle27", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Angle28_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/angle28", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Angle29_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/angle29", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Angle30_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/angle30", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Angle31_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/angle31", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void Angle32_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string message_str = std::to_string(msg->data);

        // MQTT message
        mqtt::message_ptr mqtt_message = mqtt::make_message("sensors/Velodyne/navigation/angle32", message_str);
        mqtt_message->set_qos(1);

        try {
            mqtt_client_.publish(mqtt_message);
        } catch (const mqtt::exception& exc) {
            
        }
    }
    
    void message_arrived(mqtt::const_message_ptr msg) override
    {       
       try {
            double float_value = std::stof(msg->to_string());  // Convert string to float	
            double int_value = std::stoi(msg->to_string());  // Convert string to integer
            
            auto int_ros_msg = std_msgs::msg::Int32();
            int_ros_msg.data = int_value;
            
            auto float_ros_msg = std_msgs::msg::Float64();
            float_ros_msg.data = float_value;
            
              // Publish to ROS 2 topic
            
            std::string topic = msg->get_topic();
        if (topic == "control/safety/emergency_shutdown") {
            Shutdown_publisher->publish(int_ros_msg);
        }
        
        else if (topic == "control/navigation/start_stop") {
            StartStop_publisher->publish(int_ros_msg);
        }
        
        else if (topic == "control/mode_control") {
            Mode_publisher->publish(int_ros_msg);
        }
        
        else if (topic == "control/navigation/move_forward") {
            Forward_publisher->publish(int_ros_msg);
        }
        
        else if (topic == "control/navigation/move_backward") {
            Backward_publisher->publish(int_ros_msg);
        }
        
        else if (topic == "control/navigation/vel_control") {
            Velocity_publisher->publish(float_ros_msg);
        }
        
        else if (topic == "control/navigation/turn_left") {
            Left_publisher->publish(int_ros_msg);
        }
        
        else if (topic == "control/navigation/turn_right") {
            Right_publisher->publish(int_ros_msg);
        }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error parsing MQTT message: %s", e.what());
        } 
    }
  
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr StartStop_publisher;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr Shutdown_publisher;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr Mode_publisher;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr Forward_publisher;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr Backward_publisher;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr Left_publisher;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr Right_publisher;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr Velocity_publisher;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr posex_subscription;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr posey_subscription;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr MinRange_subscription;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr MaxRange_subscription;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr AngleRotation_subscription;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr Velocity_subscription;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr HSamples_subscription;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr VSamples_subscription;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr HResolution_subscription;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr VResolution_subscription;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr Control_subscription;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr StartStop_subscription;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr Distance1_subscription, Distance2_subscription, Distance3_subscription, Distance4_subscription, Distance5_subscription, Distance6_subscription, Distance7_subscription, Distance8_subscription, Distance9_subscription, Distance10_subscription, Distance11_subscription, Distance12_subscription, Distance13_subscription, Distance14_subscription, Distance15_subscription, Distance16_subscription, Distance17_subscription, Distance18_subscription, Distance19_subscription, Distance20_subscription, Distance21_subscription, Distance22_subscription, Distance23_subscription, Distance24_subscription, Distance25_subscription, Distance26_subscription, Distance27_subscription, Distance28_subscription, Distance29_subscription, Distance30_subscription, Distance31_subscription, Distance32_subscription;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr Angle1_subscription, Angle2_subscription, Angle3_subscription, Angle4_subscription, Angle5_subscription, Angle6_subscription, Angle7_subscription, Angle8_subscription, Angle9_subscription, Angle10_subscription, Angle11_subscription, Angle12_subscription, Angle13_subscription, Angle14_subscription, Angle15_subscription, Angle16_subscription, Angle17_subscription, Angle18_subscription, Angle19_subscription, Angle20_subscription, Angle21_subscription, Angle22_subscription, Angle23_subscription, Angle24_subscription, Angle25_subscription, Angle26_subscription, Angle27_subscription, Angle28_subscription, Angle29_subscription, Angle30_subscription, Angle31_subscription, Angle32_subscription;
  size_t count_;
  

    // MQTT client
    mqtt::async_client mqtt_client_;
};

int main(int argc, char **argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create and spin the Talker node
    rclcpp::spin(std::make_shared<Talker>());

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}

