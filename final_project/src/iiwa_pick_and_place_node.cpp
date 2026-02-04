#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <map>
#include <chrono>
#include <std_msgs/msg/empty.hpp> 

#include <ros_gz_interfaces/srv/spawn_entity.hpp>
#include <ros_gz_interfaces/srv/delete_entity.hpp>

using namespace std::chrono_literals;

class IiwaPickAndPlaceNode : public rclcpp::Node
{
public:
  IiwaPickAndPlaceNode() : Node("iiwa_pick_and_place_node")
  {
    declare_parameter<std::string>("pose_file", "");
    pose_file_ = get_parameter("pose_file").as_string();

    if (pose_file_.empty() || !load_yaml(pose_file_)) {
      RCLCPP_FATAL(get_logger(), "Failed to load YAML");
      rclcpp::shutdown(); return;
    }
    // --- Publisher Robot ---
    arm_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("/iiwa_arm_trajectory_controller/joint_trajectory", 10);
    gripper_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("/gripper_controller/joint_trajectory", 10);
    // --- Publisher Magnete iiwa ---
    attach_pub_ = create_publisher<std_msgs::msg::Empty>("/iiwa/grasp/attach", 10);
    detach_pub_ = create_publisher<std_msgs::msg::Empty>("/iiwa/grasp/detach", 10);
    // --- Publisher Magnete FRA2MO ---
    fra2mo_attach_pub_ = create_publisher<std_msgs::msg::Empty>("/fra2mo/grasp/fix_package", 10);
    fra2mo_detach_pub_ = create_publisher<std_msgs::msg::Empty>("/fra2mo/grasp/release_package", 10);

    // Publisher (per dire a Fra2mo che abbiamo finito)
    done_pub_ = create_publisher<std_msgs::msg::Empty>("/iiwa/manipulation_done", 10); 

    start_sub_ = create_subscription<std_msgs::msg::Empty>(
      "/iiwa/start_manipulation", 10, 
      std::bind(&IiwaPickAndPlaceNode::start_callback, this, std::placeholders::_1));

      // Trigger per lo scarico (Place)
    place_sub_ = create_subscription<std_msgs::msg::Empty>(
      "/fra2mo/trigger_place", 10, 
      std::bind(&IiwaPickAndPlaceNode::place_callback, this, std::placeholders::_1));

    // --- CLIENTS (Usando i servizi del Bridge) ---
    delete_client_ = create_client<ros_gz_interfaces::srv::DeleteEntity>("/world/warehouse_world/remove");
    spawn_client_ = create_client<ros_gz_interfaces::srv::SpawnEntity>("/world/warehouse_world/create");

    state_ = State::WAIT_FOR_TRIGGER;
    next_action_time_ = now() + rclcpp::Duration::from_seconds(2.0);
    timer_ = create_wall_timer(100ms, std::bind(&IiwaPickAndPlaceNode::fsm_step, this));
    
    RCLCPP_INFO(get_logger(), ">>> NODO AVVIATO: In attesa di segnale... <<<");
  }

private:
  // Stati della Macchina a Stati Finiti
  enum class State { WAIT_FOR_TRIGGER, PREGRASP, OPEN_GRIPPER, GRASP, CLOSE_GRIPPER, WAIT_FOR_GRIP, LIFT, PRE_HANDOVER, HANDOVER, OPEN_GRIPPER_RELEASE, DONE };
  
  struct Pose { std::vector<double> positions; double duration; };

  void place_callback(const std_msgs::msg::Empty::SharedPtr) {
      RCLCPP_INFO(get_logger(), ">>> TRIGGER PLACE! Eseguo operazione tramite Bridge... <<<");

      // 1. DELETE Blue box target
      auto request_del = std::make_shared<ros_gz_interfaces::srv::DeleteEntity::Request>();
      request_del->entity.name = "blue_box_target";
      request_del->entity.type = 2; // 2 = MODEL (Costante per i modelli)

      if (delete_client_->service_is_ready()) {
          delete_client_->async_send_request(request_del);
          RCLCPP_INFO(get_logger(), "🗑️ Richiesta DELETE inviata.");
      } else {
          RCLCPP_ERROR(get_logger(), "Bridge Delete non trovato! Controlla il launch file.");
      }

      // 2. SPAWN 
      auto request_spawn = std::make_shared<ros_gz_interfaces::srv::SpawnEntity::Request>();
      
      // Configurazione Entity Factory
      request_spawn->entity_factory.name = "delivered_box_R2"; 
      request_spawn->entity_factory.allow_renaming = true;
      
      // SDF String (Delivered box)
      std::string sdf_string = 
        "<sdf version='1.6'><model name='delivered_box'><static>false</static><link name='link'>"
        "<inertial><mass>0.001</mass><inertia><ixx>0.0001</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.0001</iyy><iyz>0</iyz><izz>0.0001</izz></inertia></inertial>"
        "<visual name='visual'> <geometry><box><size>0.20 0.20 0.20</size></box></geometry><material><ambient>0.3 0.5 0.7 1</ambient><diffuse>0.3 0.5 0.7 1</diffuse></material></visual>"
        "<collision name='collision'><geometry><box><size>0.20 0.20 0.20</size></box></geometry></collision>"
        "</link></model></sdf>";

      request_spawn->entity_factory.sdf = sdf_string;
      
      // Coordinate
      request_spawn->entity_factory.pose.position.x = 4.0;
      request_spawn->entity_factory.pose.position.y = -2.35;
      request_spawn->entity_factory.pose.position.z = 1.45;
      request_spawn->entity_factory.pose.orientation.w = 1.0;

      if (spawn_client_->service_is_ready()) {
          spawn_client_->async_send_request(request_spawn);
          RCLCPP_INFO(get_logger(), "📦 Richiesta SPAWN inviata!");
      } else {
          RCLCPP_ERROR(get_logger(), "Bridge Spawn non trovato!");
      }
  }
  
  void start_callback(const std_msgs::msg::Empty::SharedPtr) {
      if (state_ == State::WAIT_FOR_TRIGGER) {
          RCLCPP_INFO(get_logger(), ">>> SEGNALE RICEVUTO! Inizio... <<<");
          state_ = State::PREGRASP;
          next_action_time_ = now();
      }
  }

  void do_attach() { attach_pub_->publish(std_msgs::msg::Empty()); RCLCPP_INFO(get_logger(), "ATTACH"); }
  void do_detach() { detach_pub_->publish(std_msgs::msg::Empty()); RCLCPP_INFO(get_logger(), "DETACH"); }

  void do_fix_fra2mo() { fra2mo_attach_pub_->publish(std_msgs::msg::Empty()); RCLCPP_INFO(get_logger(), "Magnete FRA2MO: ON"); }
  void do_detach_fra2mo() { fra2mo_detach_pub_->publish(std_msgs::msg::Empty()); RCLCPP_INFO(get_logger(), "Magnete FRA2MO: OFF"); }

  bool load_yaml(const std::string &path) {
    try {
      YAML::Node yaml = YAML::LoadFile(path);
      joint_names_ = yaml["joint_names"].as<std::vector<std::string>>();
      for (const auto &it : yaml["poses"]) {
        Pose p;
        p.positions = it.second["positions"].as<std::vector<double>>();
        p.duration  = it.second["time_from_start"].as<double>();
        poses_[it.first.as<std::string>()] = p;
      }
      return true;
    } catch (const std::exception &e) { return false; }
  }

  void send_arm(const Pose &pose) {
    trajectory_msgs::msg::JointTrajectory traj; traj.joint_names = joint_names_;
    trajectory_msgs::msg::JointTrajectoryPoint pt; pt.positions = pose.positions;
    pt.time_from_start = rclcpp::Duration::from_seconds(pose.duration);
    traj.points.push_back(pt); arm_pub_->publish(traj);
  }

  void send_gripper(double left, double right) {
    trajectory_msgs::msg::JointTrajectory traj; traj.joint_names = {"left_finger_joint", "right_finger_joint"};
    trajectory_msgs::msg::JointTrajectoryPoint pt; pt.positions = {left, right};
    pt.time_from_start = rclcpp::Duration::from_seconds(1.0);
    traj.points.push_back(pt); gripper_pub_->publish(traj);
  }

  bool safety_detach_done_ = false;

  void fsm_step() {
    if (now() < next_action_time_) return;

    if (!safety_detach_done_) {
        do_detach(); do_detach_fra2mo(); 
        safety_detach_done_ = true;
        next_action_time_ = now() + rclcpp::Duration::from_seconds(1.0);
        return; 
    }

    switch (state_) {
      case State::WAIT_FOR_TRIGGER: break;

      case State::PREGRASP:
        send_arm(poses_["pregrasp"]);
        next_action_time_ = now() + rclcpp::Duration::from_seconds(poses_["pregrasp"].duration + 5.5);
        state_ = State::OPEN_GRIPPER;
        break;

      case State::OPEN_GRIPPER:
        send_gripper(0.02, -0.02); 
        next_action_time_ = now() + rclcpp::Duration::from_seconds(1.5);
        state_ = State::GRASP;
        break;

      case State::GRASP:
        send_arm(poses_["grasp"]);
        next_action_time_ = now() + rclcpp::Duration::from_seconds(poses_["grasp"].duration + 2.5);
        state_ = State::CLOSE_GRIPPER;
        break;

      case State::CLOSE_GRIPPER:
        send_gripper(-0.02, 0.02); 
        next_action_time_ = now() + rclcpp::Duration::from_seconds(2.5);
        state_ = State::WAIT_FOR_GRIP;
        break;

      case State::WAIT_FOR_GRIP:
        do_attach();
        next_action_time_ = now() + rclcpp::Duration::from_seconds(1.0);
        state_ = State::LIFT;
        break;

      case State::LIFT:
        send_arm(poses_["lift"]);
        next_action_time_ = now() + rclcpp::Duration::from_seconds(poses_["lift"].duration + 2.5);
        state_ = State::PRE_HANDOVER;
        break;
      
      case State::PRE_HANDOVER:
        send_arm(poses_["pre_handover"]);
        next_action_time_ = now() + rclcpp::Duration::from_seconds(poses_["pre_handover"].duration + 1.0);
        state_ = State::HANDOVER;
        break;

      case State::HANDOVER:
        send_arm(poses_["handover"]);
        next_action_time_ = now() + rclcpp::Duration::from_seconds(poses_["handover"].duration + 5.0);
        state_ = State::OPEN_GRIPPER_RELEASE;
        break;

      case State::OPEN_GRIPPER_RELEASE:
        do_detach(); // Detach iiwa
        do_fix_fra2mo(); // Attach Fra2mo
        send_gripper(0.02, -0.02);
        state_ = State::DONE;
        
        next_action_time_ = now() + rclcpp::Duration::from_seconds(1.0);
        break;

      case State::DONE:
        RCLCPP_INFO(get_logger(), "SEQUENCE COMPLETED. Notifying Fra2mo.");
        
        done_pub_->publish(std_msgs::msg::Empty()); 
        timer_->cancel();
        break;
    }
  }

  std::string pose_file_;
  std::vector<std::string> joint_names_;
  std::map<std::string, Pose> poses_;
  State state_;
  rclcpp::Time next_action_time_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr attach_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr detach_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr fra2mo_attach_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr fra2mo_detach_pub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_sub_;
  
  
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr done_pub_; 

  // Membri Bridge
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr place_sub_;
  rclcpp::Client<ros_gz_interfaces::srv::DeleteEntity>::SharedPtr delete_client_;
  rclcpp::Client<ros_gz_interfaces::srv::SpawnEntity>::SharedPtr spawn_client_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IiwaPickAndPlaceNode>());
  rclcpp::shutdown();
  return 0;
}

