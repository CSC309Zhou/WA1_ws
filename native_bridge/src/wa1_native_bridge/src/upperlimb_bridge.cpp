#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>
#include <ros/ros.h>
#include <ros/message_event.h>
#include <ros/subscribe_options.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int8.h>
#include <tf2_msgs/TFMessage.h>
#include <upperlimb/Joints.h>
#include <upperlimb/SpeedJ.h>
#include <upperlimb/DualPose.h>
#include <upperlimb/SpeedL.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <upperlimb/msg/joints.hpp>
#include <upperlimb/msg/speed_j.hpp>
#include <upperlimb/msg/dual_pose.hpp>
#include <upperlimb/msg/speed_l.hpp>
#include <ros1_bridge/bridge.hpp>

using Clock = std::chrono::steady_clock;
static int64_t steady_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
}
template<class Range> static bool finite(const Range & values) {
  return std::all_of(values.begin(), values.end(), [](double v) {return std::isfinite(v);});
}
struct Part {std::string suffix; size_t size; int mask;};
struct Service {std::string suffix, package, type;};
using TfEvent = ros::MessageEvent<tf2_msgs::TFMessage const>;

class Bridge {
 public:
  Bridge() : data_(std::make_shared<rclcpp::Node>("wa1_upperlimb_bridge")) {
    rcl_interfaces::msg::ParameterDescriptor fixed;
    fixed.read_only = true;
    lift_ = data_->declare_parameter<bool>("include_lift", true, fixed);
    latency_probe_ = data_->declare_parameter<bool>("latency_probe", false, fixed);
    max_age_ms_ = data_->declare_parameter<int>("max_command_age_ms", 100, fixed);
    state_timeout_ms_ = data_->declare_parameter<int>("state_timeout_ms", 250, fixed);
    watchdog_ms_ = data_->declare_parameter<int>("watchdog_ms", 150, fixed);
    bool reliable = data_->declare_parameter<bool>("state_reliable", false, fixed);
    if (max_age_ms_ <= 0 || state_timeout_ms_ <= 0 || watchdog_ms_ < 0) {
      throw std::runtime_error("Invalid timeout configuration");
    }
    parts_ = {{"left_arm",7,1},{"right_arm",7,2},{"dual_arm",14,3},{"neck",2,4},{"waist",2,8}};
    if (lift_) {parts_.push_back({"lifting",1,16}); parts_.push_back({"whole_body",19,31});}
    std::vector<Service> services;
    for (const auto & side : {"left_arm", "right_arm"}) {
      services.push_back({std::string("FK/")+side,"upperlimb","FK"});
      services.push_back({std::string("IK/")+side,"upperlimb","IK"});
      services.push_back({std::string("is_singular/")+side,"upperlimb","IsSingular"});
    }
    for (const auto & p : parts_) {
      services.push_back({"movej/"+p.suffix,"upperlimb","MoveJ"});
      services.push_back({"movej_by_path/"+p.suffix,"upperlimb","MoveJByPath"});
    }
    for (const auto & s : {"set_servo_params", "clear_servo_params"}) {
      services.push_back({s,"upperlimb","Servo"});
    }
    services.push_back({"enable_speedj","std_srvs","SetBool"});
    services.push_back({"stop","std_srvs","Trigger"});
    services.push_back({"unlock","std_srvs","Trigger"});
    services.push_back({"enable_speedl","std_srvs","SetBool"});
    for (const auto & side : {"left_arm", "right_arm", "dual_arm"}) {
      for (const auto & mode : {"movel", "movel_by_path", "movej_by_pose"}) {
        std::string type = std::string(mode)=="movel" ? "MoveL" :
          (std::string(mode)=="movel_by_path" ? "MoveLByPath" : "MoveJByPose");
        services.push_back({std::string(mode)+"/"+side,"upperlimb",type});
      }
    }
    // These remaps affect the ROS2 service node only. ROS1 keeps its original names.
    std::vector<std::string> remaps{"--ros-args"};
    for (const auto & s : services) {
      remaps.insert(remaps.end(), {"-r", ros1_+s.suffix+":="+ros2_+s.suffix});
    }
    rclcpp::NodeOptions options;
    options.use_global_arguments(false).arguments(remaps);
    service_node_ = std::make_shared<rclcpp::Node>("wa1_upperlimb_services", options);
    stop_service_node_ = std::make_shared<rclcpp::Node>("wa1_upperlimb_stop_service", options);
    for (const auto & s : services) {
      auto factory = ros1_bridge::get_service_factory("ros1", s.package, s.type);
      if (!factory) {throw std::runtime_error("Missing service factory: "+s.type);}
      service_handles_.push_back(factory->service_bridge_2_to_1(
        ros1_node_, s.suffix=="stop" ? stop_service_node_ : service_node_, ros1_+s.suffix));
      service_factories_.push_back(std::move(factory));
    }
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile();
    if (reliable) {qos.reliable();} else {qos.best_effort();}
    struct Uplink {std::string from,to,type1,type2;};
    const std::vector<Uplink> states = {
      {"tcp_pose/left_arm","tcp_pose/left","upperlimb/Pose","upperlimb/msg/Pose"},
      {"tcp_pose/right_arm","tcp_pose/right","upperlimb/Pose","upperlimb/msg/Pose"},
      {"tcp_speed/dual_arm","tcp_speed/dual","upperlimb/TcpSpeed","upperlimb/msg/TcpSpeed"},
      {"uplimb_state","uplimb_state","upperlimb/UplimbState","upperlimb/msg/UplimbState"},
      {"occupancy_state","occupancy","std_msgs/Int8","std_msgs/msg/Int8"}};
    for (const auto & s : states) {
      uplinks_.push_back(ros1_bridge::create_bridge_from_1_to_2(
        ros1_node_, data_, s.type1, ros1_+s.from, 1, s.type2, ros2_+s.to, qos));
    }
    // /tf and /tf_static also carry navigation transforms. Keep every message
    // from the robot's upper-limb TF node, and exclude the other ROS1 publishers.
    tf_pub_ = data_->create_publisher<tf2_msgs::msg::TFMessage>(
      "/tf", rclcpp::QoS(100).reliable().durability_volatile());
    tf_static_pub_ = data_->create_publisher<tf2_msgs::msg::TFMessage>(
      "/tf_static", rclcpp::QoS(1).reliable().transient_local());
    boost::function<void(const TfEvent &)> dynamic_callback =
      [this](const TfEvent & event) {forward_tf(event, false);};
    ros::SubscribeOptions dynamic_options;
    dynamic_options.initByFullCallbackType<const TfEvent &>("/tf", 100, dynamic_callback);
    dynamic_options.transport_hints = ros::TransportHints().tcpNoDelay();
    tf_sub_ = ros1_node_.subscribe(dynamic_options);
    boost::function<void(const TfEvent &)> static_callback =
      [this](const TfEvent & event) {forward_tf(event, true);};
    ros::SubscribeOptions static_options;
    static_options.initByFullCallbackType<const TfEvent &>("/tf_static", 10, static_callback);
    static_options.transport_hints = ros::TransportHints().tcpNoDelay();
    tf_static_sub_ = ros1_node_.subscribe(static_options);
    auto joint_factory=ros1_bridge::get_factory("sensor_msgs/JointState","sensor_msgs/msg/JointState");
    auto joint_pub=data_->create_publisher<sensor_msgs::msg::JointState>(ros2_+"joint_states",qos);
    auto timing_pub=data_->create_publisher<sensor_msgs::msg::TimeReference>(ros2_+"bridge/joint_receive_time",qos);
    joints_monitor_ = ros1_node_.subscribe<sensor_msgs::JointState>(ros1_+"joint_states",1,
      [this,joint_factory,joint_pub,timing_pub](const sensor_msgs::JointState::ConstPtr & m) {
        const int64_t received=steady_ns();
        const std::set<std::string> names(m->name.begin(),m->name.end());
        if (m->name.size()==19 && names.size()==19 && m->position.size()==19 && finite(m->position)) {
          last_state_ns_=steady_ns(); ++state_count_;
        } else {last_state_ns_=0;}
        sensor_msgs::msg::JointState out;
        joint_factory->convert_1_to_2(m.get(),&out);
        joint_pub->publish(out);
        sensor_msgs::msg::TimeReference timing;
        timing.header=out.header;
        timing.time_ref.sec=received/1000000000;
        timing.time_ref.nanosec=received%1000000000;
        timing.source="PC CLOCK_MONOTONIC ROS1 callback entry; header preserves robot time";
        timing_pub->publish(timing);
      }, ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
    occupancy_monitor_ = ros1_node_.subscribe<std_msgs::Int8>(ros1_+"occupancy_state",1,
      [this](const std_msgs::Int8::ConstPtr & m) {occupancy_=m->data; last_occupancy_ns_=steady_ns();},
      ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
    if (latency_probe_) {
      command_timing_ = data_->create_publisher<std_msgs::msg::String>(
        ros2_+"bridge/servoj_left_arm_timing", rclcpp::QoS(512).best_effort());
    }
    for (const auto & p : parts_) {
      add_command<upperlimb::Joints,upperlimb::msg::Joints>("servoj/"+p.suffix,"Joints",
        [p](const auto & m){return m.joint.size()==p.size && finite(m.joint);});
      if (p.suffix!="whole_body") {
        add_command<upperlimb::SpeedJ,upperlimb::msg::SpeedJ>("speedj/"+p.suffix,"SpeedJ",
          [p](const auto & m){return m.joint_speed.size()==p.size && finite(m.joint_speed) &&
            std::isfinite(m.acc) && m.acc>0 && m.arm_type==p.mask;});
      }
    }
    for (const auto & p : std::vector<Part>{{"left_arm",6,1},{"right_arm",6,2},{"dual_arm",12,3}}) {
      add_command<upperlimb::SpeedL,upperlimb::msg::SpeedL>("speedl/"+p.suffix,"SpeedL",
        [p](const auto & m){return m.tcp_speed.size()==p.size && finite(m.tcp_speed) && std::isfinite(m.acc) && m.acc>0;});
      add_command<upperlimb::DualPose,upperlimb::msg::DualPose>("servol/"+p.suffix,"DualPose",
        [p](const auto & m){return (!(p.mask&1)||pose_valid(m.left_arm_pose)) &&
          (!(p.mask&2)||pose_valid(m.right_arm_pose));});
    }
    stop_client_=data_->create_client<std_srvs::srv::Trigger>(ros2_+"stop");
    reset_=data_->create_service<std_srvs::srv::Trigger>(ros2_+"bridge/reset_fault",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> r){
        r->success=feedback_ready();
        if (r->success) {last_command_ns_=0; fault_=false;}
        r->message=r->success ? "stream fault reset; no command was sent" : "feedback stale or occupied";
      });
    status_=data_->create_publisher<std_msgs::msg::String>(ros2_+"bridge/status",rclcpp::QoS(1).transient_local());
    timer_=data_->create_wall_timer(std::chrono::milliseconds(10),[this](){tick();});
    RCLCPP_INFO(data_->get_logger(),"WA1 bridge ready: lift=%s; fixed upperlimb whitelist; queue=1",
      lift_?"true":"false");
  }
  void spin() {
    ros::AsyncSpinner ros_spinner(2); ros_spinner.start();
    rclcpp::executors::SingleThreadedExecutor service_executor;
    rclcpp::executors::SingleThreadedExecutor stop_executor;
    service_executor.add_node(service_node_);
    stop_executor.add_node(stop_service_node_);
    auto run_service=[this](rclcpp::executors::SingleThreadedExecutor & executor){
      while (rclcpp::ok()) {
        try {executor.spin(); break;}
        catch (const std::exception & e) {
          RCLCPP_ERROR(data_->get_logger(),"ROS1 service forwarding failed: %s; data plane remains active",e.what());
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
      }
    };
    std::thread service_thread([&](){run_service(service_executor);});
    std::thread stop_thread([&](){run_service(stop_executor);});
    rclcpp::spin(data_);
    service_executor.cancel(); stop_executor.cancel(); service_thread.join(); stop_thread.join();
    ros_spinner.stop(); ros::shutdown();
  }
 private:
  void forward_tf(const TfEvent & event, bool is_static) {
    if (event.getPublisherName() != "/zj_humanoid/uplimb_tf_publisher_node") {return;}
    const auto source = event.getConstMessage();
    if (!source || source->transforms.empty()) {return;}
    tf2_msgs::msg::TFMessage out;
    out.transforms.reserve(source->transforms.size());
    for (const auto & transform : source->transforms) {
      geometry_msgs::msg::TransformStamped converted;
      converted.header.stamp.sec = static_cast<int32_t>(transform.header.stamp.sec);
      converted.header.stamp.nanosec = transform.header.stamp.nsec;
      converted.header.frame_id = transform.header.frame_id;
      converted.child_frame_id = transform.child_frame_id;
      converted.transform.translation.x = transform.transform.translation.x;
      converted.transform.translation.y = transform.transform.translation.y;
      converted.transform.translation.z = transform.transform.translation.z;
      converted.transform.rotation.x = transform.transform.rotation.x;
      converted.transform.rotation.y = transform.transform.rotation.y;
      converted.transform.rotation.z = transform.transform.rotation.z;
      converted.transform.rotation.w = transform.transform.rotation.w;
      out.transforms.push_back(std::move(converted));
    }
    if (is_static) {
      // A late ROS2 listener receives the complete upper-limb static tree.
      for (const auto & transform : out.transforms) {
        static_tf_cache_[transform.child_frame_id] = transform;
      }
      out.transforms.clear();
      for (const auto & item : static_tf_cache_) {out.transforms.push_back(item.second);}
      tf_static_pub_->publish(out);
    } else {tf_pub_->publish(out);}
  }
  template<class P> static bool pose_valid(const P & p) {
    const auto & q=p.orientation; const auto & v=p.position;
    const double n=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z)&&std::isfinite(n)&&std::abs(n-1)<0.02;
  }
  bool feedback_ready() const {
    const int64_t now=steady_ns(), ttl=int64_t(state_timeout_ms_)*1000000;
    return now-last_state_ns_.load()<ttl && now-last_occupancy_ns_.load()<ttl && occupancy_.load()==0;
  }
  template<class R1,class R2,class Validator> void add_command(const std::string & suffix,const std::string & type,Validator valid) {
    auto factory=ros1_bridge::get_factory("upperlimb/"+type,"upperlimb/msg/"+type);
    auto pub=ros1_node_.advertise<R1>(ros1_+suffix,1,false);
    auto qos=rclcpp::QoS(1).reliable().durability_volatile();
    auto sub=data_->create_subscription<R2>("/cmd/upperlimb/"+suffix,qos,
      [this,pub,factory,valid,suffix](typename R2::ConstSharedPtr msg,const rclcpp::MessageInfo & info){
        const bool probe = latency_probe_ && suffix=="servoj/left_arm";
        const int64_t callback_entry_ns = probe ? steady_ns() : 0;
        const int64_t source=info.get_rmw_message_info().source_timestamp;
        const int64_t now=std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count();
        if (fault_ || !feedback_ready() || !valid(*msg) || source<=0 || std::abs(now-source)>int64_t(max_age_ms_)*1000000) {
          ++dropped_; return;
        }
        R1 out;
        factory->convert_2_to_1(msg.get(),&out);
        const int64_t ros1_publish_start_ns = probe ? steady_ns() : 0;
        pub.publish(out);
        const int64_t ros1_publish_return_ns = probe ? steady_ns() : 0;
        last_command_ns_=steady_ns(); ++sent_;
        if constexpr (std::is_same_v<R2, upperlimb::msg::Joints>) {
          if (probe && msg->joint.size()>4) {
            uint64_t target_bits=0;
            static_assert(sizeof(target_bits)==sizeof(double));
            std::memcpy(&target_bits, &msg->joint[4], sizeof(target_bits));
            char json[512];
            const int length=std::snprintf(json,sizeof(json),
              "{\"target_bits\":\"%016llx\",\"callback_entry_ns\":%lld,"
              "\"ros1_publish_start_ns\":%lld,\"ros1_publish_return_ns\":%lld,"
              "\"dds_source_timestamp_ns\":%lld}",
              static_cast<unsigned long long>(target_bits),
              static_cast<long long>(callback_entry_ns),
              static_cast<long long>(ros1_publish_start_ns),
              static_cast<long long>(ros1_publish_return_ns),
              static_cast<long long>(source));
            if (length>0 && static_cast<size_t>(length)<sizeof(json)) {
              std_msgs::msg::String timing;
              timing.data.assign(json, static_cast<size_t>(length));
              command_timing_->publish(timing);
            }
          }
        }
      });
    commands_.push_back(sub); command_publishers_.push_back(pub);
  }
  void tick() {
    const int64_t now=steady_ns(), last=last_command_ns_.load();
    if (last>0 && !fault_ &&
      (!feedback_ready() || (watchdog_ms_>0 && now-last>int64_t(watchdog_ms_)*1000000))) {
      fault_=true;
      RCLCPP_ERROR(data_->get_logger(),"Stream watchdog latched; requesting upperlimb stop once; explicit reset required");
      if (stop_client_->service_is_ready()) {
        stop_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),
          [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture f){
            RCLCPP_WARN(data_->get_logger(),"Stop request returned success=%d: %s",f.get()->success,f.get()->message.c_str());
          });
      } else {RCLCPP_ERROR(data_->get_logger(),"Stop service unavailable; robot stop is NOT confirmed");}
    }
    if (++ticks_%100==0) {
      std::string connections="{";
      for (size_t i=0;i<command_publishers_.size();++i) {
        const auto & p=command_publishers_[i];
        if (i) {connections+=",";}
        connections+="\""+p.getTopic()+"\":"+std::to_string(p.getNumSubscribers());
      }
      connections+="}";
      std_msgs::msg::String m;
      // Keep this field until the existing teleop and operator tools migrate.
      m.data=std::string("{\"read_only\":false")+
        ",\"ros1_master_uri\":\""+ros::master::getURI()+"\""+
        ",\"feedback_ready\":"+(feedback_ready()?"true":"false")+
        ",\"stream_fault\":"+(fault_?"true":"false")+
        ",\"state_count\":"+std::to_string(state_count_.load())+
        ",\"sent\":"+std::to_string(sent_.load())+",\"dropped\":"+std::to_string(dropped_.load())+
        ",\"ros1_command_connections\":"+connections+"}";
      status_->publish(m);
    }
  }
  const std::string ros1_="/zj_humanoid/upperlimb/", ros2_="/upperlimb/";
  ros::NodeHandle ros1_node_;
  rclcpp::Node::SharedPtr data_,service_node_,stop_service_node_;
  bool lift_,latency_probe_;
  int max_age_ms_,state_timeout_ms_,watchdog_ms_,ticks_=0;
  std::vector<Part> parts_;
  std::atomic<int64_t> last_state_ns_{0},last_occupancy_ns_{0},last_command_ns_{0};
  std::atomic<int> occupancy_{-1};
  std::atomic<bool> fault_{false};
  std::atomic<uint64_t> state_count_{0},sent_{0},dropped_{0};
  std::vector<ros1_bridge::Bridge1to2Handles> uplinks_;
  std::vector<std::unique_ptr<ros1_bridge::ServiceFactoryInterface>> service_factories_;
  std::vector<ros1_bridge::ServiceBridge2to1> service_handles_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> commands_;
  std::vector<ros::Publisher> command_publishers_;
  ros::Subscriber joints_monitor_,occupancy_monitor_,tf_sub_,tf_static_sub_;
  std::map<std::string,geometry_msgs::msg::TransformStamped> static_tf_cache_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_,tf_static_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_timing_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc,char ** argv) {
  // ROS2 remaps/parameters must not leak into ROS1's naming or parameter server.
  int ros_argc=1; char * ros_argv[]={argv[0],nullptr};
  ros::init(ros_argc,ros_argv,"wa1_upperlimb_bridge",ros::init_options::NoSigintHandler);
  rclcpp::init(argc,argv);
  try {Bridge bridge; bridge.spin();}
  catch (const std::exception & e) {
    fprintf(stderr,"WA1 bridge failed: %s\n",e.what()); rclcpp::shutdown(); ros::shutdown(); return 1;
  }
  rclcpp::shutdown();
  return 0;
}
