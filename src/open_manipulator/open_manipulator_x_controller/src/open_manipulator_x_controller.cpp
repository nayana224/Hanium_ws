#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <yaml-cpp/yaml.h>

#include <iostream>
#include <map>
#include <vector>
#include <string>
#include <thread>
#include <atomic>
#include <chrono>
#include <cmath>
#include <queue>
#include <mutex>
#include <limits>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ---------------- Dynamixel Control Table (XM430) ----------------
constexpr uint16_t ADDR_OPERATING_MODE    = 11;   // 3 = Position mode
constexpr uint16_t ADDR_TORQUE_ENABLE     = 64;   // 1=ON, 0=OFF
constexpr uint16_t ADDR_PROFILE_ACCEL     = 108;  // 4 bytes
constexpr uint16_t ADDR_PROFILE_VELOCITY  = 112;  // 4 bytes
constexpr uint16_t ADDR_GOAL_POSITION     = 116;  // 4 bytes
constexpr uint16_t ADDR_PRESENT_POSITION  = 132;  // 4 bytes

// ---------------- Serial/Protocol ----------------
constexpr double   PROTOCOL_VERSION = 2.0;
constexpr int      BAUDRATE         = 1000000;
constexpr char     DEVICENAME[]     = "/dev/ttyUSB0";

// ---------------- Safety / Motion Settings ----------------
constexpr int      POS_TOL_TICKS = 120;
constexpr double   GOAL_TIMEOUT_S = 20.0;
constexpr int      POLL_PERIOD_MS = 80;

// ---------------- Vacuum settle ----------------
constexpr double   VAC_ON_SETTLE_S  = 0.25;
constexpr double   VAC_OFF_SETTLE_S = 0.10;

// ---------------- Dwell (멈칫) ----------------
constexpr int      DWELL_MS = 0;   // 멈칫 제거

// ---------------- 기본 프로파일(모든 구간) ----------------
constexpr uint32_t PROFILE_ACC_MAIN = 100;   // 부드러운 가속
constexpr uint32_t PROFILE_VEL_MAIN = 240;   // 적당한 속도

// ---------------- 보간(일반 구간) ----------------
constexpr int      SMOOTH_STEPS     = 60;    // 분할 스텝 수 (부드러움 ↑)
constexpr int      SMOOTH_STEP_MS   = 30;    // 스텝 간 간격(ms)

// 공백 제거
static inline std::string trim_k(const std::string& s) {
  auto start = s.find_first_not_of(" \t\r\n");
  if (start == std::string::npos) return "";
  auto end = s.find_last_not_of(" \t\r\n");
  return s.substr(start, end - start + 1);
}

class DxlQrControlNode : public rclcpp::Node
{
public:
  DxlQrControlNode()
  : Node("dxl_qr_control_node")
  {
    dxl_ids_ = {1, 2, 3, 4};  // 모터 ID

    // --- 파라미터
    std::string yaml_path = this->declare_parameter<std::string>("waypoints_yaml", "config/waypoints.yaml");
    ultrasonic_topic_     = this->declare_parameter<std::string>("ultrasonic_topic", "/ultrasonic_distance");
    ultrasonic_thresh_    = this->declare_parameter<double>("ultrasonic_thresh", 5.5);

    // JointState 송출 관련 파라미터
    joint_rate_hz_        = this->declare_parameter<int>("joint_state_rate_hz", 50);
    joint_zero_ticks_     = this->declare_parameter<int>("joint_state_zero_ticks", 2048);
    joint_use_center_     = this->declare_parameter<bool>("joint_state_use_center", true);

    joint_names_ = this->declare_parameter<std::vector<std::string>>(
      "joint_names", std::vector<std::string>{});
    if (joint_names_.empty()) {
      joint_names_.resize(dxl_ids_.size());
      for (size_t i=0;i<dxl_ids_.size();++i) joint_names_[i] = "joint" + std::to_string(i+1);
    }

    // 완료/실패/임의 알림용 토픽
    done_topic_   = this->declare_parameter<std::string>("done_topic",   "/manipulator_done");
    done_payload_ = this->declare_parameter<std::string>("done_payload", "DONE");
    fail_payload_ = this->declare_parameter<std::string>("fail_payload", "FAIL");

    build_builtin_sequences();
    try { load_waypoints(yaml_path); }
    catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "YAML load failed or skipped: %s. Using built-in defaults.", e.what());
    }

    // --- DXL 초기화
    port_   = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    packet_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!port_->openPort()) {
      RCLCPP_ERROR(get_logger(), "Failed to open port: %s", DEVICENAME);
      throw std::runtime_error("openPort failed");
    }
    if (!port_->setBaudRate(BAUDRATE)) {
      RCLCPP_ERROR(get_logger(), "Failed to set baudrate: %d", BAUDRATE);
      throw std::runtime_error("setBaudRate failed");
    }

    for (int id : dxl_ids_) write1_ok(id, ADDR_OPERATING_MODE, 3);  // Position
    set_profile_all(PROFILE_ACC_MAIN, PROFILE_VEL_MAIN);
    for (int id : dxl_ids_) write1_ok(id, ADDR_TORQUE_ENABLE, 1);

    sanity_ping_all();

    // 퍼블리셔/구독/타이머
    vac_pub_  = create_publisher<std_msgs::msg::Bool>("/vacuum_cmd", 1);
    done_pub_ = create_publisher<std_msgs::msg::String>(done_topic_, 10);

    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::QoS(30));
    if (joint_rate_hz_ < 1) joint_rate_hz_ = 1;
    joint_timer_ = create_wall_timer(
      std::chrono::milliseconds(1000 / joint_rate_hz_),
      std::bind(&DxlQrControlNode::publish_joint_state_timer, this));

    sub_qr_ = create_subscription<std_msgs::msg::String>(
      "/qr_info", 10,
      std::bind(&DxlQrControlNode::onQrEnqueue, this, std::placeholders::_1));

    sub_ultra_ = create_subscription<std_msgs::msg::Float32>(
      ultrasonic_topic_, rclcpp::QoS(10),
      std::bind(&DxlQrControlNode::onUltrasonic, this, std::placeholders::_1));

    process_timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&DxlQrControlNode::try_process_queue, this));

    RCLCPP_INFO(get_logger(),
      "DXL ready. Subscribing /qr_info & %s ... JointState @ %d Hz (center=%s, zero=%d)",
      ultrasonic_topic_.c_str(), joint_rate_hz_, joint_use_center_?"true":"false", joint_zero_ticks_);
  }

  ~DxlQrControlNode() override {
    for (int id : dxl_ids_) write1_ok(id, ADDR_TORQUE_ENABLE, 0);
    port_->closePort();
  }

private:
  // ---------------- Waypoints ----------------
  void build_builtin_sequences() {
    std::vector<uint32_t> P1 = {2048, 2586, 1463, 3023};  // 처음 잡을때
    std::vector<uint32_t> P2 = {2048, 2129, 1799, 3179};  // 처음 들때
    std::vector<uint32_t> P3 = {1024, 2129, 1799, 3179};  // -90 들때
    std::vector<uint32_t> P4 = {3072, 2129, 1799, 3179};  // +90 들때
    std::vector<uint32_t> P5 = {4095, 2498, 1051, 3461};  // +180 들때
    std::vector<uint32_t> P6 = {1024, 2637, 1261, 3156};  // -90 놓을때
    std::vector<uint32_t> P7 = {3072, 2398, 1811, 2976};  // +90 놓을때
    std::vector<uint32_t> P8 = {4095, 2760, 1075, 3272};  // +180 놓을때

    zones_.clear();
    zones_["서울"]  = {P1, P2, P3, P6, P3, P2};
    zones_["인천"]  = {P1, P2, P4, P7, P4, P2};
    zones_["부산"]  = {P1, P2, P5, P8, P5, P2};

    RCLCPP_INFO(get_logger(), "Built-in sequences set for 서울/인천/부산.");
  }

  void load_waypoints(const std::string &yaml_path) {
    YAML::Node config = YAML::LoadFile(yaml_path);
    const std::size_t DOF = dxl_ids_.size();

    int overridden = 0;
    for (auto it = config.begin(); it != config.end(); ++it) {
      const std::string zone = it->first.as<std::string>();
      const YAML::Node& arr = it->second;
      if (!arr.IsSequence()) continue;

      std::vector<std::vector<uint32_t>> waypoints;
      waypoints.reserve(arr.size());
      for (std::size_t i = 0; i < arr.size(); ++i) {
        const YAML::Node& wp = arr[i];
        if (!wp.IsSequence() || wp.size() != DOF) {
          RCLCPP_WARN(get_logger(),
            "Zone '%s' waypoint[%zu] shape invalid. Skipped.", zone.c_str(), i);
          continue;
        }
        std::vector<uint32_t> pos(DOF);
        for (std::size_t j = 0; j < DOF; ++j) pos[j] = wp[j].as<uint32_t>();
        waypoints.emplace_back(std::move(pos));
      }
      if (!waypoints.empty()) { zones_[zone] = std::move(waypoints); overridden++; }
    }
    RCLCPP_INFO(get_logger(), "Waypoints loaded from %s, overridden zones=%d",
                yaml_path.c_str(), overridden);
  }

  // ---------------- Low-level I/O (★전역 뮤텍스 적용) ----------------
  bool write1_ok(int id, uint16_t addr, uint8_t val) {
    std::lock_guard<std::mutex> lk(dxl_bus_mtx_);
    uint8_t dxl_err = 0;
    int rc = packet_->write1ByteTxRx(port_, id, addr, val, &dxl_err);
    if (rc != COMM_SUCCESS || dxl_err != 0)
      RCLCPP_WARN(get_logger(), "DXL write1 fail id=%d addr=%u rc=%d err=%u", id, addr, rc, dxl_err);
    return (rc == COMM_SUCCESS && dxl_err == 0);
  }
  bool write4_ok(int id, uint16_t addr, uint32_t val) {
    std::lock_guard<std::mutex> lk(dxl_bus_mtx_);
    uint8_t dxl_err = 0;
    int rc = packet_->write4ByteTxRx(port_, id, addr, val, &dxl_err);
    if (rc != COMM_SUCCESS || dxl_err != 0)
      RCLCPP_WARN(get_logger(), "DXL write4 fail id=%d addr=%u rc=%d err=%u", id, addr, rc, dxl_err);
    return (rc == COMM_SUCCESS && dxl_err == 0);
  }
  bool read4_ok(int id, uint16_t addr, uint32_t &out) {
    std::lock_guard<std::mutex> lk(dxl_bus_mtx_);
    uint8_t dxl_err = 0;
    int rc = packet_->read4ByteTxRx(port_, id, addr, &out, &dxl_err);
    if (rc != COMM_SUCCESS || dxl_err != 0)
      RCLCPP_WARN(get_logger(), "DXL read4 fail id=%d addr=%u rc=%d err=%u", id, addr, rc, dxl_err);
    return (rc == COMM_SUCCESS && dxl_err == 0);
  }

  void set_profile_all(uint32_t acc, uint32_t vel) {
    for (int id : dxl_ids_) {
      write4_ok(id, ADDR_PROFILE_ACCEL,    acc);
      write4_ok(id, ADDR_PROFILE_VELOCITY, vel);
    }
  }

  void sanity_ping_all() {
    for (int id : dxl_ids_) {
      std::lock_guard<std::mutex> lk(dxl_bus_mtx_);
      uint8_t dxl_err = 0; uint16_t model = 0;
      packet_->ping(port_, id, &model, &dxl_err);
    }
  }

  std::vector<uint32_t> read_present_all() {
    std::vector<uint32_t> out(dxl_ids_.size(), 0);
    for (size_t i = 0; i < dxl_ids_.size(); ++i)
      read4_ok(dxl_ids_[i], ADDR_PRESENT_POSITION, out[i]);
    return out;
  }

  static inline double ease_cos(double t) {
    return 0.5 - 0.5 * std::cos(M_PI * t);
  }

  // ticks -> radians
  inline double ticks_to_rad(uint32_t tick) const {
    const double scale = (2.0 * M_PI) / 4095.0;
    double centered = joint_use_center_ ? (static_cast<int>(tick) - joint_zero_ticks_) : static_cast<double>(tick);
    return centered * scale;
  }

  bool move_to(const std::vector<uint32_t>& goals, double timeout_s = GOAL_TIMEOUT_S) {
    // 목표 쓰기 실패 시 즉시 실패
    for (size_t i = 0; i < dxl_ids_.size(); ++i)
      if (!write4_ok(dxl_ids_[i], ADDR_GOAL_POSITION, goals[i])) return false;

    auto t0 = std::chrono::steady_clock::now();
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < timeout_s) {
      bool all_ok = true;
      for (size_t i = 0; i < dxl_ids_.size(); ++i) {
        uint32_t present = 0; read4_ok(dxl_ids_[i], ADDR_PRESENT_POSITION, present);
        if (std::abs(int(present) - int(goals[i])) > POS_TOL_TICKS) all_ok = false;
      }
      if (all_ok) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(POLL_PERIOD_MS));
    }
    return false;
  }

  bool move_linear_ease(const std::vector<uint32_t>& start,
                        const std::vector<uint32_t>& target,
                        int steps, int step_ms) {
    if (start.size() != target.size() || steps <= 0) return false;
    const size_t n = start.size();

    for (int s = 1; s <= steps; ++s) {
      double t  = static_cast<double>(s) / steps;
      double tt = ease_cos(t);
      for (size_t i = 0; i < n; ++i) {
        double val = (1.0 - tt) * start[i] + tt * target[i];
        uint32_t mid = static_cast<uint32_t>(std::lround(val));
        if (!write4_ok(dxl_ids_[i], ADDR_GOAL_POSITION, mid)) return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(step_ms));
    }
    return move_to(target, GOAL_TIMEOUT_S);
  }

  void vacuum_on() {
    std_msgs::msg::Bool m; m.data = true;
    vac_pub_->publish(m);
    std::this_thread::sleep_for(std::chrono::milliseconds(int(VAC_ON_SETTLE_S * 1000)));
  }
  void vacuum_off() {
    std_msgs::msg::Bool m; m.data = false;
    vac_pub_->publish(m);
    std::this_thread::sleep_for(std::chrono::milliseconds(int(VAC_OFF_SETTLE_S * 1000)));
  }

  // -------- JointState 주기 퍼블리시 --------
  void publish_joint_state_timer() {
    const auto ticks = read_present_all();
    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    js.name         = joint_names_;
    js.position.resize(ticks.size());
    for (size_t i=0;i<ticks.size();++i) js.position[i] = ticks_to_rad(ticks[i]);
    joint_pub_->publish(js);
  }

  // ---------------- Subscribers & Queue ----------------
  void onQrEnqueue(const std_msgs::msg::String::SharedPtr msg) {
    const std::string key = trim_k(msg->data);
    if (key.empty()) return;
    { std::lock_guard<std::mutex> lk(qr_mtx_); qr_queue_.push(key); }
    RCLCPP_INFO(get_logger(), "QR enqueued: '%s' (queue size=%zu)", key.c_str(), queue_size_unsafe());
  }

  void onUltrasonic(const std_msgs::msg::Float32::SharedPtr msg) {
    ultrasonic_value_ = static_cast<double>(msg->data);
    try_process_queue();
  }

  void publish_done(bool success, const std::string& key) {
    (void)key;
    if (!done_pub_) return;
    std_msgs::msg::String m;
    m.data = success ? done_payload_ : fail_payload_;
    done_pub_->publish(m);
  }

  void try_process_queue() {
    if (busy_.load()) return;
    if (!(ultrasonic_value_ <= ultrasonic_thresh_)) return;

    std::string key;
    {
      std::lock_guard<std::mutex> lk(qr_mtx_);
      if (qr_queue_.empty()) return;
      key = qr_queue_.front();
      qr_queue_.pop();
    }

    auto it = zones_.find(key);
    if (it == zones_.end()) {
      RCLCPP_WARN(get_logger(), "No zone for key '%s'. Ignored.", key.c_str());
      return;
    }
    auto seq = it->second;
    if (seq.size() < 2) {
      RCLCPP_WARN(get_logger(), "Zone '%s' has insufficient waypoints.", key.c_str());
      return;
    }
    if (seq.size() == 4) { seq.push_back(seq[2]); seq.push_back(seq[0]); }

    // 디바운스(1초)
    auto now_ts = this->now();
    if (last_trigger_time_.nanoseconds() != 0 &&
        (now_ts - last_trigger_time_).seconds() < 1.0) {
      RCLCPP_DEBUG(get_logger(), "Debounced trigger.");
      return;
    }
    last_trigger_time_ = now_ts;

    if (busy_.exchange(true)) return;

    RCLCPP_INFO(get_logger(), "Dequeued '%s' → executing sequence (queue size now=%zu)",
                key.c_str(), queue_size_unsafe());

    std::thread([this, seq, key](){
      auto curr = read_present_all();

      auto go = [&](const std::vector<uint32_t>& target, int steps, int step_ms)->bool {
        bool ok = move_linear_ease(curr, target, steps, step_ms);
        if (ok) curr = target;
        if (ok && DWELL_MS > 0) std::this_thread::sleep_for(std::chrono::milliseconds(DWELL_MS));
        return ok;
      };

      for (size_t idx=0; idx<seq.size(); ++idx) {
        int steps   = SMOOTH_STEPS;
        int step_ms = SMOOTH_STEP_MS;
        uint32_t acc = PROFILE_ACC_MAIN, vel = PROFILE_VEL_MAIN;
        if (idx >= 4) {
          steps   = std::max(SMOOTH_STEPS, 80);
          step_ms = std::max(SMOOTH_STEP_MS, 35);
        }
        set_profile_all(acc, vel);
        if (!go(seq[idx], steps, step_ms)) {
          RCLCPP_ERROR(get_logger(), "Motion failed at step %zu for zone '%s'", idx, key.c_str());
          publish_done(false, key);
          busy_ = false;
          return;
        }

        if (idx == 0) {
          // 집기 직전: 진공 ON
          vacuum_on();
        }

        if (idx == 3) {
          // 내려놓는 위치에 도달: 진공 OFF + uart_node로 알림("vacuum off")
          vacuum_off();
          std_msgs::msg::String m; m.data = "vacuum off";
          done_pub_->publish(m);
        }
      }

      busy_ = false;
      RCLCPP_INFO(get_logger(), "Sequence finished for '%s'", key.c_str());
      publish_done(true, key);
    }).detach();
  }

  // ---------------- Utils ----------------
  size_t queue_size_unsafe() {
    std::lock_guard<std::mutex> lk(qr_mtx_);
    return qr_queue_.size();
  }

  // ---------------- Members ----------------
  std::vector<int> dxl_ids_;
  std::map<std::string, std::vector<std::vector<uint32_t>>> zones_;
  dynamixel::PortHandler *port_{nullptr};
  dynamixel::PacketHandler *packet_{nullptr};

  // ★ DXL 버스 보호용 전역 뮤텍스
  std::mutex dxl_bus_mtx_;

  // 퍼블리셔/구독/타이머
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr          vac_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr        done_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr     sub_qr_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr    sub_ultra_;
  rclcpp::TimerBase::SharedPtr                                process_timer_;
  rclcpp::TimerBase::SharedPtr                                joint_timer_;

  // JointState 설정
  std::vector<std::string> joint_names_;
  int   joint_rate_hz_{50};
  int   joint_zero_ticks_{2048};
  bool  joint_use_center_{true};

  // 큐 & 동기화
  std::queue<std::string> qr_queue_;
  std::mutex qr_mtx_;

  std::atomic<bool> busy_{false};
  rclcpp::Time last_trigger_time_{0,0,RCL_ROS_TIME};

  // 초음파
  std::string ultrasonic_topic_;
  double ultrasonic_value_{std::numeric_limits<double>::infinity()};
  double ultrasonic_thresh_{5.5};

  // 문자열 알림 퍼블리셔용
  std::string done_topic_;
  std::string done_payload_;
  std::string fail_payload_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DxlQrControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
