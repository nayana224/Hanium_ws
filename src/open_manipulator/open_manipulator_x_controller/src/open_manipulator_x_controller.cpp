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
#include <memory>
#include <algorithm>

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
constexpr int      POS_TOL_TICKS   = 120;
constexpr double   GOAL_TIMEOUT_S  = 20.0;
constexpr int      POLL_PERIOD_MS  = 80;

// ---------------- Vacuum settle ----------------
constexpr double   VAC_ON_SETTLE_S  = 0.25;
constexpr double   VAC_OFF_SETTLE_S = 0.10;

// ---------------- Dwell (멈칫) ----------------
constexpr int      DWELL_MS = 0;   // 멈칫 제거

// ---------------- 기본 프로파일(모든 구간) ----------------
constexpr uint32_t PROFILE_ACC_MAIN = 100;   // 부드러운 가속
constexpr uint32_t PROFILE_VEL_MAIN = 270;   // 적당한 속도

// ---------------- 보간(일반 구간) ----------------
constexpr int      SMOOTH_STEPS     = 15;    // 분할 스텝 수 (부드러움 ↑)
constexpr int      SMOOTH_STEP_MS   = 10;    // 스텝 간 간격(ms)




// =============== Function =============== //
static inline double ticks_to_rad(uint32_t tick, int zero_ticks = 2048);




class DxlQrControlNode : public rclcpp::Node
{
public:
  DxlQrControlNode()
  : Node("dxl_qr_control_node")
  {
    // =========== Dynamixel Communication ======== //
    dxl_ids_ = {1, 2, 3, 4}; // Motor ID
    port_ = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    packet_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    if (!port_->openPort()) {
      RCLCPP_ERROR(get_logger(), "Failed to open port: %s", DEVICENAME);
      throw std::runtime_error("openPort failed");
    }
    if (!port_->setBaudRate(BAUDRATE)) {
      RCLCPP_ERROR(get_logger(), "Failed to set baudrate: %d", BAUDRATE);
      throw std::runtime_error("openPort failed");
    }

    // 1) Torque OFF () - Reset
    for (int id : dxl_ids_) {
      uint8_t dxl_err = 0;
      int rc = packet_->write1ByteTxRx(port_, id, ADDR_TORQUE_ENABLE, 0, &dxl_err);
      if (rc != COMM_SUCCESS || dxl_err) {
        RCLCPP_ERROR(get_logger(), "Torque off failed. id=%d err=%u", id, dxl_err);
      }
    } // for 1)
    // 2) Operating Mode Set (3 = Position Mode)
    for (int id : dxl_ids_) {
      uint8_t dxl_err = 0;
      int rc = packet_->write1ByteTxRx(port_, id, ADDR_OPERATING_MODE, 3, &dxl_err);
      if (rc != COMM_SUCCESS || dxl_err) {
        RCLCPP_ERROR(get_logger(), "Set mode failed. id=%d rc=%d err=%u", id, rc, dxl_err);
      }
    } // for 2)
    // 3) Profile(Accel/Vel) Set
    for (int id : dxl_ids_) {
      uint8_t dxl_err = 0;
      int rc1 = packet_->write4ByteTxRx(port_, id, ADDR_PROFILE_ACCEL, PROFILE_ACC_MAIN, &dxl_err);
      int rc2 = packet_->write4ByteTxRx(port_, id, ADDR_PROFILE_VELOCITY, PROFILE_VEL_MAIN, &dxl_err);
      if (rc1 != COMM_SUCCESS || rc2 != COMM_SUCCESS || dxl_err) {
        RCLCPP_ERROR(get_logger(), "Set profile failed. id=%d rc1=%d rc2=%d err=%u", id, rc1, rc2, dxl_err);
      }
    } // for 3)
    // 4) Torque ON
    for (int id : dxl_ids_) {
      uint8_t dxl_err = 0;
      int rc = packet_->write1ByteTxRx(port_, id, ADDR_TORQUE_ENABLE, 1, &dxl_err);
      if (rc != COMM_SUCCESS || dxl_err) {
        RCLCPP_ERROR(get_logger(), "Torque ON failed. id=%d rc=%d err=%u", id, rc, dxl_err);
      }
    } // for 4)


    // ============== Joint State Publisher & Timer ============ //
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::QoS(30));

    // joint name
    joint_names_.resize(dxl_ids_.size());
    for (size_t i = 0; i < dxl_ids_.size(); ++i) {
      joint_names_[i] = "joint" + std::to_string(i + 1);
    }
    // Period: 50Hz
    joint_timer_ = create_wall_timer(
      std::chrono::milliseconds(1000 / 50),
      std::bind(&DxlQrControlNode::publish_joint_state_timer, this));
    
    RCLCPP_INFO(get_logger(), "DXL ready. Publishing /joint_states at 50 Hz.");


    // ============== Way Points Registration ============ //
    build_builtin_sequences();

    // ============== Vacuum & Done Publisher ============= //
    vac_pub_ = create_publisher<std_msgs::msg::Bool>("/vacuum_cmd", 1);
    done_pub_ = create_publisher<std_msgs::msg::String>(done_topic_, 10);

    // ============== Subscriber ============= //
    // QR subscriber
    sub_qr_ = create_subscription<std_msgs::msg::String>(
      "/qr_info", 10,
      std::bind(&DxlQrControlNode::onQrEnqueue, this, std::placeholders::_1));
    // Ultrasonic subscriber
    sub_ultra_ = create_subscription<std_msgs::msg::Float32>(
      ultrasonic_topic_, rclcpp::QoS(10),
      std::bind(&DxlQrControlNode::onUltrasonic, this, std::placeholders::_1));
    

    // =============== Queue Timer ============= // 
    process_timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&DxlQrControlNode::try_process_queue, this));

    RCLCPP_INFO(get_logger(), "DXL ready. Subscribing /qr_info & %s. Threshold=%.2f",
                ultrasonic_topic_.c_str(), ultrasonic_thresh_);

  } // Constructor

  ~DxlQrControlNode()
  {
    RCLCPP_INFO(this->get_logger(), "Shutting down, disabling torque...");

    // 모든 모터 토크 OFF
    for (int id : dxl_ids_) {
      uint8_t dxl_err = 0;
      int rc = packet_->write1ByteTxRx(port_, id, ADDR_TORQUE_ENABLE, 0, &dxl_err);
      if (rc != COMM_SUCCESS || dxl_err) {
        RCLCPP_WARN(this->get_logger(), "Failed to disable torque for ID %d (err=%u)", id, dxl_err);
      }
    }

    // 포트 닫기 (isOpen() 없음!)
    if (port_) {
      port_->closePort();
      RCLCPP_INFO(this->get_logger(), "Port closed");
    }
  }


// Member Variables
private:
  // ==========Dynamixel Communication variables============= //
  std::vector<int> dxl_ids_; // Motor ID
  dynamixel::PortHandler *port_{nullptr}; // port handler member
  dynamixel::PacketHandler *packet_{nullptr}; // port handler members


  // ========== Joint Variables ============ //
  std::vector<std::string> joint_names_;
  // ========== Topic Name Variables ========= //
  std::string ultrasonic_topic_ = "/ultrasonic_distance";
  std::string done_topic_ = "/manipulator_done";
  // ========== State Variables ========== //
  std::string done_payload_ = "DONE";
  std::string fail_payload_ = "FAIL";
  double ultrasonic_value_{std::numeric_limits<double>::infinity()};
  double ultrasonic_thresh_{5.5};
  std::atomic<bool> vac_state_on_{false};   // 현재 공압 ON/OFF 상태 추적
  std::atomic<bool> vac_lock_off_{false};   // "시퀸스 종료까지 OFF 유지" 락
  // ★ 추가: 에지 트리거용 상태
  std::atomic<bool> was_below_{false};   // 직전 측정이 임계값 미만이었는지
  std::atomic<bool> gate_open_{false};   // 이번 하강 에지에서 1건만 허용


  
  // ========== Publisher ======== //
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_; // Joint State Publisher
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vac_pub_; // Vacuum Cmd Publish
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr done_pub_; // Opx Action Completed Publish
  // ========== Subscriber ======== //
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_qr_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_ultra_;


  // ========== Timer ========== //
  rclcpp::TimerBase::SharedPtr joint_timer_; // Joint States Timer
  rclcpp::TimerBase::SharedPtr process_timer_; // Queue Check Timer


  // ======= Bus Protect ======= // 
  std::mutex bus_mtx_; // Dynamixel Mutex
  std::mutex qr_mtx_; // QR Code Mutex

  // ======= Queue & Sync ====== //
  std::queue<std::string> qr_queue_;
  std::atomic<bool> busy_{false};
  rclcpp::Time last_trigger_time_{0, 0, RCL_ROS_TIME};
   

  // ======= Zones Sequences ====== //
  std::map<std::string, std::vector<std::vector<uint32_t>>> zones_;



// Member Functions
private:
  // ========== Read Dynamixel Present Position All ========= //
  std::vector<uint32_t> read_present_all() 
  {
    std::vector<uint32_t> out(dxl_ids_.size(), 0); // Position vector variable
    std::lock_guard<std::mutex> lk(bus_mtx_); // mutex lock start!
    for (size_t i = 0; i < dxl_ids_.size(); ++i) {
      uint8_t e = 0;
      packet_->read4ByteTxRx(port_, dxl_ids_[i], ADDR_PRESENT_POSITION, &out[i], &e);
    }
    return out;
  } // mutex Unlick Finished!!


  // ============= Joint State Timer Callback ========== //
  void publish_joint_state_timer() 
  {
    const auto ticks = read_present_all();
    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    js.name = joint_names_;
    js.position.resize(ticks.size());
    for (size_t i = 0; i < ticks.size(); ++i) {
      js.position[i] = ticks_to_rad(ticks[i], 2048);
    }

    joint_pub_->publish(js); // Publish
  }


  // =============== Way Points ============= //
  void build_builtin_sequences() 
  {
    std::vector<uint32_t> P1 = {2048, 2586, 1463, 3023};  // 처음 잡을때
    std::vector<uint32_t> P2 = {2048, 2129, 1799, 3179};  // 처음 들때
    std::vector<uint32_t> P3 = {1024, 2129, 1799, 3179};  // -90 들때
    std::vector<uint32_t> P4 = {3072, 2129, 1799, 3179};  // +90 들때
    std::vector<uint32_t> P5 = {4095, 2301, 1531, 3141};  // +180 들때
    std::vector<uint32_t> P6 = {1024, 2637, 1261, 3156};  // -90 놓을때
    std::vector<uint32_t> P7 = {3072, 2398, 1811, 2976};  // +90 놓을때
    std::vector<uint32_t> P8 = {4095, 2519, 1382, 3196};  // +180 놓을때

    zones_.clear();
    zones_["서울"] = {P1, P2, P3, P6, P3, P2};
    zones_["인천"] = {P1, P2, P4, P7, P4, P2};
    zones_["부산"] = {P1, P2, P5, P8, P5, P2};

    RCLCPP_INFO(get_logger(), "Built-in sequences set for 서울/인천/부산");
  }


  // =========== Move to Target Angle and Confirm Arrival ============ //
  bool move_to(const std::vector<uint32_t>& goal, double timeout_s = GOAL_TIMEOUT_S)
  {
    { // Scope: Mutex Lock Begin!!
      std::lock_guard<std::mutex> lk(bus_mtx_); // Mutex Lock Start -> class object
      for (size_t i = 0; i < dxl_ids_.size(); ++i) {
        uint8_t e = 0;
        packet_->write4ByteTxRx(port_, dxl_ids_[i], ADDR_GOAL_POSITION, goal[i], &e);
      }
    } // Scope: Mutex Unlock Finished!!
    

    // Waiting for goal to be reached
    auto t0 = std::chrono::steady_clock::now();
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < timeout_s) {
      bool all_ok = true;
      { // Scope: Mutex Lock Begin!!
        std::lock_guard<std::mutex> lk(bus_mtx_); // Mutex Lock Start
        for (size_t i = 0; i < dxl_ids_.size(); ++i) {
          uint32_t present = 0;
          uint8_t e = 0;
          packet_->read4ByteTxRx(port_, dxl_ids_[i], ADDR_PRESENT_POSITION, &present, &e);
          if (std::abs(int(present) - int(goal[i])) > POS_TOL_TICKS) {
            all_ok = false;
          } // if
        } // for
      } // Scope: Mutex Unlock Finished!! 

      if  (all_ok) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(POLL_PERIOD_MS));
    }
    return false;
  }


  // =============== Easing Function for Interpolation ============= // 
  static inline double ease_cos(double t)
  {
    return 0.5 - 0.5 * std::cos(M_PI * t);
  }


  // ============ Start -> Target with Cosine Easing Move ========== //
  bool move_linear_ease
  (const std::vector<uint32_t>& start, 
    const std::vector<uint32_t>& target, 
    int steps = SMOOTH_STEPS, 
    int step_ms = SMOOTH_STEP_MS)
  {
    if (start.size() != target.size() || steps <= 0) return false;
    const size_t n = start.size();

    for (int s = 1; s <= steps; ++s) {
      double t = static_cast<double>(s) / steps;
      double tt = ease_cos(t);
      { // Scope: Mutex Lock Begin!!
        std::lock_guard<std::mutex> lk(bus_mtx_);
        for (size_t i = 0; i < n; ++i) {
          double val = (1.0 - tt) * start[i] + tt * target[i];
          uint32_t mid = static_cast<uint32_t>(std::lround(val));
          uint8_t e = 0;
          packet_->write4ByteTxRx(port_, dxl_ids_[i], ADDR_GOAL_POSITION, mid, &e);
        } // for
      } // Scope: Mutex Unlock Finished!!
      std::this_thread::sleep_for(std::chrono::milliseconds(step_ms));
    }
    // Check Target Reached 
    return move_to(target, GOAL_TIMEOUT_S);
  }


  // ==================== QR Callback Function ==================== //
  void onQrEnqueue(const std_msgs::msg::String::SharedPtr msg) 
  {
    std::string key = msg->data;
    key.erase(0, key.find_first_not_of(" \t\r\n"));
    key.erase(key.find_last_not_of(" \t\r\n") + 1);
    if (key.empty()) return;

    { // Scope: Mutex Lock Begin!!
      std::lock_guard<std::mutex> lk(qr_mtx_);
      qr_queue_.push(key);
    } // Scope: Mutex Unlock Finished!!
    RCLCPP_INFO(get_logger(), "QR enqueued: '%s' (queue size=%zu)", key.c_str(), queue_size_unsafe());
  }
  size_t queue_size_unsafe() // qr_queue size check function 
  {
    std::lock_guard<std::mutex> lk(qr_mtx_);
    return qr_queue_.size();
  }


  // ======================== Ultrasonic Callback Function ===================== //
  void onUltrasonic(const std_msgs::msg::Float32::SharedPtr msg)
  {
    ultrasonic_value_ = static_cast<double>(msg->data);
    const bool below = (ultrasonic_value_ <= ultrasonic_thresh_);

    // 위(false) -> 아래(true)로 내려오는 "하강 에지"에서만 1건 허용
    bool prev = was_below_.exchange(below);
    if (below && !prev) {
      gate_open_.store(true);
    }

    // 즉시 한 번 시도
    try_process_queue();
  }
  void try_process_queue()
  {
    // already processing -> return
    if (busy_.load()) return;

    // Ultrasonic Condition Check
    if (!(ultrasonic_value_ <= ultrasonic_thresh_)) return;

    // Debounce(1 sec)
    auto now_ts = this->now();
    if (last_trigger_time_.nanoseconds() != 0 && (now_ts - last_trigger_time_).seconds() < 1.0)
    { return; }
    last_trigger_time_ = now_ts;

    // Pop one of key in queue
    std::string key;
    { // Scope: Mutex Lock Begin!!
      std::lock_guard<std::mutex> lk(qr_mtx_);
      if (qr_queue_.empty()) return;
      key = qr_queue_.front();
      qr_queue_.pop();
    } // Scope: Mutex Unlock Finished!!
    
    auto it = zones_.find(key); // Finding the right target area
    if (it == zones_.end() || it->second.size() < 2) {
      RCLCPP_WARN(get_logger(), "No zone or insufficient waypoints for key '%s'", key.c_str());
      return;
    }


    if (busy_.exchange(true)) return;

    RCLCPP_INFO(get_logger(), "Dequeued '%s' -> executing sequence (queue size now=%zu)",
                key.c_str(), queue_size_unsafe());

    std::thread([this, seq = it->second]() { // it: zone_
      bool ok = run_sequence_with_vacuum(seq);
      publish_done(ok);
      busy_ = false;
    }).detach();
  
  } // try_process_queue()


  // =================== Execute Vacuum ON/OFF =================== //
  bool run_sequence_with_vacuum(const std::vector<std::vector<uint32_t>>& seq)
  {
    // 새 시퀸스: OFF-락 해제 (필요하면 이번 시퀸스에서 ON 가능하도록)
    vac_lock_off_.store(false);

    // Start = Present Positions
    auto curr = read_present_all();

    // Set Motor Profile
    { // Mutex Lock Begin!!
      std::lock_guard<std::mutex> lk(bus_mtx_);
      for (int id : dxl_ids_) {
        uint8_t e = 0;
        packet_->write4ByteTxRx(port_, id, ADDR_PROFILE_ACCEL, PROFILE_ACC_MAIN, &e);
        packet_->write4ByteTxRx(port_, id, ADDR_PROFILE_VELOCITY, PROFILE_VEL_MAIN, &e);
      }
    } // Mutex Unlock Finished!!

    // seq: Seqeunce Position (ex) {P1, P3, P4, ...}
    for (size_t idx = 0; idx < seq.size(); ++idx) {
      const auto& target = seq[idx];

      // Immediately after reaching 1st step
      if (idx == 0) {
        if (!move_linear_ease(curr, target)) return false;
        curr = target;
        vacuum_on();
        continue;
      }

      // 4th Step -> vacuum OFF
      if (idx == 3) {
        if (!move_linear_ease(curr, target)) return false;
        curr = target;
        vacuum_off();
        continue;
      }

      if (!move_linear_ease(curr, target)) return false;
      curr = target;

      if (DWELL_MS > 0) std::this_thread::sleep_for(std::chrono::milliseconds(DWELL_MS));
    }

    /*
    if (vac_state_on_.load()) {
      vacuum_off();
    }
    */
    vac_lock_off_.store(false);
    
    RCLCPP_INFO(get_logger(), "Sequience Finished");
    return true;
  }



  // ========================== Publish Messages ========================= // 
  void vacuum_on() 
  {
    // OFF-락이 켜져 있으면 어떤 ON 명령도 무시
    if (vac_lock_off_.load()) {
      RCLCPP_WARN(get_logger(), "vacuum_on() ignored (locked OFF until sequence end)");
      return;
    }
    // 이미 ON이면 중복 퍼블리시 방지
    bool was_on = vac_state_on_.exchange(true);
    if (!was_on) {
      std_msgs::msg::Bool m; m.data = true;
      vac_pub_->publish(m);
      std::this_thread::sleep_for(std::chrono::milliseconds(int(VAC_ON_SETTLE_S * 1000)));
    }
  }
  void vacuum_off()
  {
    vac_lock_off_.store(true);
    bool was_on = vac_state_on_.exchange(false);

    if (was_on) {
      std_msgs::msg::Bool m; m.data = false;
      vac_pub_->publish(m);
      std::this_thread::sleep_for(std::chrono::milliseconds(int(VAC_OFF_SETTLE_S * 1000)));

      // 상태가 실제로 OFF로 바뀐 경우에만 문자열 알림
      std_msgs::msg::String s;
      s.data = "vacuum off";
      done_pub_->publish(s);
    }
  }


  void publish_done(bool success) 
  {
    if (!done_pub_) return;
    std_msgs::msg::String m;
    m.data = success ? done_payload_ : fail_payload_;
    done_pub_->publish(m);
  }

}; // Node Class


// =============== Global Function =============== //
static inline double ticks_to_rad(uint32_t tick, int zero_ticks)
{
  // XM430 Motor: 0~4095 <=> 0~2pi
  const double scale = (2.0 * M_PI) / 4095.0;
  int centered = static_cast<int>(tick) - zero_ticks; // center
  return centered * scale;
}




int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DxlQrControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
