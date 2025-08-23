// open_manipulator_x_controller.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <yaml-cpp/yaml.h>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netinet/tcp.h>
#include <fcntl.h>
#include <errno.h>

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
#include <array>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
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

// ---------------- Dwell ----------------
constexpr int      DWELL_MS = 0;

// ---------------- 기본 프로파일 ----------------
constexpr uint32_t PROFILE_ACC_MAIN = 100;
constexpr uint32_t PROFILE_VEL_MAIN = 240;

// ---------------- 보간 ----------------
constexpr int      SMOOTH_STEPS     = 60;
constexpr int      SMOOTH_STEP_MS   = 30;

static inline std::string trim_k(const std::string& s) {
  auto a = s.find_first_not_of(" \t\r\n");
  if (a == std::string::npos) return "";
  auto b = s.find_last_not_of(" \t\r\n");
  return s.substr(a, b - a + 1);
}

class DxlQrControlNode : public rclcpp::Node
{
public:
  DxlQrControlNode()
  : Node("dxl_qr_control_node")
  {
    dxl_ids_ = {1, 2, 3, 4};

    // --- Parameters
    std::string yaml_path = this->declare_parameter<std::string>("waypoints_yaml", "config/waypoints.yaml");
    ultrasonic_topic_     = this->declare_parameter<std::string>("ultrasonic_topic", "/ultrasonic_distance");
    ultrasonic_thresh_    = this->declare_parameter<double>("ultrasonic_thresh", 5.5);

    done_topic_           = this->declare_parameter<std::string>("done_topic",   "/manipulator_done");
    done_payload_         = this->declare_parameter<std::string>("done_payload", "DONE");
    fail_payload_         = this->declare_parameter<std::string>("fail_payload", "FAIL");

    // UART 브릿지 트리거 문자열(이 문자를 퍼블리시하면 uart_node가 '3' 송신)
    vacuum_off_trigger_   = this->declare_parameter<std::string>("vacuum_off_trigger", "vacuum off");

    // Unity 송신 파라미터
    unity_host_           = this->declare_parameter<std::string>("unity_host", "10.49.212.68");
    unity_port_           = this->declare_parameter<int>("unity_port", 9999);
    unity_send_period_ms_ = this->declare_parameter<int>("unity_send_period_ms", 20);

    build_builtin_sequences();
    try { load_waypoints(yaml_path); }
    catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "YAML load failed: %s. Using built-in defaults.", e.what());
    }

    // --- DXL init
    port_   = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    packet_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!port_->openPort())  throw std::runtime_error("openPort failed");
    if (!port_->setBaudRate(BAUDRATE)) throw std::runtime_error("setBaudRate failed");

    for (int id : dxl_ids_) write1_ok(id, ADDR_OPERATING_MODE, 3);
    set_profile_all(PROFILE_ACC_MAIN, PROFILE_VEL_MAIN);
    for (int id : dxl_ids_) write1_ok(id, ADDR_TORQUE_ENABLE, 1);

    sanity_ping_all();

    // --- ROS I/F
    vac_pub_   = create_publisher<std_msgs::msg::Bool>("/vacuum_cmd", 1);
    done_pub_  = create_publisher<std_msgs::msg::String>(done_topic_, 10);
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    sub_qr_ = create_subscription<std_msgs::msg::String>(
      "/qr_info", 10, std::bind(&DxlQrControlNode::onQrEnqueue, this, std::placeholders::_1));
    sub_ultra_ = create_subscription<std_msgs::msg::Float32>(
      ultrasonic_topic_, rclcpp::QoS(10), std::bind(&DxlQrControlNode::onUltrasonic, this, std::placeholders::_1));

    // joint_states publish (가볍게, executor에서 실행)
    joint_timer_ = create_wall_timer(std::chrono::milliseconds(50),
                    std::bind(&DxlQrControlNode::publish_joint_states_from_cache, this));

    // 텔레메트리 캐시(현재값) 갱신
    telemetry_timer_ = create_wall_timer(std::chrono::milliseconds(80),
                         std::bind(&DxlQrControlNode::update_present_cache_idle, this));

    // QR 큐 처리
    process_timer_ = create_wall_timer(std::chrono::milliseconds(50),
                      std::bind(&DxlQrControlNode::try_process_queue, this));

    update_present_cache_from_bus();

    // Unity 네트워킹은 **ROS executor 밖의 전용 스레드**에서 처리
    start_unity_thread();

    RCLCPP_INFO(get_logger(), "DXL ready. done_topic=%s, unity=%s:%d",
                done_topic_.c_str(), unity_host_.c_str(), unity_port_);
  }

  ~DxlQrControlNode() override {
    stop_unity_thread();
    for (int id : dxl_ids_) write1_ok(id, ADDR_TORQUE_ENABLE, 0);
    if (port_) port_->closePort();
  }

private:
  // ---------------- Waypoints ----------------
  void build_builtin_sequences() {
    std::vector<uint32_t> P1 = {2048, 2586, 1463, 3023};
    std::vector<uint32_t> P2 = {2048, 2129, 1799, 3179};
    std::vector<uint32_t> P3 = {1024, 2129, 1799, 3179};
    std::vector<uint32_t> P4 = {3072, 2129, 1799, 3179};
    std::vector<uint32_t> P5 = {4095, 2498, 1051, 3461};
    std::vector<uint32_t> P6 = {1024, 2637, 1261, 3156};
    std::vector<uint32_t> P7 = {3072, 2398, 1811, 2976};
    std::vector<uint32_t> P8 = {4095, 2760, 1075, 3272};

    zones_.clear();
    zones_["서울"] = {P1, P2, P3, P6, P3, P2};
    zones_["인천"] = {P1, P2, P4, P7, P4, P2};
    zones_["부산"] = {P1, P2, P5, P8, P5, P2};
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
        if (!wp.IsSequence() || wp.size() != DOF) continue;
        std::vector<uint32_t> pos(DOF);
        for (std::size_t j = 0; j < DOF; ++j) pos[j] = wp[j].as<uint32_t>();
        waypoints.emplace_back(std::move(pos));
      }
      if (!waypoints.empty()) { zones_[zone] = std::move(waypoints); overridden++; }
    }
    RCLCPP_INFO(get_logger(), "Waypoints loaded, overridden=%d", overridden);
  }

  // ---------------- Low-level I/O (mutex 보호) ----------------
  bool write1_ok(int id, uint16_t addr, uint8_t val) {
    std::lock_guard<std::mutex> lk(dxl_io_mtx_);
    uint8_t dxl_err = 0; int rc = packet_->write1ByteTxRx(port_, id, addr, val, &dxl_err);
    return (rc == COMM_SUCCESS && dxl_err == 0);
  }
  bool write4_ok(int id, uint16_t addr, uint32_t val) {
    std::lock_guard<std::mutex> lk(dxl_io_mtx_);
    uint8_t dxl_err = 0; int rc = packet_->write4ByteTxRx(port_, id, addr, val, &dxl_err);
    return (rc == COMM_SUCCESS && dxl_err == 0);
  }
  bool read4_ok(int id, uint16_t addr, uint32_t &out) {
    std::lock_guard<std::mutex> lk(dxl_io_mtx_);
    uint8_t dxl_err = 0; int rc = packet_->read4ByteTxRx(port_, id, addr, &out, &dxl_err);
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
      std::lock_guard<std::mutex> lk(dxl_io_mtx_);
      uint8_t dxl_err = 0; uint16_t model = 0;
      packet_->ping(port_, id, &model, &dxl_err);
    }
  }

  std::vector<uint32_t> read_present_all() {
    std::vector<uint32_t> out(dxl_ids_.size(), 0);
    for (size_t i = 0; i < dxl_ids_.size(); ++i) read4_ok(dxl_ids_[i], ADDR_PRESENT_POSITION, out[i]);
    return out;
  }

  static inline double ease_cos(double t) { return 0.5 - 0.5 * std::cos(M_PI * t); }

  bool move_to(const std::vector<uint32_t>& goals, double timeout_s = GOAL_TIMEOUT_S) {
    for (size_t i = 0; i < dxl_ids_.size(); ++i) write4_ok(dxl_ids_[i], ADDR_GOAL_POSITION, goals[i]);

    auto t0 = std::chrono::steady_clock::now();
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < timeout_s) {
      bool all_ok = true;
      std::vector<uint32_t> snapshot(dxl_ids_.size(), 0);
      for (size_t i = 0; i < dxl_ids_.size(); ++i) {
        uint32_t present = 0; read4_ok(dxl_ids_[i], ADDR_PRESENT_POSITION, present);
        snapshot[i] = present;
        if (std::abs(int(present) - int(goals[i])) > POS_TOL_TICKS) all_ok = false;
      }
      // 캐시 갱신
      {
        std::lock_guard<std::mutex> lk(present_mtx_);
        const size_t N = std::min(snapshot.size(), size_t(4));
        for (size_t i = 0; i < N; ++i) last_present_[i] = snapshot[i];
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

    std::vector<uint32_t> mid(n);
    for (int s = 1; s <= steps; ++s) {
      double t  = static_cast<double>(s) / steps;
      double tt = ease_cos(t);
      for (size_t i = 0; i < n; ++i) {
        double val = (1.0 - tt) * start[i] + tt * target[i];
        mid[i] = static_cast<uint32_t>(std::lround(val));
        write4_ok(dxl_ids_[i], ADDR_GOAL_POSITION, mid[i]);
      }
      {
        std::lock_guard<std::mutex> lk(present_mtx_);
        const size_t N = std::min(n, size_t(4));
        for (size_t i = 0; i < N; ++i) last_present_[i] = mid[i];
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(step_ms));
    }
    return move_to(target, GOAL_TIMEOUT_S);
  }

  // ---------------- Vacuum ----------------
  void vacuum_on()  { std_msgs::msg::Bool m; m.data = true;  vac_pub_->publish(m);
                      std::this_thread::sleep_for(std::chrono::milliseconds(int(VAC_ON_SETTLE_S * 1000))); }
  void vacuum_off() { std_msgs::msg::Bool m; m.data = false; vac_pub_->publish(m);
                      std::this_thread::sleep_for(std::chrono::milliseconds(int(VAC_OFF_SETTLE_S * 1000))); }

  // 강건하게 전달: 짧은 burst로 여러 번 퍼블리시
  void publish_vacuum_off_burst() {
    if (!done_pub_) return;
    std_msgs::msg::String m; m.data = vacuum_off_trigger_;
    for (int i = 0; i < 3; ++i) {
      done_pub_->publish(m);
      RCLCPP_INFO(get_logger(), "[TX] %s (%d/3) -> %s", m.data.c_str(), i+1, done_topic_.c_str());
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

  void publish_done(bool ok) {
    if (!done_pub_) return;
    std_msgs::msg::String m; m.data = ok ? done_payload_ : fail_payload_;
    done_pub_->publish(m);
  }

  // ---------------- Subscribers & Queue ----------------
  void onQrEnqueue(const std_msgs::msg::String::SharedPtr msg) {
    const std::string key = trim_k(msg->data);
    if (key.empty()) return;
    { std::lock_guard<std::mutex> lk(qr_mtx_); qr_queue_.push(key); }
    RCLCPP_INFO(get_logger(), "QR enqueued: '%s' (q=%zu)", key.c_str(), queue_size_unsafe());
  }

  void onUltrasonic(const std_msgs::msg::Float32::SharedPtr msg) {
    ultrasonic_value_ = static_cast<double>(msg->data);
    try_process_queue();
  }

  void try_process_queue() {
    if (busy_.load()) return;
    if (!(ultrasonic_value_ <= ultrasonic_thresh_)) return;

    std::string key;
    {
      std::lock_guard<std::mutex> lk(qr_mtx_);
      if (qr_queue_.empty()) return;
      key = qr_queue_.front(); qr_queue_.pop();
    }
    auto it = zones_.find(key);
    if (it == zones_.end()) { RCLCPP_WARN(get_logger(), "No zone for '%s'", key.c_str()); return; }
    auto seq = it->second;
    if (seq.size() < 2) { RCLCPP_WARN(get_logger(), "Zone '%s' too few waypoints", key.c_str()); return; }
    if (seq.size() == 4) { seq.push_back(seq[2]); seq.push_back(seq[0]); }

    // 디바운스(1초)
    auto now = this->now();
    if (last_trigger_time_.nanoseconds() != 0 && (now - last_trigger_time_).seconds() < 1.0) return;
    last_trigger_time_ = now;

    if (busy_.exchange(true)) return;

    std::thread([this, seq, key](){
      auto curr = read_present_all();

      // 캐시 초기화
      {
        std::lock_guard<std::mutex> lk(present_mtx_);
        const size_t N = std::min(curr.size(), size_t(4));
        for (size_t i=0;i<N;++i) last_present_[i] = curr[i];
      }

      auto go = [&](const std::vector<uint32_t>& target, int steps, int step_ms)->bool {
        bool ok = move_linear_ease(curr, target, steps, step_ms);
        if (ok) curr = target;
        if (ok && DWELL_MS > 0) std::this_thread::sleep_for(std::chrono::milliseconds(DWELL_MS));
        return ok;
      };

      for (size_t idx=0; idx<seq.size(); ++idx) {
        int steps = SMOOTH_STEPS, step_ms = SMOOTH_STEP_MS;
        uint32_t acc = PROFILE_ACC_MAIN, vel = PROFILE_VEL_MAIN;
        if (idx >= 4) { steps = std::max(SMOOTH_STEPS, 80); step_ms = std::max(SMOOTH_STEP_MS, 35); }
        set_profile_all(acc, vel);

        if (!go(seq[idx], steps, step_ms)) {
          RCLCPP_ERROR(get_logger(), "Motion failed at step %zu for '%s'", idx, key.c_str());
          publish_done(false); busy_ = false; return;
        }

        if (idx == 0) vacuum_on();        // 집기 직전
        if (idx == 3) {                   // 내려놓기 도달
          vacuum_off();
          publish_vacuum_off_burst();     // -> uart_node 가 '3' 3회 송신
        }
      }

      publish_done(true);
      busy_ = false;
      RCLCPP_INFO(get_logger(), "Sequence finished for '%s'", key.c_str());
    }).detach();
  }

  // ---------------- Unity Thread (비블로킹, 병렬) ----------------
  void start_unity_thread() {
    net_run_.store(true);
    net_thr_ = std::thread([this](){ this->unity_loop(); });
  }
  void stop_unity_thread() {
    net_run_.store(false);
    if (net_thr_.joinable()) net_thr_.join();
    close_socket();
  }

  void unity_loop() {
    // 전용 스레드: 연결 + 주기 송신. 실패 시 재시도. ROS executor를 절대 막지 않음.
    while (net_run_.load()) {
      if (sock_ < 0) {
        open_and_connect_socket();
        if (sock_ < 0) { std::this_thread::sleep_for(std::chrono::milliseconds(500)); continue; }
      }

      float data[4] = {0.f,0.f,0.f,0.f};
      {
        std::lock_guard<std::mutex> lk(present_mtx_);
        for (size_t i=0;i<4;++i) data[i] = static_cast<float>(last_present_[i]); // tick 그대로
      }

      ssize_t sent = ::send(sock_, reinterpret_cast<const char*>(data),
                            sizeof(float)*4, MSG_NOSIGNAL | MSG_DONTWAIT);
      if (sent != static_cast<ssize_t>(sizeof(float)*4)) {
        // EWOULDBLOCK/EAGAIN이면 버퍼가 찬 것이므로 이번 주기는 드롭
        if (errno != EWOULDBLOCK && errno != EAGAIN) {
          RCLCPP_WARN(get_logger(), "Unity send failed (%zd, errno=%d). Reconnect.", sent, errno);
          close_socket(); // 다음 루프에서 재연결
        }
      }

      // 주기
      std::this_thread::sleep_for(std::chrono::milliseconds(unity_send_period_ms_));
    }
  }

  void open_and_connect_socket() {
    close_socket();

    int s = ::socket(AF_INET, SOCK_STREAM, 0);
    if (s < 0) { RCLCPP_ERROR(get_logger(), "socket() failed"); return; }

    // Nagle off
    int one = 1;
    ::setsockopt(s, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    // 논블로킹
    int flags = fcntl(s, F_GETFL, 0);
    fcntl(s, F_SETFL, flags | O_NONBLOCK);

    sockaddr_in sa{}; sa.sin_family = AF_INET;
    sa.sin_port = htons(static_cast<uint16_t>(unity_port_));
    if (::inet_pton(AF_INET, unity_host_.c_str(), &sa.sin_addr) != 1) {
      RCLCPP_ERROR(get_logger(), "inet_pton failed for %s", unity_host_.c_str());
      ::close(s); return;
    }

    int rc = ::connect(s, reinterpret_cast<sockaddr*>(&sa), sizeof(sa));
    if (rc == 0 || errno == EINPROGRESS) {
      sock_ = s;
      RCLCPP_INFO(get_logger(), "Unity socket ready (non-blocking) %s:%d", unity_host_.c_str(), unity_port_);
    } else {
      RCLCPP_ERROR(get_logger(), "connect() failed: errno=%d", errno);
      ::close(s);
    }
  }

  void close_socket() {
    if (sock_ >= 0) { ::close(sock_); sock_ = -1; }
  }

  // ---------------- JointState publish ----------------
  void publish_joint_states_from_cache() {
    if (!joint_pub_) return;
    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    js.name = {"joint1","joint2","joint3","joint4"};
    js.position.resize(4);
    {
      std::lock_guard<std::mutex> lk(present_mtx_);
      for (size_t i=0;i<4;++i) js.position[i] = static_cast<double>(last_present_[i]);
    }
    joint_pub_->publish(js);
  }

  // Idle 시 현재값 버스에서 갱신(모션 중엔 move_* 에서 갱신)
  void update_present_cache_idle() {
    if (busy_.load()) return;
    const auto vals = read_present_all();
    std::lock_guard<std::mutex> lk(present_mtx_);
    const size_t N = std::min(vals.size(), size_t(4));
    for (size_t i=0;i<N;++i) last_present_[i] = vals[i];
  }

  // ---------------- Utils ----------------
  size_t queue_size_unsafe() {
    std::lock_guard<std::mutex> lk(qr_mtx_);
    return qr_queue_.size();
  }
  // DXL 버스에서 현재 위치를 한 번 읽어 캐시 초기화
  void update_present_cache_from_bus() {
    const auto vals = read_present_all();        // 각 모터의 PRESENT_POSITION 읽음
    std::lock_guard<std::mutex> lk(present_mtx_);
    const size_t N = std::min(vals.size(), size_t(4));
    for (size_t i = 0; i < N; ++i) {
      last_present_[i] = vals[i];                // tick 그대로 캐시에 저장
    }
    for (size_t i = N; i < 4; ++i) {
      last_present_[i] = 0;
    }
  }

  // ---------------- Members ----------------
  std::vector<int> dxl_ids_;
  std::map<std::string, std::vector<std::vector<uint32_t>>> zones_;
  dynamixel::PortHandler *port_{nullptr};
  dynamixel::PacketHandler *packet_{nullptr};

  // ROS I/F
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr  sub_qr_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_ultra_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr       vac_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr     done_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr process_timer_;
  rclcpp::TimerBase::SharedPtr joint_timer_;
  rclcpp::TimerBase::SharedPtr telemetry_timer_;

  // 큐 & 동기화
  std::queue<std::string> qr_queue_;
  std::mutex qr_mtx_;

  // 현재값 캐시
  std::array<uint32_t,4> last_present_{ {0,0,0,0} };
  std::mutex present_mtx_;

  // DXL 보호
  std::mutex dxl_io_mtx_;

  std::atomic<bool> busy_{false};
  rclcpp::Time last_trigger_time_{0,0,RCL_ROS_TIME};

  // 초음파
  std::string ultrasonic_topic_;
  double ultrasonic_value_{std::numeric_limits<double>::infinity()};
  double ultrasonic_thresh_{5.5};

  // 완료 문자열
  std::string done_topic_;
  std::string done_payload_;
  std::string fail_payload_;
  std::string vacuum_off_trigger_;

  // Unity networking
  std::string unity_host_;
  int unity_port_{9999};
  int unity_send_period_ms_{20};
  std::atomic<bool> net_run_{false};
  std::thread net_thr_;
  int sock_{-1};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DxlQrControlNode>();

  // **중요**: 병렬 실행
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), /*number_of_threads=*/4);
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
