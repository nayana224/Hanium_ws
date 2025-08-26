#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <unistd.h>

#include <array>
#include <string>
#include <mutex>
#include <thread>
#include <atomic>
#include <algorithm>

class OpxJointTcp : public rclcpp::Node {
public:
  OpxJointTcp() : Node("opx_joint_tcp")
  {
    // --- 파라미터 ---
    host_      = declare_parameter<std::string>("host", "10.49.212.68");  // Unity IP
    port_      = declare_parameter<int>("port", 9999);                   // Unity TCP 포트(관절)
    send_hz_   = declare_parameter<int>("send_hz", 50);                  // 전송 주기(Hz)
    n_joints_  = declare_parameter<int>("n_joints", 4);                  // 보낼 관절 개수(최대 4)

    // QR → UDP JSON 관련
    udp_json_enable_ = declare_parameter<bool>("udp_json_enable", true); // QR 이벤트 UDP JSON 전송
    udp_json_port_   = declare_parameter<int>("udp_json_port", 5005);    // Unity UdpSpawnReceiver 포트
    qr_cmd_          = declare_parameter<std::string>("qr_cmd", "spawn");// "spawn"|"belt" 등
    qr_state_        = declare_parameter<std::string>("qr_state", "");   // "on"/"off" 또는 빈 문자열

    n_joints_ = std::clamp(n_joints_, 1, 4);

    // JointState 구독: 최신 값만 캐시에 반영
    sub_joint_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::QoS(30),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lk(mtx_);
        const size_t n = std::min(static_cast<size_t>(n_joints_), msg->position.size());
        for (size_t i = 0; i < n; ++i) joints_[i] = static_cast<float>(msg->position[i]); // 라디안
        for (size_t i = n; i < static_cast<size_t>(n_joints_); ++i) joints_[i] = 0.f;
      });

    // QR 토픽 구독: 내용은 전송하지 않고, "발행됨" 신호만 UDP JSON으로 보냄
    sub_qr_ = create_subscription<std_msgs::msg::String>(
      "/qr_info", rclcpp::QoS(10),
      [this](const std_msgs::msg::String::SharedPtr /*msg*/)
      {
        if (!udp_json_enable_) return;
        // {"cmd":"spawn"} 또는 {"cmd":"belt","state":"on"} 형태
        std::string json = build_qr_json();
        send_udp_json(json);
      });

    // 송신 스레드 시작 (TCP: joint angles)
    run_.store(true);
    tx_thread_ = std::thread([this]{ tx_loop(); });

    RCLCPP_INFO(get_logger(),
      "opx_joint_tcp started (TCP %s:%d @%dHz, joints=%d | QR→UDP_JSON=%s:%d cmd=%s state=%s)",
      host_.c_str(), port_, send_hz_, n_joints_,
      udp_json_enable_ ? "ON" : "OFF", udp_json_port_,
      qr_cmd_.c_str(), (qr_state_.empty() ? "-" : qr_state_.c_str()));
  }

  ~OpxJointTcp() override {
    run_.store(false);
    if (tx_thread_.joinable()) tx_thread_.join();
    if (sock_ >= 0) ::close(sock_);
  }

private:
  // ===== TCP 전송 루프: 관절 4개(16바이트) 고정 =====
  void tx_loop() {
    if (send_hz_ < 1) send_hz_ = 1;
    rclcpp::WallRate rate(send_hz_);

    while (rclcpp::ok() && run_.load()) {
      ensure_tcp_socket();

      float out[4] = {0,0,0,0};
      {
        std::lock_guard<std::mutex> lk(mtx_);
        for (int i = 0; i < n_joints_; ++i) out[i] = joints_[i];
      }

      if (sock_ >= 0) {
        const size_t bytes = sizeof(float) * 4; // 16바이트 고정
        ssize_t sent = ::send(sock_, reinterpret_cast<const char*>(out),
                              bytes, MSG_NOSIGNAL /*SIGPIPE 방지*/);
        if (sent != static_cast<ssize_t>(bytes)) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                               "send() failed (sent=%zd). Reconnecting...", sent);
          ::close(sock_); sock_ = -1;
        }
      }

      rate.sleep();
    }
  }

  // ===== TCP 소켓 연결 보장/재접속 =====
  void ensure_tcp_socket() {
    if (sock_ >= 0) return;

    sock_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sock_ < 0) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000, "socket() failed");
      return;
    }

    int one = 1;
    ::setsockopt(sock_, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    ::setsockopt(sock_, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));

    sockaddr_in sa{};
    sa.sin_family = AF_INET;
    sa.sin_port   = htons(port_);
    if (::inet_pton(AF_INET, host_.c_str(), &sa.sin_addr) != 1) {
      RCLCPP_ERROR(get_logger(), "Invalid IP: %s", host_.c_str());
      ::close(sock_); sock_ = -1; return;
    }

    if (::connect(sock_, reinterpret_cast<sockaddr*>(&sa), sizeof(sa)) < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "connect(%s:%d) failed. Will retry...", host_.c_str(), port_);
      ::close(sock_); sock_ = -1;
    } else {
      RCLCPP_INFO(get_logger(), "Connected to Unity TCP %s:%d", host_.c_str(), port_);
    }
  }

  // ===== QR → UDP JSON 전송 =====
  std::string build_qr_json() const {
    // {"cmd":"spawn"} 또는 {"cmd":"belt","state":"on"}
    if (qr_state_.empty()) {
      return std::string{"{\"cmd\":\""} + qr_cmd_ + "\"}";
    } else {
      return std::string{"{\"cmd\":\""} + qr_cmd_ + "\",\"state\":\"" + qr_state_ + "\"}";
    }
  }

  void send_udp_json(const std::string& json) {
    int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "UDP(JSON) socket() failed");
      return;
    }

    sockaddr_in sa{};
    sa.sin_family = AF_INET;
    sa.sin_port   = htons(udp_json_port_);
    if (::inet_pton(AF_INET, host_.c_str(), &sa.sin_addr) != 1) {
      RCLCPP_ERROR(get_logger(), "Invalid UDP(JSON) IP: %s", host_.c_str());
      ::close(sock);
      return;
    }

    ssize_t sent = ::sendto(sock, json.data(), json.size(), 0,
                            reinterpret_cast<sockaddr*>(&sa), sizeof(sa));
    if (sent != static_cast<ssize_t>(json.size())) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "UDP(JSON) send failed");
    } else {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "UDP(JSON) sent: %s", json.c_str());
    }
    ::close(sock);
  }

  // ===== 멤버들 =====
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr        sub_qr_;

  std::string host_;
  int port_{9999};
  int send_hz_{50};
  int n_joints_{4};

  // QR → UDP JSON
  bool        udp_json_enable_{true};
  int         udp_json_port_{5010};
  std::string qr_cmd_{"spawn"};
  std::string qr_state_{};

  std::mutex mtx_;
  std::array<float,4> joints_{0.f,0.f,0.f,0.f};

  std::atomic<bool> run_{false};
  std::thread tx_thread_;
  int sock_{-1};
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OpxJointTcp>());
  rclcpp::shutdown();
  return 0;
}
