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
    udp_json_enable_ = declare_parameter<bool>("udp_json_enable", true);
    udp_json_port_   = declare_parameter<int>("udp_json_port", 5005);
    qr_cmd_          = declare_parameter<std::string>("qr_cmd", "spawn");
    qr_state_        = declare_parameter<std::string>("qr_state", "");

    n_joints_ = std::clamp(n_joints_, 1, 4);

    // JointState 구독
    sub_joint_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::QoS(30),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lk(mtx_);
        const size_t n = std::min(static_cast<size_t>(n_joints_), msg->position.size());
        for (size_t i = 0; i < n; ++i) joints_[i] = static_cast<float>(msg->position[i]);
        for (size_t i = n; i < static_cast<size_t>(n_joints_); ++i) joints_[i] = 0.f;
      }); // Receive and Reset

    // QR 토픽 구독 -> Spawning Box
    sub_qr_ = create_subscription<std_msgs::msg::String>(
      "/qr_info", rclcpp::QoS(10),
      [this](const std_msgs::msg::String::SharedPtr)
      {
        if (!udp_json_enable_) return;
        std::string json = build_qr_json();
        send_udp_json(json);
      });

    // manipulator_done 구독 → "vacuum off" 시 drop
    sub_done_ = create_subscription<std_msgs::msg::String>(
      "/manipulator_done", rclcpp::QoS(10),
      [this](const std_msgs::msg::String::SharedPtr msg)
      {
        if (msg->data.find("vacuum off") != std::string::npos) {
          send_udp_json("{\"cmd\":\"drop\"}");
        }
      });

    // 송신 스레드 시작
    run_.store(true);
    tx_thread_ = std::thread([this]{ tx_loop(); });

    RCLCPP_INFO(get_logger(),
      "opx_joint_tcp started (TCP %s:%d @%dHz, joints=%d | UDP_JSON=%s:%d)",
      host_.c_str(), port_, send_hz_, n_joints_,
      udp_json_enable_ ? "ON" : "OFF", udp_json_port_);
  }

  ~OpxJointTcp() override {
    run_.store(false);
    if (tx_thread_.joinable()) tx_thread_.join();
    if (sock_ >= 0) ::close(sock_);
  }

private:
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
        const size_t bytes = sizeof(float) * 4;
        ssize_t sent = ::send(sock_, reinterpret_cast<const char*>(out),
                              bytes, MSG_NOSIGNAL);
        if (sent != static_cast<ssize_t>(bytes)) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                               "send() failed (sent=%zd). Reconnecting...", sent);
          ::close(sock_); sock_ = -1;
        }
      }
      rate.sleep();
    }
  }

  void ensure_tcp_socket() {
    if (sock_ >= 0) return;
    sock_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sock_ < 0) return;

    int one = 1;
    ::setsockopt(sock_, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    ::setsockopt(sock_, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));

    sockaddr_in sa{};
    sa.sin_family = AF_INET;
    sa.sin_port   = htons(port_);
    if (::inet_pton(AF_INET, host_.c_str(), &sa.sin_addr) != 1) {
      ::close(sock_); sock_ = -1; return;
    }
    if (::connect(sock_, reinterpret_cast<sockaddr*>(&sa), sizeof(sa)) < 0) {
      ::close(sock_); sock_ = -1;
    }
  }

  std::string build_qr_json() const {
    if (qr_state_.empty()) {
      return std::string{"{\"cmd\":\""} + qr_cmd_ + "\"}";
    } else {
      return std::string{"{\"cmd\":\""} + qr_cmd_ + "\",\"state\":\"" + qr_state_ + "\"}";
    }
  }

  void send_udp_json(const std::string& json) {
    int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) return;
    sockaddr_in sa{};
    sa.sin_family = AF_INET;
    sa.sin_port   = htons(udp_json_port_);
    if (::inet_pton(AF_INET, host_.c_str(), &sa.sin_addr) != 1) {
      ::close(sock); return;
    }
    ::sendto(sock, json.data(), json.size(), 0,
             reinterpret_cast<sockaddr*>(&sa), sizeof(sa));
    ::close(sock);
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_qr_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_done_;

  std::string host_;
  int port_{9999};
  int send_hz_{50};
  int n_joints_{4};

  bool udp_json_enable_{true};
  int udp_json_port_{5010};
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
