#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

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
    host_     = declare_parameter<std::string>("host", "10.49.212.68");  // Unity IP
    port_     = declare_parameter<int>("port", 9999);                 // Unity 포트
    send_hz_  = declare_parameter<int>("send_hz", 50);                // 전송 주기(Hz)
    n_joints_ = declare_parameter<int>("n_joints", 4);                // 보낼 관절 개수(최대 4)

    n_joints_ = std::clamp(n_joints_, 1, 4);

    // JointState 구독: 최신 값만 캐시에 반영
    sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::QoS(30),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lk(mtx_);
        const size_t n = std::min(static_cast<size_t>(n_joints_), msg->position.size());
        for (size_t i = 0; i < n; ++i) joints_[i] = static_cast<float>(msg->position[i]); // 라디안
        for (size_t i = n; i < static_cast<size_t>(n_joints_); ++i) joints_[i] = 0.f;
      });

    // 송신 스레드 시작
    run_.store(true);
    tx_thread_ = std::thread([this]{ tx_loop(); });

    RCLCPP_INFO(get_logger(), "opx_joint_tcp started (dst=%s:%d, %dHz, joints=%d)",
                host_.c_str(), port_, send_hz_, n_joints_);
  }

  ~OpxJointTcp() override {
    run_.store(false);
    if (tx_thread_.joinable()) tx_thread_.join();
    if (sock_ >= 0) ::close(sock_);
  }

private:
  // 주기 루프: 소켓 보장 → 최신 관절 16바이트(또는 4*N바이트) 전송
  void tx_loop() {
    if (send_hz_ < 1) send_hz_ = 1;
    rclcpp::WallRate rate(send_hz_);

    while (rclcpp::ok() && run_.load()) {
      ensure_socket();

      float out[4] = {0,0,0,0};
      {
        std::lock_guard<std::mutex> lk(mtx_);
        for (int i = 0; i < n_joints_; ++i) out[i] = joints_[i];
      }

      if (sock_ >= 0) {
        const size_t bytes = sizeof(float) * 4; // 항상 16바이트 고정(수신 코드 간단)
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

  // 소켓 연결 보장/재접속
  void ensure_socket() {
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
      RCLCPP_INFO(get_logger(), "Connected to Unity %s:%d", host_.c_str(), port_);
    }
  }

  // members
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;

  std::string host_;
  int port_{9999};
  int send_hz_{50};
  int n_joints_{4};

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
