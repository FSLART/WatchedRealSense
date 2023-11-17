#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include <unistd.h>

class MyNode : public rclcpp::Node {
public:
    MyNode() : Node("my_node") {
        // Initialize ROS2 publisher
        heartbeat_publisher_ = create_publisher<std_msgs::msg::String>("carnary_heartbeat", 10);

        // Create a timer to send heartbeats periodically
        timer_ = create_wall_timer(std::chrono::seconds(5), [this]() {
            send_heartbeat();
        });

        // Perform CARnary-specific initialization here.

        if (establishCommunicationWithDaemon() && performHandshake()) {
            RCLCPP_INFO(get_logger(), "Connected to CARnary daemon and performed handshake.");
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to connect to CARnary daemon or perform handshake.");
        }
    }

    bool establishCommunicationWithDaemon() {
        // Create a Unix socket to communicate with the CARnary daemon.
        sockfd_ = socket(AF_UNIX, SOCK_STREAM, 0);
        if (sockfd_ == -1) {
            RCLCPP_ERROR(get_logger(), "Error creating socket: %s", strerror(errno));
            return false;
        }

        sockaddr_un server_addr{};
        server_addr.sun_family = AF_UNIX;
        strncpy(server_addr.sun_path, "/tmp/carnary_daemon_socket", sizeof(server_addr.sun_path) - 1);

        // Connect to the CARnary daemon.
        if (connect(sockfd_, reinterpret_cast<struct sockaddr*>(&server_addr), sizeof(server_addr)) == -1) {
            RCLCPP_ERROR(get_logger(), "Error connecting to CARnary daemon: %s", strerror(errno));
            close(sockfd_);
            return false;
        }

           return true;
    }


     bool performHandshake() {
        // Implement the handshake protocol with the CARnary daemon.


        // Example: Send a handshake message.
        std::string handshake_message = "Hello CARnary!";
        ssize_t bytes_sent = send(sockfd_, handshake_message.c_str(), handshake_message.size(), 0);
        if (bytes_sent == -1) {
            RCLCPP_ERROR(get_logger(), "Error sending handshake message: %s", strerror(errno));
            return false;
        }
        return true;
    }

    void send_heartbeat() {
        auto heartbeat_msg = std_msgs::msg::String();
        heartbeat_msg.data = "Node is alive"; // Modify as needed
        heartbeat_publisher_->publish(heartbeat_msg);
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyNode>());
    rclcpp::shutdown();
    return 0;
}


