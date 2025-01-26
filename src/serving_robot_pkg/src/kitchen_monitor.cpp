#include <rclcpp/rclcpp.hpp>
#include <string>
#include "std_msgs/msg/string.hpp"

#include <QApplication>
#include <QMainWindow>
#include <QtWidgets>

class KitchenMonitorNode : public rclcpp::Node {

rclcpp::Publisher<std_msgs::msg::String>::SharedPtr test_pub;
rclcpp::TimerBase::SharedPtr timer_;

public:
    KitchenMonitorNode() : Node("kitchen_monitor_node") {
        RCLCPP_INFO(this->get_logger(), "Kitchen Monitor Node has started!");

        test_pub = this->create_publisher<std_msgs::msg::String>("test_msg", 10);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&KitchenMonitorNode::publish_msg, this));
    }

private:
    void publish_msg() {
        // 메시지 객체 생성 및 데이터 설정
        auto message = std_msgs::msg::String();
        message.data = "test";

        // 퍼블리시
        test_pub->publish(message);

        // 로그 출력
        RCLCPP_INFO(this->get_logger(), "Published: '%s'", message.data.c_str());
    }

};


class MainWindow : public QMainWindow {
public:
    MainWindow() {
        setWindowTitle("Kitchen Monitor");
        setGeometry(100, 100, 400, 300);

		set_layout();

    }

	void set_layout() {
        // 기본 레이아웃 설정
        QWidget *centralWidget = new QWidget(this);
        QVBoxLayout *layout = new QVBoxLayout(centralWidget);

		QLabel *label = new QLabel("Welcome to the Kitchen Monitor!", centralWidget);
        QPushButton *button = new QPushButton("Start Monitoring", centralWidget);

        layout->addWidget(label);
        layout->addWidget(button);

		setCentralWidget(centralWidget);

		return;

	};
};

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);

    QApplication app(argc, argv);
    MainWindow window;
    window.show();

    // ROS2 이벤트 루프는 별도로 spin_some()을 사용하여 Qt와 병렬 실행
    rclcpp::executors::SingleThreadedExecutor exec;
    auto node = std::make_shared<KitchenMonitorNode>();
    exec.add_node(node);

    // Qt 이벤트 루프와 ROS2 이벤트 루프 병행 실행
    while (rclcpp::ok()) {
        app.processEvents();  // Qt 이벤트 처리
        exec.spin_some();     // ROS2 이벤트 처리
    }

    rclcpp::shutdown();

    return app.exec();
}
