#include <thread>
#include <queue>
#include <regex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <QApplication>
#include <QMainWindow>
#include <QPushButton>
#include <QtWidgets>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QLabel>
#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTimer>




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

class MainDashboard : public QWidget {

int is_order_cell_;
int cell_count_; 

public:
    explicit MainDashboard(std::shared_ptr<rclcpp::Node> node, QWidget *parent = nullptr)
        : QWidget(parent), is_order_cell_(-1), cell_count_(0), node_(node) {
		RCLCPP_INFO(node_->get_logger(), "Order cell index: %d", is_order_cell_);
		RCLCPP_INFO(node_->get_logger(), "Total cell count: %d", cell_count_);
        setup_layout();
    }

    void come_back_btn() {
        RCLCPP_INFO(node_->get_logger(), "Robot is being called back.");
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    QGridLayout *grid_layout_;
    QWidget *grid_widget_;
    QScrollArea *scroll_area_;
    QVBoxLayout *extra_scroll_layout_;
    QWidget *extra_scroll_widget_;
    std::vector<QWidget*> cells_;

    void setup_layout() {
        QVBoxLayout *main_layout = new QVBoxLayout(this);

        // Title Layout
        QVBoxLayout *title_layout = new QVBoxLayout();
        QLabel *title_label = new QLabel("🐟 날로먹는집 주방 모니터 🐟");
        title_label->setAlignment(Qt::AlignCenter);
        title_label->setStyleSheet("font-size: 24px; font-weight: bold; margin: 5px;");
        title_label->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        title_layout->addWidget(title_label);

        // Button Layout
        QHBoxLayout *button_layout = new QHBoxLayout();
        button_layout->addStretch(1);
        QPushButton *button = new QPushButton("로봇 호출");
        button->setStyleSheet("font-size: 24px; font-weight: bold; margin: 5px;");
        button_layout->addWidget(button);
        title_layout->addLayout(button_layout);

        // 버튼 클릭 시 직접 멤버 함수 호출 (시그널/슬롯 사용 안 함)
        connect(button, &QPushButton::clicked, this, [this]() { come_back_btn(); });

        main_layout->addLayout(title_layout);

        // Order list scroll area
        grid_layout_ = new QGridLayout();
        grid_layout_->setAlignment(Qt::AlignTop | Qt::AlignLeft);

        grid_widget_ = new QWidget();
        grid_widget_->setLayout(grid_layout_);

        scroll_area_ = new QScrollArea(this);
        scroll_area_->setWidgetResizable(true);
        scroll_area_->setWidget(grid_widget_);
        scroll_area_->setFixedHeight(700);
        scroll_area_->setFixedWidth(1200);

        QHBoxLayout *horizontal_layout = new QHBoxLayout();
        horizontal_layout->addWidget(scroll_area_);

        extra_scroll_layout_ = new QVBoxLayout();
        extra_scroll_layout_->setSpacing(0);
        extra_scroll_layout_->setAlignment(Qt::AlignTop);

        extra_scroll_widget_ = new QWidget();
        extra_scroll_widget_->setLayout(extra_scroll_layout_);

        QScrollArea *extra_scroll_area = new QScrollArea();
        extra_scroll_area->setWidgetResizable(true);
        extra_scroll_area->setWidget(extra_scroll_widget_);
        extra_scroll_area->setFixedHeight(700);

        QVBoxLayout *extra_layout = new QVBoxLayout();
        extra_layout->addWidget(extra_scroll_area);

        QWidget *extra_layout_widget = new QWidget();
        extra_layout_widget->setLayout(extra_layout);

        horizontal_layout->addWidget(extra_layout_widget);
        main_layout->addLayout(horizontal_layout);

        setLayout(main_layout);
    }
};

class Cell : public QWidget {

int cell_index_;
int table_number_;
 
public:
    explicit Cell(int table_number, 
                  std::vector<std::string> order_details, 
                  const std::string& order_time, 
                  std::shared_ptr<rclcpp::Node> node, 
                  QWidget* parent = nullptr, 
                  int index = 0)
        : QWidget(parent), node_(node), cell_index_(index), 
          table_number_(table_number), order_details_(order_details), 
          order_time_(order_time) {
        set_cell_layout();
		
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::vector<std::string> order_details_;
    std::string order_time_;

    QLabel* table_number_label_;
    QLabel* move_robot_label_;
    QLabel* goal_robot_label_;
    QLabel* come_back_robot_label_;
    QLabel* timer_label_;
    QPushButton* confirm_button_;
    QPushButton* cancel_button_;
    QTimer* timer_;

    void set_cell_layout() {
        int screen_width = 1300;
        int available_width = screen_width - 40;
        int cell_width = available_width / 3;
        int cell_height = 550;

        setFixedSize(cell_width, cell_height);

        QWidget* wrapper = new QWidget(this);
        wrapper->setGeometry(0, 0, cell_width, cell_height);
        wrapper->setStyleSheet("background-color: white; border: 2px solid #4CAF50; border-radius: 5px; margin: 5px;");

        QVBoxLayout* layout = new QVBoxLayout(wrapper);
        layout->setAlignment(Qt::AlignTop);
        layout->setSpacing(0);
        layout->setContentsMargins(10, 10, 10, 10);

        QHBoxLayout* top_layout = new QHBoxLayout();
        top_layout->setSpacing(10);

        // 테이블 번호 라벨
        table_number_label_ = new QLabel(QString("테이블 %1").arg(table_number_), wrapper);
        table_number_label_->setStyleSheet("background-color: blue; padding: 5px; font-size: 18px; font-weight: bold; color: white;");
        table_number_label_->setFixedHeight(50);
        table_number_label_->setAlignment(Qt::AlignCenter);

        top_layout->addWidget(table_number_label_);

        // 버튼 레이아웃
        QHBoxLayout* button_layout = new QHBoxLayout();
        button_layout->setSpacing(5);

        confirm_button_ = new QPushButton("확인", wrapper);
        confirm_button_->setStyleSheet("background-color: #0d3383; color: white; font-size: 18px; font-weight: bold;");
        confirm_button_->setFixedSize(100, 50);

        move_robot_label_ = new QLabel("로봇 이동중", wrapper);
        move_robot_label_->setStyleSheet("background-color: green; color: white; font-size: 18px; font-weight: bold;");
        move_robot_label_->setFixedSize(110, 50);
        move_robot_label_->setAlignment(Qt::AlignCenter);
        move_robot_label_->setVisible(false);

        goal_robot_label_ = new QLabel("테이블 도착", wrapper);
        goal_robot_label_->setStyleSheet("background-color: green; color: white; font-size: 18px; font-weight: bold;");
        goal_robot_label_->setFixedSize(110, 50);
        goal_robot_label_->setAlignment(Qt::AlignCenter);
        goal_robot_label_->setVisible(false);

        come_back_robot_label_ = new QLabel("주방 복귀중", wrapper);
        come_back_robot_label_->setStyleSheet("background-color: green; color: white; font-size: 18px; font-weight: bold;");
        come_back_robot_label_->setFixedSize(110, 50);
        come_back_robot_label_->setAlignment(Qt::AlignCenter);
        come_back_robot_label_->setVisible(false);

        cancel_button_ = new QPushButton("취소", wrapper);
        cancel_button_->setStyleSheet("background-color: #cc0033; color: white; font-size: 18px; font-weight: bold;");
        cancel_button_->setFixedSize(100, 50);

        button_layout->addWidget(confirm_button_);
        button_layout->addWidget(move_robot_label_);
        button_layout->addWidget(goal_robot_label_);
        button_layout->addWidget(come_back_robot_label_);
        button_layout->addWidget(cancel_button_);

        top_layout->addLayout(button_layout);
        layout->addLayout(top_layout);

        timer_label_ = new QLabel("경과시간: 00:00", wrapper);
        timer_label_->setStyleSheet("background-color: lightgreen; padding: 5px; font-size: 16px; color: black;");
        timer_label_->setAlignment(Qt::AlignCenter);
        layout->addWidget(timer_label_);

        setLayout(layout);

        // 버튼 클릭 시그널 연결
        // connect(confirm_button_, &QPushButton::clicked, this, &Cell::confirm_order);
        // connect(cancel_button_, &QPushButton::clicked, this, &Cell::cancel_order);
    }

};

class Tab1Content : public QWidget {
public:
    explicit Tab1Content(std::shared_ptr<rclcpp::Node> node, QWidget *parent = nullptr)
        : QWidget(parent), node_(node) {
        QVBoxLayout *layout = new QVBoxLayout(this);
        main_dashboard_ = new MainDashboard(node, this);
        layout->addWidget(main_dashboard_);
        setLayout(layout);
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    MainDashboard *main_dashboard_;
};


class MainWindow : public QMainWindow {
public:
    MainWindow() {
        setWindowTitle("Kitchen Monitor");
		resize(1920, 1000);

		setup_layout();

    }


private:
	void setup_layout() {
        // 중앙 위젯 설정
        QWidget *centralWidget = new QWidget(this);
        centralWidget->setObjectName("centralwidget");

        // 탭 위젯 생성 및 설정
        QTabWidget *tabWidget = new QTabWidget(centralWidget);
        tabWidget->setObjectName("tabWidget");

        // Tab 1 생성 및 추가
		std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("kitchen_monitor_node");
        Tab1Content *tab1 = new Tab1Content(node, this);
        tabWidget->addTab(tab1, tr("주방 모니터"));

        // 중앙 레이아웃 설정 및 위젯 추가
        QVBoxLayout *centralLayout = new QVBoxLayout(centralWidget);
        centralLayout->addWidget(tabWidget);
        centralWidget->setLayout(centralLayout);

        // QMainWindow의 중앙 위젯 설정
        setCentralWidget(centralWidget);
    };
};


int main(int argc, char **argv) {
    // ROS 2 초기화
    rclcpp::init(argc, argv);

    QApplication app(argc, argv);
    MainWindow window;
    window.show();

    // ROS 2 노드 및 실행기 설정
    auto node = std::make_shared<KitchenMonitorNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    // ROS 2 실행을 백그라운드 스레드에서 실행
    std::thread ros_thread([&executor]() {
        executor.spin();
    });

    // Qt 이벤트 루프 실행
    int result = app.exec();

    // 프로그램 종료 시 정리
    rclcpp::shutdown();
    ros_thread.join();

    return result;
}





