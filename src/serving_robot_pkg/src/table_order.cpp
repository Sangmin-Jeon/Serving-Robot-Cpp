#include <thread>
#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <QApplication>
#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTableWidget>
#include <QLabel>
#include <QPushButton>
#include <QGroupBox>
#include <QTimer>
#include <QMessageBox>
#include <QGridLayout>
#include <QPixmap>
#include <QFrame>

class TableOrderNode : public rclcpp::Node {
public:
    TableOrderNode() : Node("table_order_node") {
        RCLCPP_INFO(this->get_logger(), "OrderNode has started");

        // 서비스 클라이언트 및 서버 생성
        order_client_ = this->create_client<std_srvs::srv::SetBool>("order_service");
        cancel_service_ = this->create_service<std_srvs::srv::SetBool>(
            "order_cancel_service",
            std::bind(&TableOrderNode::cancel_order_callback, this, std::placeholders::_1, std::placeholders::_2));
    }

    void place_order() {
        if (!order_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Service not available, try again later.");
            return;
        }

        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;

        auto future = order_client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Order placed");
    }

private:
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr order_client_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr cancel_service_;

    void cancel_order_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        response->success = true;
        RCLCPP_INFO(this->get_logger(), "Order cancelled");
    }
};

class MainWindow : public QWidget {
public:
    explicit MainWindow(std::shared_ptr<TableOrderNode> node, QWidget *parent = nullptr)
        : QWidget(parent), node_(node) {
        setup_ui();
    }

private:
    std::shared_ptr<TableOrderNode> node_;
    QLabel *table_label_;
    QTableWidget *order_table_;
    QLabel *total_label_;
    std::map<std::string, int> order_data_;

    void setup_ui() {
        QVBoxLayout *main_layout = new QVBoxLayout(this);

        // 상단 레이아웃 설정
        QHBoxLayout *top_layout = new QHBoxLayout();
        table_label_ = new QLabel("B4", this);
        table_label_->setAlignment(Qt::AlignCenter);
        top_layout->addWidget(table_label_);

        QLabel *shop_label = new QLabel("🐟날로먹는집🐟", this);
        shop_label->setAlignment(Qt::AlignCenter);
        top_layout->addWidget(shop_label);
        main_layout->addLayout(top_layout);

        // 메뉴 및 주문 레이아웃 추가
        QHBoxLayout *content_layout = new QHBoxLayout();
        content_layout->addLayout(create_menu_layout(), 3);
        content_layout->addWidget(create_separator());
        content_layout->addLayout(create_order_layout(), 1);
        main_layout->addLayout(content_layout);

        setLayout(main_layout);
    }

    QVBoxLayout* create_menu_layout() {
        QVBoxLayout *menu_layout = new QVBoxLayout();
        QGridLayout *grid_layout = new QGridLayout();

        std::vector<std::pair<std::string, int>> menu_items = {
            {"방어회", 50000}, {"향어회", 35000}, {"광어+우럭 세트", 38000}, {"매운탕", 10000}, {"소주", 5000}
        };

        int row = 0;
        for (const auto &item : menu_items) {
            grid_layout->addLayout(create_menu_item_layout(item.first, item.second), row / 3, row % 3);
            ++row;
        }

        menu_layout->addLayout(grid_layout);
        return menu_layout;
    }

    QVBoxLayout* create_menu_item_layout(const std::string &name, int price) {
        QVBoxLayout *item_layout = new QVBoxLayout();

        QLabel *item_label = new QLabel(QString::fromStdString(name + "\n" + std::to_string(price) + "원"), this);
        item_label->setAlignment(Qt::AlignCenter);
        item_label->setStyleSheet("font-size: 18px;");

        QPushButton *add_button = new QPushButton("+", this);
        connect(add_button, &QPushButton::clicked, this, [this, name]() { add_item(name); });

        QPushButton *remove_button = new QPushButton("-", this);
        connect(remove_button, &QPushButton::clicked, this, [this, name]() { remove_item(name); });

        QHBoxLayout *button_layout = new QHBoxLayout();
        button_layout->addWidget(add_button);
        button_layout->addWidget(remove_button);

        item_layout->addWidget(item_label);
        item_layout->addLayout(button_layout);

        return item_layout;
    }

    QFrame* create_separator() {
        QFrame *separator = new QFrame(this);
        separator->setFrameShape(QFrame::VLine);
        separator->setFrameShadow(QFrame::Sunken);
        return separator;
    }

    QVBoxLayout* create_order_layout() {
        QVBoxLayout *order_layout = new QVBoxLayout();

        QLabel *order_label = new QLabel("주문 내역", this);
        order_label->setStyleSheet("font-size: 18px; font-weight: bold;");
        order_layout->addWidget(order_label);

        order_table_ = new QTableWidget(this);
        order_table_->setColumnCount(3);
        order_table_->setHorizontalHeaderLabels({"메뉴명", "수량", "금액"});
        order_layout->addWidget(order_table_);

        total_label_ = new QLabel("총 금액: 0원", this);
        order_layout->addWidget(total_label_);

        QPushButton *order_button = new QPushButton("주문", this);
        connect(order_button, &QPushButton::clicked, this, &MainWindow::place_order);

        QPushButton *cancel_button = new QPushButton("취소", this);
        connect(cancel_button, &QPushButton::clicked, this, &MainWindow::cancel_order);

        QHBoxLayout *button_layout = new QHBoxLayout();
        button_layout->addWidget(order_button);
        button_layout->addWidget(cancel_button);
        order_layout->addLayout(button_layout);

        return order_layout;
    }

    void add_item(const std::string &name) {
        ++order_data_[name];
        update_order_list();
    }

    void remove_item(const std::string &name) {
        if (order_data_.count(name) && order_data_[name] > 0) {
            --order_data_[name];
            if (order_data_[name] == 0) {
                order_data_.erase(name);
            }
            update_order_list();
        }
    }

    void update_order_list() {
        order_table_->setRowCount(0);
        int total_price = 0;

        for (const auto &item : order_data_) {
            int row = order_table_->rowCount();
            order_table_->insertRow(row);
            order_table_->setItem(row, 0, new QTableWidgetItem(QString::fromStdString(item.first)));
            order_table_->setItem(row, 1, new QTableWidgetItem(QString::number(item.second)));
            int price = item.second * 5000;  // 예시 가격
            total_price += price;
            order_table_->setItem(row, 2, new QTableWidgetItem(QString::number(price) + "원"));
        }

        total_label_->setText("총 금액: " + QString::number(total_price) + "원");
    }

    void place_order() {
        node_->place_order();
        QMessageBox::information(this, "주문 성공", "주문이 성공적으로 완료되었습니다!");
    }

    void cancel_order() {
        order_data_.clear();
        update_order_list();
        QMessageBox::warning(this, "주문 취소", "주문이 취소되었습니다.");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    QApplication app(argc, argv);
    auto node = std::make_shared<TableOrderNode>();

    MainWindow window(node);
    window.show();

    std::thread ros_thread([&]() {
        rclcpp::spin(node);
    });

    int result = app.exec();

    rclcpp::shutdown();
    ros_thread.join();

    return result;
    
}
