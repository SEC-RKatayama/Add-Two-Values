#include "node.hpp"

namespace add_two_values{

// コンストラクタ
AddTwoValues::AddTwoValues(const rclcpp::NodeOptions& options) : Node("add_two_values", options),
        val1_(0.0), val2_(0.0)  // 初期値を0.0に設定
{
    // サブスクライバの設定
    val1_func_ = std::bind(&AddTwoValues::val1_sub, this, std::placeholders::_1);   // 関数val1_sub()をstd::function型でラップして、create_subscriptionに入れられるようにする（thisはインスタンス自身を指す）
    val1_sub_ = this->create_subscription<std_msgs::msg::Float64>("/val1", rclcpp::QoS(1).reliable(), val1_func_);
    //          ^                         ^                       ^                                   ^
    //          サブスク作成メソッド        トピックタイプ           トピック名                           bindした関数をコールバック関数に設定

    // val2に対しても同様
    val2_func_ = std::bind(&AddTwoValues::val2_sub, this, std::placeholders::_1);
    val2_sub_ = this->create_subscription<std_msgs::msg::Float64>("/val2", rclcpp::QoS(1).reliable(), val2_func_);


    // タイマーコールバック関数の設定
    timer_func_ = std::bind(&AddTwoValues::timerCallback, this);    // 関数timerCallback()をstd::function型でラップして、create_wall_timerに入れられるようにする (thisはインスタンス自身を指す)
    timer_ = this->create_wall_timer(std::chrono::seconds(2), timer_func_);
    //                               ^                        ^
    //                               周期を指定             bindした関数をコールバック関数に設定
}


// デストラクタ
AddTwoValues::~AddTwoValues(){} // 何もしない


void AddTwoValues::val1_sub(std_msgs::msg::Float64::SharedPtr msg){
    set_val1(msg->data); // msg->dataでメッセージからデータ取り出し（https://docs.ros2.org/galactic/api/std_msgs/msg/Float64.html）
    return;
}

void AddTwoValues::val2_sub(std_msgs::msg::Float64::SharedPtr msg){
    set_val2(msg->data); // msg->dataでメッセージからデータ取り出し（https://docs.ros2.org/galactic/api/std_msgs/msg/Float64.html）
    return;
}


void AddTwoValues::timerCallback(){
    answer_ = val1_ + val2_;    // メンバ変数を足す
    printAnswer();
}


void AddTwoValues::set_val1(const double val){
    val1_ = val;    // メンバ変数を更新する
    return;
}


void AddTwoValues::set_val2(const double val){
    val2_ = val;    // メンバ変数を更新する
    return;
}


void AddTwoValues::printAnswer(){
    RCLCPP_INFO(rclcpp::get_logger("add_two_values"), "%lf + %lf = %lf", val1_, val2_, answer_);  // ROS2版printfみたいなもの。第一引数については深く考えないで。第二引数以降はprintfと同じ。rclcpp.hppに入ってる。
    return;
}

}