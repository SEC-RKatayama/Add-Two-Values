#include <stdio.h>  // C言語なので、とりまインポート
#include <rclcpp/rclcpp.hpp>            // ROSノードを作成するので、とりまインポート
#include <std_msgs/msg/float64.hpp>     // float64のメッセージをトピックとして使うのでインポート

namespace add_two_values
{

class AddTwoValues : public rclcpp::Node {// rclcpp::Nodeを継承することでノードを作成
    public:
        // コンストラクタ
        AddTwoValues(const rclcpp::NodeOptions& options); // ここでコールバック関数を定義する

        // デストラクタ
        ~AddTwoValues();


        /**
         * @brief 足し算結果を表示する
        */
        void printAnswer();


        /**
         * @brief val1に値を設定する
         * @note トピック通信以外の経路でval1を設定することを可能にするために、
         * サブスクのコールバック関数内で直接設定するのではなく、publicの本関数によって設定する。
         * サブスクのコールバック関数をpublicにしない理由は、当該関数val1_subのdocsを参照。
         * 
         * @param val val1に設定される値
        */
       void set_val1(const double val);
       void set_val2(const double val);


    private:
        double val1_;
        double val2_;
        double answer_;

        void timerCallback();   // タイマーコールバック関数
        std::function<void()> timer_func_;      // create_wall_timer()にタイマーコールバック関数を渡すためのもの
        rclcpp::TimerBase::SharedPtr timer_;    // タイマーコールバック関数を時間周期で作動させるもの。SharedPtrはポインタのこと。つまりこれはポインタ変数。
        

        
        /// @brief val1のサブスクリプションのコールバック関数
        /// @attention 本関数はこのノードのこのトピックのサブスクのコールバック関数という意味を持つので、他のクラスからアクセスできる必要はないはずである。
        /// したがってprivateとする。（val1に値を設定するという意味を持つ関数は、他のクラスからアクセスされてもよいと考えたのでset_val1はpublicにしている。）
        void val1_sub(const std_msgs::msg::Float64::SharedPtr msg); // val1のサブスクリプションのコールバック関数
        void val2_sub(const std_msgs::msg::Float64::SharedPtr msg); // val2のサブスクリプションのコールバック関数

        /// @brief create_subscription()にコールバック関数を渡すためのもの
        std::function<void(std::shared_ptr<std_msgs::msg::Float64>)> val1_func_;
        std::function<void(std::shared_ptr<std_msgs::msg::Float64>)> val2_func_;

        /// @brief val1のサブスクライバ。SharedPtrはポインタのこと。つまりこれはポインタ変数。
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr val1_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr val2_sub_;
        
};

} // namespace add_two_values
