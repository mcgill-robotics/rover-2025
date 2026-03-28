#include "rover_nav2/bt_ros_node.hpp"

namespace polymath
{
    namespace bt_ros_example
    {

        BtRosNode::BtRosNode(const rclcpp::NodeOptions &options)
            : rclcpp_lifecycle::LifecycleNode("bt_ros_node", options){}

        BtRosNode::~BtRosNode()
        {
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        BtRosNode::on_configure(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(get_logger(), "Configuring Behavior Tree...");

            blackboard_ = BT::Blackboard::create();

            try
            {
                tree_ = factory_.createTreeFromFile("BT.xml", blackboard_);
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(get_logger(), "Tree creation failed: %s", e.what());
                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
            }

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        BtRosNode::on_activate(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(get_logger(), "Activating BT Node...");

            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&BtRosNode::timer_callback, this));

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        BtRosNode::on_deactivate(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(get_logger(), "Deactivating BT Node...");
            timer_.reset();
            tree_.haltTree();
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        BtRosNode::on_cleanup(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(get_logger(), "Cleaning up BT Node...");
            blackboard_.reset();
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        BtRosNode::on_shutdown(const rclcpp_lifecycle::State &)
        {
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        void BtRosNode::timer_callback()
        {
            BT::NodeStatus status = tree_.tickOnce();

            if (status == BT::NodeStatus::SUCCESS)
            {
                RCLCPP_INFO(get_logger(), "BT Finished with SUCCESS");
                timer_.reset();
            }
            else if (status == BT::NodeStatus::FAILURE)
            {
                RCLCPP_INFO(get_logger(), "BT Finished with FAILURE");
                timer_.reset();
            }
        }

    } 
} 

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<polymath::bt_ros_example::BtRosNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
