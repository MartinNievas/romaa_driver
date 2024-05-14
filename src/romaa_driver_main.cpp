#include "rclcpp/rclcpp.hpp"
#include "romaa_driver/RomaaDriverNode.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto romaa_drv = std::make_shared<romaa_driver::RomaaDriverNode>();

    if( romaa_drv->is_connected() )
        rclcpp::spin(romaa_drv);

    rclcpp::shutdown();
    return 0;
}
