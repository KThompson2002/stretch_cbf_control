// code written by chatgpt

// ROS headers
#include <rclcpp/rclcpp.hpp>
#include <stretch_ee_vel_ctrl_interfaces/msg/relative_move.hpp>
// read keyboard input without waiting for Enter
#include <termios.h> // switch terminal to raw ASCII
#include <unistd.h>
#include <fcntl.h>
#include <iostream>

using RelativeMove = stretch_ee_vel_ctrl_interfaces::msg::RelativeMove;

class KeyboardTeleop : public rclcpp::Node
{
public:
    KeyboardTeleop() :
    Node("keyboard_relmove"),
    scale_by(0.05)
    {
        declare_parameter("scale_by", rclcpp::ParameterValue(scale_by));
        get_parameter("scale_by", scale_by);
        pub_ = create_publisher<RelativeMove>(
            "/velocity_pub/vel_command", 100);

        setupTerminal();

        timer_ = create_wall_timer(
            std::chrono::milliseconds(5), // 5 ms = 200 Hz
            std::bind(&KeyboardTeleop::loop, this));

        RCLCPP_INFO(get_logger(), "Keyboard teleop started");
        RCLCPP_INFO(get_logger(), "w:+z s:-z a:+x d:-x z:-y x:+y c:+dtheta v:-dtheta e:zero o:open gripper p:close gripper q:quit");
    }

    // restore the terminal when done so shell works again
    ~KeyboardTeleop()
    {
        restoreTerminal();
    }

private:
    rclcpp::Publisher<RelativeMove>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double scale_by; // how much to move per key press, in meters

    struct termios orig_termios; // save so we can restore later

    //// \brief disables terminal buffering, echo, and enables raw keyboard input
    void setupTerminal()
    {
        tcgetattr(STDIN_FILENO, &orig_termios);

        struct termios raw = orig_termios;
        raw.c_lflag &= ~(ICANON | ECHO); // disables line buffering, i.e., everything in a new line

        tcsetattr(STDIN_FILENO, TCSANOW, &raw);

        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK); // set to nonblocking, e.g., immediately read from terminal, don't wait for key press. keeps ros2 timer running
    }

    void restoreTerminal()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios);
    }

    void publishMove(double dx, double dy, double dz, double dtheta,
                     int8_t gripper = RelativeMove::GRIPPER_HOLD)
    {
        auto msg = RelativeMove();
        msg.dx = dx;
        msg.dy = dy;
        msg.dz = dz;
        msg.dtheta = dtheta;
        msg.gripper = gripper;

        pub_->publish(msg);

        // RCLCPP_INFO(get_logger(), "Published dx=%.2f dy=%.2f dz=%.2f dtheta=%.2f gripper=%d", dx, dy, dz, dtheta, gripper);
    }

    void loop()
    {
        char c;

        if (read(STDIN_FILENO, &c, 1) < 0) // read 1 byte from keyboard input
            // publishMove(0.0, 0.0, 0.0, 0.0); // if nothing read, publish zero command to stop the robot
            return; // if nothing read, loop() does nothing

        if (c == 'q') // equivalent to ctrl C
        {
            rclcpp::shutdown();
            return;
        }

        switch(c)
        {
            case 'w':
                publishMove(0.0, 0.0, scale_by, 0.0);
                break;
            case 's':
                publishMove(0.0, 0.0, -scale_by, 0.0);
                break;
            case 'a':
                publishMove(scale_by, 0.0, 0.0, 0.0);
                break;
            case 'd':
                publishMove(-scale_by, 0.0, 0.0, 0.0);
                break;
            case 'z':
                publishMove(0.0, -scale_by, 0.0, 0.0);
                break;
            case 'x':
                publishMove(0.0, scale_by, 0.0, 0.0);
                break;
            case 'c':
                publishMove(0.0, 0.0, 0.0, scale_by*5); // scale up rotation by 5x so it's easier to test
                break;
            case 'v':
                publishMove(0.0, 0.0, 0.0, -scale_by*5);
                break;
            case 'e':
                publishMove(0.0, 0.0, 0.0, 0.0);
                break;
            case 'o':
                publishMove(0.0, 0.0, 0.0, 0.0, RelativeMove::GRIPPER_OPEN);
                break;
            case 'p':
                publishMove(0.0, 0.0, 0.0, 0.0, RelativeMove::GRIPPER_CLOSED);
                break;
            default:
                break;
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardTeleop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}
