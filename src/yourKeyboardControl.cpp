#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <termios.h>
#include <unistd.h>

using namespace std;

char getch() {
        char buf = 0;
        struct termios old = {0};
        if (tcgetattr(0, &old) < 0)
                perror("tcsetattr()");
        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        old.c_cc[VMIN] = 1;
        old.c_cc[VTIME] = 0;
        if (tcsetattr(0, TCSANOW, &old) < 0)
                perror("tcsetattr ICANON");
        if (read(0, &buf, 1) < 0)
                perror ("read()");
        old.c_lflag |= ICANON;
        old.c_lflag |= ECHO;
        if (tcsetattr(0, TCSADRAIN, &old) < 0)
                perror ("tcsetattr ~ICANON");
        return (buf);
}

ros::Publisher command_pub;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh;

    

    command_pub = nh.advertise<std_msgs::Float64MultiArray>("/real_robot_arm_joint",10);
    
    std_msgs::Float64MultiArray joint_angle;

    joint_angle.data = {0, 0, 0, 0, 0};

    // processing command
    
    while(ros::ok())
    {
        char c = getch();
        if(c=='a') joint_angle.data[0] += 0.01;
        else if (c == 's') joint_angle.data[1] += 0.01;
        else if (c == 'd') joint_angle.data[2] += 0.01;
        else if (c == 'f') joint_angle.data[3] += 0.01;
        else if (c == 'z') joint_angle.data[0] -= 0.01;
        else if (c == 'x') joint_angle.data[1] -= 0.01;
        else if (c == 'c') joint_angle.data[2] -= 0.01;
        else if (c == 'v') joint_angle.data[3] -= 0.01;
        else if (c == '0') joint_angle.data[4] = 0;
        else if (c == '1') joint_angle.data[4] = 1;
        cout << joint_angle << endl;
        command_pub.publish(joint_angle);
    }

    

    ros::spin(); 

    return 0;
}