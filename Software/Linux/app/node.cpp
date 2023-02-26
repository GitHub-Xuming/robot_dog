#include "Robot.h"
#include "RobotArm.h"
#include "RobotDog.h"

std::shared_ptr<Robot> g_robot;
std::shared_ptr<arm::RobotArm> g_robot_arm;
std::shared_ptr<RobotDog> g_robot_dog;

int main(int argc, char * argv[])
{
    g_robot_dog = std::make_shared<RobotDog>();
    g_robot_arm = std::make_shared<arm::RobotArm>();
    g_robot = std::make_shared<Robot>();

    g_robot_arm->RegistPoseFeedbackImpl(std::bind(&RobotDog::RobotDogPoseFeedback, g_robot_dog, std::placeholders::_1));
    g_robot->RegistRobotDogImpl(g_robot_dog);
    g_robot->RegistRobotArmImpl(g_robot_arm);
    
    getchar();
    return 0;
}
