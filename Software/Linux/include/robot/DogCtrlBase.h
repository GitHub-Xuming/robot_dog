#pragma once
#include "struct.h"
#include <vector>

class DogCtrlBase
{
public:
    virtual void EN_Ctrl(bool en) = 0;
    virtual void RobotDogCtrl(Pose posture, FootCoord foot_pos, std::vector<uint8_t> &out) = 0;
 
private:

};