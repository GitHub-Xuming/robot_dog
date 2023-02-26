#pragma once
#include "struct.h"

class ArmCtrlBase
{
public:
    virtual void ArmCoordinateMode(Pose point, AxisAngle &result) = 0;
    virtual void ArmAxisMode(AxisAngle angle) = 0;
    virtual void ClawCtrl(float angle) = 0;

    
private:

};