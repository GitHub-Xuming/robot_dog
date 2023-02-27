# 开源四足机器狗
## 开源资料说明
* 硬件资料

Hardware/Model目录包含所有零件模型文件
* 软件资料

Software/Linux目录为上位机控制软件
Software/STM32目录为单片机程序

## 机械结构设计

    1.整机由12自由度四足机器狗以及一台6轴机械臂组成，四足机器人腿部采用了五连杆机构，之所以采用这样的结构，源于五连杆机构具有更大的承载能力以及更好的刚性，但是也有它的缺点，运动范围相对串联腿小，不能做太大幅度动作，这个版本先使用并联腿，后面可以优化为串联结构！        
    2.五连杆两摆动臂采用浮动设计，中间加入减震弹簧，能减少对电机的冲击  
    3.腿部外伸自由度使用同步轮减速，减速比2.4，可以加大转矩以及减小电机间隙对输出的影响
    4.机身主要由两条碳纤维方管作为骨架支撑整个身体，其他运动部件以它为基准
    5.机械臂连杆使用碳纤维圆管连接
    6.机械臂末端安装机械爪，夹爪有弹簧缓冲结构
