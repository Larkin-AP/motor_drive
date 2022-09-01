#ifndef ROBOT_H_
#define ROBOT_H_

#include <memory>
#include <aris.hpp>

namespace robot
{

class MoveJoint : public aris::core::CloneObject<MoveJoint, aris::plan::Plan>
{
public:
    auto virtual prepareNrt()->void;
    auto virtual executeRT()->int;
    auto virtual collectNrt()->void;

    explicit MoveJoint(const std::string &name = "MoveJiont");
private:
    double dir_;
};



    class VelDrive : public aris::core::CloneObject<VelDrive,aris::plan::Plan>
    {
     public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~VelDrive();
        explicit VelDrive(const std::string &name = "vel_drive");

     private:
        double cef_;
    };


    class TcurveDrive :public aris::core::CloneObject<TcurveDrive,aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~TcurveDrive();
        explicit TcurveDrive(const std::string &name = "motor_drive");

    private:
        double cef_;
    };


    auto createMasterROSMotorTest()->std::unique_ptr<aris::control::Master>;
    auto createControllerROSMotorTest()->std::unique_ptr<aris::control::Controller>;
    auto createPlanROSMotorTest()->std::unique_ptr<aris::plan::PlanRoot>;
}

#endif
