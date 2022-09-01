#include <aris.hpp>
#include "robot.h"


int main(int argc, char *argv[])
{
    auto&cs = aris::server::ControlServer::instance();

    auto master = new aris::control::EthercatMaster;

    master->scan();
    //std::cout << aris::core::toXmlString(*master) << std::endl;


    cs.resetMaster(robot::createMasterROSMotorTest().release());


    std::cout << aris::core::toXmlString(cs.master()) << std::endl;

    //cs.resetMaster(master);

    cs.resetController(robot::createControllerROSMotorTest().release());

//    std::cout << aris::core::toXmlString(cs) << std::endl;
    cs.resetPlanRoot(robot::createPlanROSMotorTest().release());



    cs.init();
    //开启WebSocket/socket服务器//
    cs.open();
    cs.start();
    //等待终端输入函数，本函数不能去掉，否则实时线程和主线程都会结束//



    cs.runCmdLine();

    return 0;
}
