#ifndef SONAR_HPP
#define SONAR_HPP
#include "hwlib.hpp"
namespace target = hwlib::target;

class sonar{
private:
    target::pin_in & echo;
    target::pin_out & trigger;

public:
    sonar(target::pin_in & echo, target::pin_out & trigger):
        echo(echo),
        trigger(trigger)
    {};

    int measure(){
        int start = 0;
        int stop = 0;
        trigger.write(1);
        trigger.flush();
        hwlib::wait_us(10);
        trigger.write(0);
        trigger.flush();

        start = hwlib::now_us();
        
        while(echo.read() == 0){}
        while(echo.read() == 1){}
        stop = hwlib::now_us();
        return ((stop-start)/0.000058)/1000000;

    }

    

};

#endif
