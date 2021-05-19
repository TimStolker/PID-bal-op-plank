
#include "sonar.hpp"



int main( ){    //test voor afstand meten met hr-sr04 ultra sonic sensor
    namespace target = hwlib::target;
    auto servo = target::pin_out( target::pins::d8 );
    auto triggerBall = target::pin_out( target::pins::d4);
    auto echoBall = target::pin_in( target::pins::d5);
    auto triggerHand = target::pin_out( target::pins::d47);
    auto echoHand = target::pin_in( target::pins::d49);

    sonar sonarSensorBall(echoBall,triggerBall);
    sonar sonarSennsorHand(echoHand, triggerHand);

    hwlib::wait_ms( 2000 );  
    hwlib::cout << "started \n";
    int distance;
    for(;;){
        distance = sonarSensorBall.measure();
        hwlib::cout<< "distance: " << distance << hwlib::endl;
        if(distance < 22){
            servo.write(1);
            servo.flush();
            hwlib::wait_us( 2500 );  
            servo.write(0);
            servo.flush();
            hwlib::wait_us( 18000 );
        }
        else if(distance > 50){
            servo.write(1);
            servo.flush();
            hwlib::wait_us( 500 );  
            servo.write(0);
            servo.flush();
            hwlib::wait_us( 19000 );
        }
        hwlib::wait_ms(100);

    } 
}



