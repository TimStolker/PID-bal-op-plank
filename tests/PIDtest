
#include "sonar.hpp"
#define PI 3.1415926535897932384626433832795

void write_angle( uint_fast16_t angle,  target::pin_out & servo){
    servo.write(1);
    servo.flush();
    hwlib::wait_us(500+(int32_t) angle * 2000 / 180);
    servo.write(0);
    servo.flush();
}

int main( ){    
    float angleMaxRad = PI;
    float angleMinRad = 0.0;

    //float sampleTime = 0.1;

    int setpoint, setpointPrec;
    int y, yPrec;
    int error;
    float P, I, D, U;
    float IPrec=0, DPrec = 0;
    bool saturation = false;

    float Kp = 8.0;
    float Ki = 1.0;
    float Kd = 30.0;

    namespace target = hwlib::target;
    auto servo = target::pin_out( target::pins::d8 );
    auto triggerBall = target::pin_out( target::pins::d4);
    auto echoBall = target::pin_in( target::pins::d5);
    auto triggerHand = target::pin_out( target::pins::d47);
    auto echoHand = target::pin_in( target::pins::d49);

    sonar sonarSensorBall(echoBall,triggerBall);
    sonar sonarSensorHand(echoHand, triggerHand);

    hwlib::wait_ms( 2000 );  
    hwlib::cout << "started \n";
    
    setpointPrec = sonarSensorHand.measure();
    yPrec = sonarSensorBall.measure();

    for(;;){
        setpoint = sonarSensorHand.measure();
        setpoint = (int)(0.5*setpoint + 0.5*setpointPrec); //filter
        hwlib::wait_ms(5);
        y = sonarSensorBall.measure();
        y = (int)(0.5*y + 0.5*yPrec);
        hwlib::wait_ms(5);

        error = (y-setpoint);

        P = Kp*error;

        if(!saturation){
            I = IPrec + Ki*error;
        }

        D = Kd*(y - yPrec);
        hwlib::cout<< "P: "<< int(P) << "I: " << int(I) << "D: " << int(D) << "\n";
        U = -1*(P + I + (100*D)*0.01); // U in radialen
        hwlib::cout<< "U nu: " << int(U) << "\n";
        
        ///Motor kan niet verder dan 180 graden draaien///
        if(U < angleMinRad){
            U = angleMinRad;
            saturation = true;
        }
        else if(U > angleMaxRad){
            U = angleMaxRad;
            saturation = true;
        }
        else{
            saturation = false;
        }

        U = static_cast<int>((U*180/PI)); //Zet U op in graden
        write_angle(U, servo);
        hwlib::wait_ms(50);

        
        hwlib::cout<< "U: "<<int(U) <<"\n";


        IPrec = I;
        yPrec = y;
        DPrec = D;
        setpointPrec = setpoint;
    }
}




