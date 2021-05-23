
#include "sonar.hpp"
#define PI 3.1415926535897932384626433832795

int map(int x, int inputMin, int inputMax, int outputMin, int outputMax){
    return (x - inputMin) * (outputMax - outputMin) / (inputMax - inputMin) + outputMin;
}

int absolute(int N){
    if(N<0){
        N = -1*N;
    }
    return N;
}

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

    float setpoint, setpointPrev;
    float y, yPrev;
    float error;
    float P, I, D, U;
    float IPrev=0, DPrev = 0;
    bool saturation = false;

    float Kp = 1.0;
    float Ki = 1.0;
    float Kd = 1.0;

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

    setpointPrev = sonarSensorHand.measure();
    yPrev = sonarSensorBall.measure();

    for(;;){
        setpoint = sonarSensorHand.measure();
        setpoint = (0.53*setpoint + 0.47*setpointPrev); //filter
        hwlib::wait_ms(5);
        y = sonarSensorBall.measure();
        y = (0.53*y + 0.47*yPrev);
        hwlib::wait_ms(5);

        error = round(y-setpoint)*0.01;
        hwlib::cout<<"Error: "<<int(error)<<" y: "<<int(y)<<" setpoint: "<<int(setpoint)<<"\n";

        P = Kp*error;

        if(!saturation){
            I = IPrev + 0.09*Ki*error;
        }

        D = (Kd/0.09)*(y - yPrev);
        D = 0.56*D + 0.44*DPrev; //filteren van D
        U = -1*(P + I + (D*0.01)); // U in radialen

        hwlib::cout<<"U: "<<int(U)<<"\n";

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
        U = round((U*180/PI)); //Zet U op in graden

        write_angle(round(U), servo);

        
        hwlib::wait_ms(25);

        IPrev = I;
        yPrev = y;
        DPrev = D;
        setpointPrev = setpoint;
    }
}




