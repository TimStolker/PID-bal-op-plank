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
    hwlib::wait_us(500+(int32_t) angle * 2000 / 180); //zet angle om in pulse tussen 500 en 2500
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
    bool overshoot = false;

    float Kp = 6.0; //sneller reageren
    float Ki = 1.0; //steady state error oplossen
    float Kd = 1.0; //overshoot kleiner

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
    auto timeNow = 0;
    auto timeNew = hwlib::now_us();
    auto dt = 0;
    for(;;){
        setpoint = sonarSensorHand.measure();
        setpoint = (0.5*setpoint + 0.5*setpointPrev); //filter
        hwlib::wait_ms(5);
        y = sonarSensorBall.measure();
        y = (0.5*y + 0.5*yPrev);
        hwlib::wait_ms(5);

        error = round(y-setpoint)*0.01;

        P = Kp*error;

        if(!overshoot){
            I = IPrev + 0.01*Ki*error;
        }
        timeNow = hwlib::now_us();
        dt = timeNow-timeNew*0.000001;
        D = (Kd/dt)*(y - yPrev);
        D = 0.5*D + 0.5*DPrev; //filteren van D
        U = -1*(P + I + (D*dt)); // U in radialen
        timeNew = hwlib::now_us();
        //Motor kan niet verder dan 180 graden draaien
        if(U < angleMinRad){
            U = angleMinRad;
            overshoot = true;
        }
        else if(U > angleMaxRad){
            U = angleMaxRad;
            overshoot = true;
        }
        else{
            overshoot = false;
        }
        U = round((U*180/PI)); //Zet U om in graden

        write_angle(round(U), servo); //Stuur de servo aan
        
        hwlib::wait_ms(25);

        IPrev = I;
        yPrev = y;
        DPrev = D;
        setpointPrev = setpoint;
    }
}
