int main( ){    //test voor afstand meten met hr-sr04 ultra sonic sensor
    namespace target = hwlib::target;
    auto servo = target::pin_out( target::pins::d8 );
    auto triggerHand = target::pin_out( target::pins::d47);
    auto echoHand = target::pin_in( target::pins::d49);
    auto triggerBall = target::pin_out( target::pins::d4);
    auto echoBall = target::pin_in( target::pins::d5);

    sonar sonarSensorHand(echoHand, triggerHand);
    sonar sonarSensorBall(echoBall, triggerBall);

    hwlib::wait_ms( 2000 );  
    hwlib::cout << "started \n";
    int distance1;
    int distance2;
    for(;;){
        distance1 = sonarSensorHand.measure();
        hwlib::wait_ms(10);
        distance2 = sonarSensorBall.measure();
        hwlib::cout<< "distance1: " << distance1 << "distance2: " << distance2 << hwlib::endl;
        hwlib::wait_ms(100);
    } 
}
