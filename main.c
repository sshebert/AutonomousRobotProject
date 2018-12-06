/*
* ECE 375 Autonomous Rover Project
* Sam Shebert, Tim Walrath, Tyrone Clarke, Jennifer Smith
* The purpose of this project was to create an autonomous robot that navigates a predefined course
* Sensors and Motors connections to HSC12 Dragon Board:
* Ping Sensor - input into PT3, output into PT1
* Line Sensors - left AD0 pin 0, right AD0 pin 1, middle ADO pin 2
* DC Motors - left motor into PP5, right motor into PP7
* Servo Motor - into PP0
* Encoders - left into PT7, right into PT0
* Buzzer (built into board) - connected through PT5
*/
#include <hidef.h>
#include <mc9s12dg256.h>
#include "main_asm.h"

// method declarations
void timer_init(void);                                // initialize timer
void PACAB_init(void);                                // initialize 16 bit pulse accumulator A and B 
void servo_init(void);                                // initialize servo motor
void buzzer_init(void);                               // initialize buzzer 
void TurnServo(int pulse);                            // pulse parameter alters PWM duty cycle to move servo
void inputPulse(void);                                // send 5us pulse on port T pin 3 to trigger ping sensor
int outputPulse(void);                                // read output from ping sensor using HILOtimes1 on port T pin 1
int getDistance(void);                                // method calls inputPulse() and outputPulse() and then converts the length of the pulse into distance in centimeters
void SetMotorL(int speed);                            // speed parameter alters PWM duty cycle to change speed of left motor
void SetMotorR(int speed);                            // speed parameter alters PWM duty cycle to change speed of right motor
void StopMotors(void);                                // sets PWM duty cycle of both motors to a 
void AdjustLeft(void);                                // slightly adjust robot to left by reversing left motor for a short time
void AdjustRight(void);                               // slightly adjust robot to right by reversing right motor for a short time
void Forwards(int pulses);                            // forward based on encoders, does not regard line sensors
void TurnRight(int pulses);                           // right based on encoders, does not regard line sensors
void Backwards(int pulses);                           // backwards based on encoders, does not regard line sensors
void TurnLeft(int pulses);                            // left based on encoders, does not regard line sensors
// 7 drive methods corresponding to different parts of the track
void Drive1(void);                                    // start of track, until first turn
int Drive2(void);                                     // first turn, past left curve, then checks if box is blocking. If box then turn left, else return from method
// 3-6 are if the box was in place
void Drive3(void);                                    // box was in place, go forwards and turn right when middle line sensor triggers
void Drive4(void);                                    // box was in place, go forwards and turn right when middle line sensor triggers
void Drive5(void);                                    // box was in place, go forwards until the in the parking area
void Drive6(void);                                    // box was in place, park2
// 7 if the box was not in place
void Drive7(void);                                    // box was not in place, park3
// 3 variations of parking depending on which path the robot took earlier
void Park1(void);                                     // parking method if no box and the first parking spot is empty
void Park2(void);                                     // parking method if box is present
void Park3(void);                                     // parking method if no box and the first parking spot is full
void ObstacleCheck(void);                             // checks ping sensor, if the ping distance is less than 25cm stop motors and sound buzzer until object is removed
void Buzzer(void);                                    // sounds buzzer
void Approach(int distance);                          // approach until ping distance is less than parameter value passed in
void DriveStraight(int leftspeed,int rightspeed);     // set motors to parameter left/right speed, check line sensors, and adjust robot accordingly

// constant motor values, used throughout program to simplify code and increase read and writability
const int SlowLeftSpeed = 4200;
const int SlowRightSpeed = 4800;
const int DefLeftSpeed = 4150;
const int DefRightSpeed = 4850;
const int FastLeftSpeed = 3900;
const int FastRightSpeed = 5100;
const int Stop = 4500;

// constant line sensor values, used throughout program to simplify code
const int MidLineDark = 230;
const int SideLineDark = 100;

void main(void) {
    int box;

    //initializations
    seg7_disable(); // disable 7 seg display
    servo_init();   // initialize servo
    servo54_init(); // initialize left motor
    servo76_init(); // initialize right motor
    ad0_enable();   // initialize a to d converter
    lcd_init();     // initialize lcd
    PACAB_init();   // initialize accumulator A and B
    buzzer_init();  // initialize buzzer
    timer_init();   // initialize timer

    while (!SW5_down());    // wait until SW5 is pressed

    Drive1();               // start program
    box = Drive2();         // box = 1 means first box is in place, box = 0 means first box is not in place

    if (box == 1) {         // if box is in place, do...
        Drive3();
        Drive4();
        Drive5();
        Drive6();
    }else if (getDistance() < 110) {       // else box is not in place and first parking space is full, do...
        Approach(17);
        Drive7();
    }else{      // if no box in the first parking spot do parking routine 1
        Park1();
    }

    //celebration, moves servo back and forth before program ends... VICTORY!
    TurnServo(9);
    TurnServo(4);
    TurnServo(12);
    TurnServo(4);
    TurnServo(12);
    TurnServo(9);
}

// initializes timer
void timer_init(void) {
    TSCR2 = 0x00;       // disable interrupt and set prescale to 1
    TSCR1 = 0x80;       // set TEN bit to 1 to enable timer

}

// initialize 2 16 bit pulse accumulators A and B
// throughout program PACN32 corresponds to left encoder and PACN10 corresponds to right encoder
void PACAB_init(void) {
    DDRT = DDRT & 0x7E;   // Pin 0 and 7 of port T input capture
    TCTL4 = TCTL4 | 0x01; // Set EDG0A to 1 (rising edge)
    PACTL = 0x50;         // PAEN=1, PAMOD=0, PEDGE =1 (rising edge), No Interrupts
    PBCTL = 0x40;         // PBEN=1
}

// initialize servo
void servo_init(void) {
    PWME = PWME | 0x01;      // enable PWM0
    PWMPOL = PWMPOL | 0x01;  // start high
    PWMCLK = PWMCLK | 0x01;  // use clock SA
    PWMSCLA = 0xFF;          // SA = 3Mhz/2*255 = 5.9kHz
    PWMCAE = 0x00;           // left align
    PWMCTL = PWMCTL & 0xEF;  // set 8-bit channel 0

    PWMPER0 = 118;           // period =~ 20ms
    PWMDTY0 = 0;             // initial duty cycle = 0%
}

// initialize buzzer
void buzzer_init(void) {
    DDRT = DDRT | 0x20;      // set port T pin 5 to output
}

// changes duty cycle servo PWM
// PWMDTY0 = 4 => left
// PWMDTY0 = 9 => forward
// PWMDTY0 = 12 => right
void TurnServo(int pulse) {
    PWMDTY0 = pulse;    // set PWMDTY0 to desired value
    ms_delay(500);      // wait for motor to move to position
    PWMDTY0 = 0;        // set PWMDTY0 to 0 to turn off motor
}

// trigger pulse for ping sensor
void inputPulse(void) {
    int x;                        // saves the time.
    int y;                        // save the flag.

    DDRT = DDRT | 0x08;           // set Port T pin 3 to be an output.
    PTT = PTT | 0x08;             // set Port T, pin 3.
    TCTL2 = 0x40;                 // set Port T, pin 3 to toggle.
    TIOS = TIOS | 0x08;           // set Port T, pin 3 to output.
    TSCR1_TEN = 1;                // enable the TCNT.
    x = TCNT;                     // read the time of the TCNT.
    TFLG1 = TFLG1 | 0x08;         // clear the C3F flag.
    x = x + 23880;                // add the high time to the time.
    TC3 = x;                      // move the time to the TC3 register.
    y = TFLG1 & 0x08;             // check for the C3F flag.

    while (y == 0) {              // wait for flag to set.
        y = TFLG1 & 0x08;         // check for the C3F flag again.
    }
    TFLG1 = TFLG1 | 0x08;         // clear the C3F flag.
    x = x + 120;                  // add the low time to the time.
    TC3 = x;                      // move the time to the TC3 register.
    y = TFLG1 & 0x08;             // check for the C3F flag.

    while (y == 0) {              // wait for the flag set.
        y = TFLG1 & 0x08;         // check for C3F flag again.
    }
}

// finds output pulse from ping sensor
int outputPulse(void) {                 // distance from object for the first straight section of the track.                                                              // Used for iterating in the for loop.
    HILO1_init();                       // initialize the high / low function.                     // Iterate 5 times
    return get_HI_time1();              // read the high time.
}

// general method for ping sensor use, returns distance in centimeters
// calls inputPulse() method and outputPulse() method
int getDistance(void) {
    int pulses;
    int dist;

    inputPulse();                         // call inputPulse method
    pulses = outputPulse();               // get number of pulses from outputPulse method
    dist = pulses / 100;                  // convert pulses to cm

    return dist;                          // return distance in cm
}

// set left motor to desired speed
void SetMotorL(int speed) {
    set_servo54(speed);
}

// set right motor to desired speed
void SetMotorR(int speed) {
    set_servo76(speed);
}

// stops both motors, stop is a global constant value
void StopMotors() {
    SetMotorR(Stop);
    SetMotorL(Stop);
}

// stops both motors and then adjusts to the left
// after the program returns from this method the motors will be set back to previous values
void AdjustLeft() {
    StopMotors();
    ms_delay(50);

    SetMotorL(FastRightSpeed);      // left motor fast backwards
    SetMotorR(FastRightSpeed);      // right motor fast forwards
    ms_delay(50);
}

// stops both motors and then adjusts to the right
// after the program returns from this method the motors will be set back to previous values
void AdjustRight() {
    StopMotors();
    ms_delay(50);

    SetMotorL(FastLeftSpeed);       // left motor fast forwards
    SetMotorR(FastLeftSpeed);       // right motor fast backwards
    ms_delay(50);
}

// move forward until the desired number encoder pulses is reached
void Forwards(int pulses) {
    PACN32 = 0;     // reset pulse accumulator A
    PACN10 = 0;     // reset pulse accumulator B

    SetMotorL(DefLeftSpeed);    // left motor forward
    SetMotorR(DefRightSpeed);   // right motor forward

    while (PACN32 < pulses);    // wait until pulse accumulator A is greater than desired number of pulses

    PACN32 = 0;     // reset pulse accumulator A
    PACN10 = 0;     // reset pulse accumulator B
}

// turn right until the desired number encoder pulses is reached
void TurnRight(int pulses) {
    PACN32 = 0;     // reset pulse accumulator A
    PACN10 = 0;     // reset pulse accumulator B

    SetMotorL(DefLeftSpeed);    // left motor forward
    SetMotorR(DefLeftSpeed);    // right motor backwards

    while (PACN32 < pulses);    // wait until pulse accumulator A is greater than desired number of pulses

    PACN32 = 0;     // reset pulse accumulator A
    PACN10 = 0;     // reset pulse accumulator B
}

// move backwards until the desired number encoder pulses is reached
void Backwards(int pulses) {
    PACN32 = 0;     // reset pulse accumulator A
    PACN10 = 0;     // reset pulse accumulator B

    SetMotorL(DefRightSpeed);   // left motor backwards
    SetMotorR(DefLeftSpeed);    // right motor backwards

    while (PACN32 < pulses);    // wait until pulse accumulator A is greater than desired number of pulses

    PACN32 = 0;     // reset pulse accumulator A
    PACN10 = 0;     // reset pulse accumulator B
}

// turn left until the desired number encoder pulses is reached
void TurnLeft(int pulses) {
    PACN32 = 0;     // reset pulse accumulator A
    PACN10 = 0;     // reset pulse accumulator B

    SetMotorL(DefRightSpeed);   // left motor backwards
    SetMotorR(DefRightSpeed);   // right motor forwards

    while (PACN10 < pulses);    // wait until pulse accumulator B is greater than desired number of pulses

    PACN32 = 0;     // reset pulse accumulator A
    PACN10 = 0;     // reset pulse accumulator B
}

// drive forward, adjust based on side line sensors, and execute right turn when middle line sensor is over black line
void Drive1(void) {
    int MiddleLine;

    while (1) {
        DriveStraight(DefLeftSpeed, DefRightSpeed);     // drive forwards at default speed and adjust based on side line sensors

        MiddleLine = ad0conv(2) >> 2;                   // read ad0 port 2 and shift 2 bits to right

        if (MiddleLine > MidLineDark) {                 // if middle line sensor hits black line execute right turn
            // execute right turn with values based on testing
            Backwards(13);                              // backup to stay in bounds
            TurnRight(87);                              // turn right 87 encoder pulses
            break;                                      // break out of while loop, ultimately ending method
        }
    }
}

// drive forwards, adjust based on side line sensors
// if first box is in play execute left curve, else return from method
int Drive2(void) {
    int encoderDist = 0;    // total encoder distance
    PACN10 = 0;     // reset pulse accumulator B

    while (1) {
        DriveStraight(DefLeftSpeed, DefRightSpeed);     // drive forwards at default speed and adjust based on side line sensors

        // checks obstacle is in front every 10 accumulator pulses
        if (PACN10 > 10) {

            encoderDist = encoderDist + PACN10;     // remember total encoder pulses
            PACN10 = 0;     // reset pulse accumulator B

            if (encoderDist < 500)      // will only look for obstacles if the robot is before the left curve
                ObstacleCheck();        // if obstacle is in place stops robot and sounds buzzer until object removes
        }

        // checks if first box is in place after left curve, which is after approximately 615 encoder pulses
        if (encoderDist > 615) {
            int distance;
            StopMotors();   // stop both motors
            PACN10 = 0;     // reset pulse accumulator B

            distance = getDistance();       // get distance from ping sensor

            if (distance < 40) {        // if distance is less than 40 cm, box must be in place
                Approach(20);       // approach the box until robot is less than 20cm away

                // execute left turn with pulse values based on testing
                // moving forwards between each small turn prevents back wheels from touching the black line
                TurnLeft(15);
                Forwards(5);
                TurnLeft(15);
                Forwards(5);
                TurnLeft(22);
                Forwards(5);
                TurnLeft(36);

                return 1;       // return 1, box in place
            }
            return 0;       // return 0, box not in place
        }
    }
}

// box 1 was in place, drive forwards, adjust based on side line sensors
// turn right when middle line sensor is over black line
void Drive3(void) {
    int MiddleLine;
    PACN10 = 0;     // reset pulse accumulator B

    while (1) {
        DriveStraight(DefLeftSpeed, DefRightSpeed);     // drive forwards at default speed and adjust based on side line sensors

        MiddleLine = ad0conv(2) >> 2;       // Read ad0 port 2 and shift 2 bits to right

        if (MiddleLine > MidLineDark) {           //if middle line sensor hits black line
            // execute right turn with pulse values based on testing
            Backwards(8);
            TurnRight(85);
            break;      // break from while loop and ultimately method
        }
        // checks obstacle is in front every 10 accumulator pulses
        if (PACN10 > 10) {
            PACN10 = 0;     // reset pulse accumulator B
            ObstacleCheck();        // if obstacle is in place stops robot and sounds buzzer until object removes
        }
    }
}

// box 1 was in place, drive forwards, adjust based on side line sensors
// turn right when middle line sensor is over black line
void Drive4(void) {
    int MiddleLine;

    while (1) {
        DriveStraight(DefLeftSpeed, DefRightSpeed);     // drive forwards at default speed and adjust based on side line sensors

        MiddleLine = ad0conv(2) >> 2;       // Read ad0 port 2 and shift 2 bits to right

        if (MiddleLine > MidLineDark) {       // if middle line sensor hits black line
            // execute right turn with pulse values based on testing
            Backwards(13);
            TurnRight(87);
            break;      // break from while loop and ultimately method
        }
    }
}

// box 1 was in place, drive forwards, adjust based on side line sensors
// after 270 encoder pulses the robot is in parking area, return from method
void Drive5(void) {
    int MiddleLine;
    // reset pulse accumulators
    PACN10 = 0;
    PACN32 = 0;

    while (1) {
        DriveStraight(DefLeftSpeed, DefRightSpeed);     // drive forwards at default speed and adjust based on side line sensors

        MiddleLine = ad0conv(2) >> 2;   // read ad0 port 2 and shift 2 bits to right

        if (MiddleLine > MidLineDark) {           // if middle line sensor hits black line
            // execute left turn with pulse values based on testing
            Backwards(11);
            TurnLeft(64);
            StopMotors();
            break;      // break from while loop and ultimately method
        }
        // checks obstacle is in front every 10 accumulator pulses
        if (PACN10 > 10) {
            PACN10 = 0;     // reset pulse accumulator B
            ObstacleCheck();        // if obstacle is in place stops robot and sounds buzzer until object removes
        }
        // after 270 encoder pulses the start of the parking area has been reached
        if (PACN32 > 270) {
            StopMotors();      // stop both motors
            TurnServo(13);     // turn ping sensor left
            break;      // break from while loop and ultimately method
        }
    }
}
// box 1 was in place, drive forwards, adjust based on side line sensors
// checks each  parking space and parks in the first available spot
void Drive6(void) {
    // if box is not in first parking spot then park
    if (getDistance() > 30) {
        Park2();
        return;
    }
    PACN32 = 0;     // reset pulse accumulator A
    while (1) {
        DriveStraight(DefLeftSpeed, DefRightSpeed);     // drive forwards at default speed and adjust based on side line sensors
        // after 135 encoder pulses the next parking spot has been reached
        if (PACN32 > 135) {
            break;      // break from while loop
        }
    }
    // if box is not in second parking spot then park
    if (getDistance() > 30) {
        Park2();
        return;
    }
    PACN32 = 0;     // reset pulse accumulator A
    while (1) {
        DriveStraight(DefLeftSpeed, DefRightSpeed);     // drive forwards at default speed and adjust based on side line sensors
        // after 140 encoder pulses the next parking spot has been reached
        if (PACN32 > 140) {
            break;      // break from while loop
        }
    }
    // if first 2 parking spots are taken then last spot must be open, so park
    Park2();
}
// box 1 was NOT in place and first parking spot is full
// turn left and then check the next parking spots
void Drive7(void) {
    int pingDist;
    StopMotors();       // stop motors
    TurnServo(4);       // turn servo to the right
    // execute left turn with pulse values based on testing
    TurnLeft(15);
    Forwards(5);
    TurnLeft(15);
    Forwards(5);
    TurnLeft(20);
    Forwards(5);
    TurnLeft(33);
    PACN32 = 0;     // reset pulse accumulator A
    while (1) {
        DriveStraight(DefLeftSpeed, DefRightSpeed);     // drive forwards at default speed and adjust based on side line sensors
        // after 120 encoder pulses the next parking spot has been reached
        if (PACN32 > 120) {
            break;      // break from while loop
        }
    }
    // if box is not in second parking spot then park
    if (getDistance() > 30) {
        Park3();
        return;
    }
    PACN32 = 0;     // reset pulse accumulator A
    while (1) {
        DriveStraight(DefLeftSpeed, DefRightSpeed);     // drive forwards at default speed and adjust based on side line sensors
        // after 120 encoder pulses the next parking spot has been reached
        if (PACN32 > 120) {
            break;      // break from while loop
        }
    }
    Park3();
}
// if first block was not blocking path and first parking spot is open, go straight until front line sensor hits black line
void Park1(void) {
    int mid;

    while (1) {
        DriveStraight(SlowLeftSpeed, SlowRightSpeed);       // drive forwards at slow speed and adjust based on side line sensors
        mid = ad0conv(2) >> 2;      //Read ad0 port 2 and shift 2 bits to right

        if (mid > MidLineDark) {        // if middle line sensor hits black line
            StopMotors();
            break;
        }
    }
}
// parking method if box was present
void Park2(void) {
    int mid;
    // execute left turn with pulse values based on testing
    TurnLeft(15);
    Forwards(5);
    TurnLeft(15);
    Forwards(5);
    TurnLeft(20);
    Forwards(5);
    TurnLeft(35);

    while (1) {
        DriveStraight(SlowLeftSpeed, SlowRightSpeed);       // drive forwards at slow speed and adjust based on side line sensors
        mid = ad0conv(2) >> 2;      //Read ad0 port 2 and shift 2 bits to right

        if (mid > MidLineDark) {        // if middle line sensor hits black line
            StopMotors();
            break;
        }
    }
}

void Park3(void) {
    int mid;
    // execute right turn with pulse values based on testing
    TurnRight(35);
    Forwards(5);
    TurnRight(30);
    Forwards(5);
    TurnRight(25);

    while (1) {
        DriveStraight(SlowLeftSpeed, SlowRightSpeed);       // drive forwards at slow speed and adjust based on side line sensors
        mid = ad0conv(2) >> 2;      //Read ad0 port 2 and shift 2 bits to right

        if (mid > MidLineDark) {        // if middle line sensor hits black line
            StopMotors();
            break;
        }
    }
}
// checks if object is within 25 cm of ping sensor
// if object is present stop the robot and sound the buzzer until object is removed
void ObstacleCheck(void) {
    int pingDist;

    pingDist = getDistance();       // get distance from ping sensor

    if (pingDist < 25 && pingDist > 1) {        // is distance is less than 25 cm or greater than 1 cm (to remove irregularities from ping sensor
        StopMotors();       // stop both motors
        // sound buzzer and stop robot until ping distance is greater than 25
        do {
            Buzzer();       // sound buzzer
            pingDist = getDistance();       // get new ping distance
        } while (pingDist < 25 && pingDist > 1);        // break from while loop when pingDist is greater than 25
    }
}
// method sounds buzzer for approximately 1.8 seconds
void Buzzer(void) {
    int i;

    for (i = 0; i < 300; i++) {      // repeat 300 times
        PTT = 0x20;                  // pull port t pin 5 high
        ms_delay(3);                 // changing delay changes pitch
        PTT = 0x00;                  // pull port t pin 5 low
        ms_delay(3);                 // changing delay changes pitch
    }
}
// approach until the ping sensor is within desired distance
void Approach(int distance) {
    while (1) {
        DriveStraight(SlowLeftSpeed, SlowRightSpeed);       // drive forwards at slow speed and adjust based on side line sensors
        if (getDistance() < distance)       // once the ping sensors distance is less than the desired distance
            break;      // break from while loop
    }
}

void DriveStraight(int leftspeed, int rightspeed) {
    int LeftLine, RightLine;
    LeftLine = ad0conv(0) >> 2;     //Read ad0 port 0 and shift 2 bits to right
    RightLine = ad0conv(1) >> 2;    //Read ad0 port 1 and shift 2 bits to right


    if (LeftLine > SideLineDark) {             //if left line sensor hits black line
        AdjustRight();
    } else if (RightLine > SideLineDark) {            //if left line sensor hits black line
        AdjustLeft();
    } else {        // else set motor speeds to parameter speed
        SetMotorL(leftspeed);
        SetMotorR(rightspeed);
    }
}

// handler for HILOtimes1 interrupt
void interrupt9 handler1() {
    HILOtimes1();
}

