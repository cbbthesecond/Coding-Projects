/*********************************************/
/*      Team A3 FEH Robot Project Code       */
/*            OSU FEH Spring 2022            */
/*                                           */
/*      Steven Broaddus, Conolly Burgess     */
/*        Joseph Richmond, Jake Chang        */
/*                                           */
/*            Updated 4/18/2022              */
/*       Uses Doxygen for documentation      */
/*********************************************/

/************************************************/
// Include preprocessor directives
#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHRPS.h>
#include <FEHServo.h>
#include <cmath> // abs() 

/************************************************/
// Definitions
#define ROBOT_WIDTH 7.95 // Length of front/back side of OUR robot in inches
#define PI 3.14159265
#define BACKGROUND_COLOR WHITE // Background color of layout
#define FONT_COLOR BLACK // Font color of layout

// Movement/Dimension calculations
#define DIST_AXIS_CDS 4.125 // Distance from the center of the wheel axis to the CdS cell. (5.375 - 1.25)
#define COUNT_PER_INCH (318 / (2 * 3.14159265 * 1.25)) // Number of encoder counts per inch ((ENCODER_COUNTS_PER_REV / (2 * PI * WHEEL_RADIUS))) 
#define INCH_PER_COUNT ((2 * 3.14159265 * 1.25) / 318) // ^ but opposite

// Precise movement calibrations
#define BACKWARDS_CALIBRATOR 2.4 // Percent difference needed to make backward motors move the same as forward motors at 20%. Initially 2.15
#define RIGHT_MOTOR_CALIBRATOR 1 

// Servo min/max values
#define BASE_SERVO_MIN 500
#define BASE_SERVO_MAX 2290
#define ON_ARM_SERVO_MIN 500
#define ON_ARM_SERVO_MAX 2400

// Speeds the robot uses
#define FORWARD_SPEED 45
#define TURN_SPEED 30
#define RAMP_SPEED 50

// RPS pulse values
#define RPS_DELAY_TIME 0.35 // Time that the RPS takes to check again before correcting

#define RPS_TURN_PULSE_PERCENT 20 // Percent at which motors will pulse to correct movement while turning
#define RPS_TURN_PULSE_TIME 0.08 // Time that the wheels pulse for to correct heading. Originally 0.05.
#define RPS_TURN_THRESHOLD 0.5 // Degrees that the heading can differ from before calling it a day

#define RPS_TRANSLATIONAL_PULSE_PERCENT 20 // Percent at which motors will pulse to correct translational movement
#define RPS_TRANSLATIONAL_PULSE_TIME 0.1 // Time that the wheels pulse for to correct translational coords
#define RPS_TRANSLATIONAL_THRESHOLD 0.25 // Coord units that the robot can be in range of

/************************************************/
// Global variables for RPS values

// Headings
float RPS_0_Degrees = 0;
float RPS_90_Degrees = 90;
float RPS_180_Degrees = 180;
float RPS_270_Degrees = 270;

// That one spot on the top level ()
float RPS_Top_Level_X_Reference = 15.45;
float RPS_Top_Level_Y_Reference = 52.25;

// *****************************************
// Global variables for PID

// PID Right Motor
double PID_Linear_SpeedR, PID_New_CountsR, PID_Last_CountsR, PID_New_TimeR, PID_Last_TimeR, PID_New_Speed_ErrorR, PID_Last_Speed_ErrorR, PID_Error_SumR;
float PID_NEW_MOTOR_POWERR, PID_OLD_MOTOR_POWERR;
double PTermR, ITermR, DTermR;

double PConstR = 0.75;
double IConstR = 0.05; 
double DConstR = 0.25;

// PID Left Motor
double PID_Linear_SpeedL, PID_New_CountsL, PID_Last_CountsL, PID_New_TimeL, PID_Last_TimeL, PID_New_Speed_ErrorL, PID_Last_Speed_ErrorL, PID_Error_SumL;
float PID_NEW_MOTOR_POWERL, PID_OLD_MOTOR_POWERL;
double PTermL, ITermL, DTermL;

double PConstL = 0.75;
double IConstL = 0.05; 
double DConstL = 0.25;

// Both
double PID_TIME;
double PID_DISTANCE_PER_COUNT = ((2 * 3.14159265 * 1.25) / 318);

#define SLEEP_PID 0.15

/************************************************/
// Course numbers. Used in start_menu() and run_course()
enum { 
            TEST_COURSE_1 = 1, 
            TEST_COURSE_2 = 2, 
            TEST_COURSE_3 = 3, 
            CALIBRATE_SERVOS = 4,
            PERF_COURSE_1 = 5, 
            PERF_COURSE_2 = 6, 
            PERF_COURSE_3 = 7, 
            PERF_COURSE_4 = 8, 
            IND_COMP = 9, 
            FINAL_COMP = 10
         };

/************************************************/
// Function Prototypes (For reference, these don't actually do anything)
void update_RPS_Heading_values(double timeToCheck, bool checking_heading, bool checking_x, bool checking_y); 
// ^ Updates RPS values across the course
int read_start_light(double timeToCheck); // Waits for the start light with a timeout
void move_forward_inches(int percent, float inches); // Moves forward number of inches
void move_forward_seconds(float percent, float seconds); // Moves forward for a number of seconds
void turn_right_degrees(int percent, float degrees); // Turns right a specified number of degrees
void turn_left_degrees(int percent, float degrees); // Turns left a specified amount of degrees
void RPS_correct_heading(float heading, double timeToCheck); // Corrects the heading of the robot using RPS
void RPS_check_x(float x_coord, double timeToCheck); // Corrects the x-coord of the robot using RPS
void RPS_check_y(float y_coord, double timeToCheck); // Corrects the y-coord of the robot using RPS
void ResetPIDVariables(); // Resets PID variables
float RightPIDAdjustment(double expectedSpeed); // Corrects right motor based on speed, counts, and expected speed
float LeftPIDAdjustment(double expectedSpeed); // Corrects left motor based on speed, counts, and expected speed
void move_forward_PID(float in_per_sec, float inches); // Uses PID to move forward a specific amount of inches
void initiate_servos(); // Initiates servos
int detect_color(int timeToDetect); // Detects the color of the jukebox with timeout
void press_jukebox_buttons(); // Presses the jukebox buttons
void flip_burger(); // Flips the hot plate and burger
void flip_ice_cream_lever(); // Flips the correct ice cream lever
void write_status(const char status[]); // Clears room for a printed string w/o clearing entire display
void show_RPS_data(); // Shows basic RPS data for the robot
void run_course(int courseNumber); // Runs the specified course

/************************************************/
// Declarations for encoders/motors
// WHITE ENCODER -> LEFT MOTOR
DigitalEncoder right_encoder(FEHIO::P3_2);
DigitalEncoder left_encoder(FEHIO::P3_1);
FEHMotor right_motor(FEHMotor::Motor2,9.0);
FEHMotor left_motor(FEHMotor::Motor3,9.0);

// Declarations for servos
// GROUND FARTHER SIDE
FEHServo base_servo(FEHServo::Servo5);
FEHServo on_arm_servo(FEHServo::Servo7);

// Declaration for CdS cell sensorsad 
AnalogInputPin CdS_cell(FEHIO::P0_7);

/*******************************************************
 * @brief Updates RPS values by placing the robot in 90 degrees and in specific x/y coordinates on top platform (15.45, 52.25)
 * 
 * @param timeToCheck time to check heading/x/y before timing out
 * @param checking_heading true if checking heading values (90 degrees), false if not
 * @param checking_x true if checking top x coordinate (15,45), false if not
 * @param checking_y true if checking top y coordinate (52.25), false if not 
 */
void update_RPS_Heading_values(double timeToCheck, bool checking_heading, bool checking_x, bool checking_y) {
    
    int xGarb420, yGarb420;
    float tempHeading, tempX, tempY;

    // Used for timeout.
    double startTime = TimeNow();

    // Provides input if function can move on or not
    bool checkDone = false;

    // Confirmation screens pop up after each check. 
    // The order is as follows: Red, Green, Blue, Yellow

    //**********************************************// 
    // Sets heading values 

    // Sets screen to red
    LCD.SetBackgroundColor(RED);
    LCD.Clear();

    if (checking_heading) {
        write_status("Set 90 degrees");

        Sleep(1.0);
        LCD.ClearBuffer();

        // Waits until a touch (that records real RPS coordinates) is registered or until time is up. MUST HOLD PRESS SINCE IT SLEEPS.     
        while(!checkDone) {
            if ((LCD.Touch(&xGarb420, &yGarb420) && (RPS.Heading() >= 0)) || (TimeNow() - startTime >= timeToCheck)) {
                checkDone = true;
            }
            if (!checkDone) {
                Sleep(RPS_DELAY_TIME);
            }
        }
        //while((!LCD.Touch(&xGarb420, &yGarb420) && (RPS.Heading() >= 0)) || (TimeNow() - startTime >= timeToCheck));

        if ((TimeNow() - startTime < timeToCheck)) {
            tempHeading = RPS.Heading();

            // Makes sure heading isn't way off from 90, then records it
            if (abs(tempHeading - 90) < 3) {
                RPS_90_Degrees = RPS.Heading();
                RPS_180_Degrees = RPS_90_Degrees + 90;
                RPS_270_Degrees = RPS_90_Degrees + 180;

                RPS_0_Degrees = RPS_90_Degrees - 90;

                if (RPS_0_Degrees < 0) {
                    RPS_0_Degrees + 360;
                }
            }
        }        
    }

    //**********************************************// 
    // Sets x/y values for top level RPS Reference value ()
    // Sets screen to green
    LCD.SetBackgroundColor(GREEN);
    LCD.Clear();

    if (checking_x) {
        write_status("Set X for RPS"); // Facing left (180 degrees) 

        Sleep(1.0);
        LCD.ClearBuffer();

        checkDone = false;

        // Waits until a touch (that records real RPS coordinates) is registered or until time is up. MUST HOLD PRESS SINCE IT SLEEPS.    
        while(!checkDone) {
            if ((LCD.Touch(&xGarb420, &yGarb420) && (RPS.Heading() >= 0)) || (TimeNow() - startTime >= timeToCheck)) {
                checkDone = true;
            }
            if (!checkDone) {
                Sleep(RPS_DELAY_TIME);
            }
        }
        //while((!LCD.Touch(&xGarb420, &yGarb420) && (RPS.X() >= 0)) || (TimeNow() - startTime >= timeToCheck));

        if ((TimeNow() - startTime < timeToCheck)) {
            tempX = RPS.X();

            if (abs(tempX - 15.45) < 3) {
                RPS_Top_Level_X_Reference = RPS.X();
            }
        }    
    }

    // Sets screen to blue
    LCD.SetBackgroundColor(BLUE);
    LCD.Clear();

    if (checking_y) {
         write_status("Set Y for RPS"); // Facing left (180 degrees) 

        Sleep(1.0);
        LCD.ClearBuffer();

        checkDone = false;

        // Waits until a touch (that records real RPS coordinates) is registered or until time is up. MUST HOLD PRESS SINCE IT SLEEPS.
        while(!checkDone) {
            if ((LCD.Touch(&xGarb420, &yGarb420) && (RPS.Heading() >= 0)) || (TimeNow() - startTime >= timeToCheck)) {
                checkDone = true;
            }
            if (!checkDone) {
                Sleep(RPS_DELAY_TIME);
            }
        }
        //while((!LCD.Touch(&xGarb420, &yGarb420) && (RPS.Y() >= 0)) || (TimeNow() - startTime >= timeToCheck));

        if ((TimeNow() - startTime < timeToCheck)) {
            tempY = RPS.Y();

            if (abs(tempY - 52.25) < 3) {
                RPS_Top_Level_Y_Reference = RPS.Y();
            }
        }
    }

    //**********************************************// 
    // DONE

    // Sets screen to yellow
    LCD.SetBackgroundColor(YELLOW);
    LCD.Clear();

    if (checking_heading || checking_x || checking_y) {
        Sleep(1.0);
        LCD.ClearBuffer();

        // Waits until touch
        while(!LCD.Touch(&xGarb420, &yGarb420));
    }

    // Clears the screen
    LCD.SetBackgroundColor(BACKGROUND_COLOR);
    LCD.Clear();
}

/*******************************************************
 * @brief Waits until the start light to run the course
 * 
 * @param timeToCheck time allotted to check for start light before timeout
 * @return int The status of the light
 *         1 -> ON
 *         0 -> OFF
 */
int read_start_light(double timeToCheck) {
    LCD.Clear();

    double startTime = TimeNow();

    int lightOn = 0;

    write_status("Waiting for light");

    // Waits until light is detected, or until time allotted is up (timeout)
    while ((startTime - TimeNow() < timeToCheck) && !lightOn) {
        // Writes out CdS value to the screen
        LCD.WriteRC("CdS Value: ", 7, 2);
        LCD.WriteRC(CdS_cell.Value(), 7, 20);

        // Checks if any light is detected.
        if (CdS_cell.Value() < 0.5) {
            lightOn = 1;
            write_status("GO!");
        }
    }

    return lightOn;
}

/*******************************************************
 * @brief Moves CENTER OF ROBOT forward a number of inches using encoders
 * 
 * @param percent - Percent for the motors to run at. Negative for reverse.
 * @param inches - Inches to move forward .
 */
void move_forward_inches(int percent, float inches) {
    // Calculates desired counts based on the radius of the wheels and the robot
    float expectedCounts = COUNT_PER_INCH * inches;

    // Clears space for movement data and status
    LCD.SetFontColor(BACKGROUND_COLOR);
    LCD.FillRectangle(0,100,319,239);
    LCD.SetFontColor(FONT_COLOR);
    LCD.DrawHorizontalLine(100, 0, 319);

    // Writes out status to the screen
    LCD.WriteRC("Moving forward...", 7, 1);

    // Resets encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

    // Sets both motors to same percentage, but accounts for one motor moving backwards
    right_motor.SetPercent(percent);
    left_motor.SetPercent(percent);

    // Keeps running until average motor counts are in proper range
    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < expectedCounts);

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();

    //Print out data
    LCD.WriteRC("Theoretical Counts: ", 9, 1);
    LCD.WriteRC(expectedCounts, 9, 20);
    LCD.WriteRC("Motor Percent: ", 10, 1);
    LCD.WriteRC(percent, 10, 20);
    LCD.WriteRC("Actual LE Counts: ", 11, 1);
    LCD.WriteRC(left_encoder.Counts(), 11, 20);
    LCD.WriteRC("Actual RE Counts: ", 12, 1);
    LCD.WriteRC(right_encoder.Counts(), 12, 20);
}

/******************************************************* 
 * @brief Moves forward for the specified time at the specified percentage.
 * 
 * @param percent Percent that the motors will drive at
 * @param seconds Time that the motors will drive for
 */
void move_forward_seconds(float percent, float seconds) {

    if (percent < 0) {
        percent -= BACKWARDS_CALIBRATOR;
    }
    
    // Set both motors to passed percentage
    right_motor.SetPercent(percent);
    left_motor.SetPercent(percent);

    Sleep(seconds);

    // Turns off motors after elapsed time
    right_motor.Stop();
    left_motor.Stop();
}

/*******************************************************
 * @brief Turns right a certain amount of degrees
 * 
 * @param percent - Percent for the motors to run at. Negative for reverse.
 * @param degrees - Degrees to rotate.
 */
void turn_right_degrees(int percent, float degrees) {

    // Calculates desired counts based on the radius of the wheels and the robot
    float expectedCounts = COUNT_PER_INCH * ((degrees * PI) / 180.0) * (ROBOT_WIDTH / 2);

    // Clears space for movement data and status
    LCD.SetFontColor(BACKGROUND_COLOR);
    LCD.FillRectangle(0,100,319,239);
    LCD.SetFontColor(FONT_COLOR);
    LCD.DrawHorizontalLine(100, 0, 319);

    // Writes out status to the screen
    LCD.WriteRC("Turning Right...", 7, 2);

    // Resets encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

    // Sets both motors to specific percentage
    right_motor.SetPercent(-percent - BACKWARDS_CALIBRATOR);
    left_motor.SetPercent(percent);

    // Keeps running until average motor counts are in proper range
    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < expectedCounts);

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();

    //Print out data
    LCD.WriteRC("Theoretical Counts: ", 9, 1);
    LCD.WriteRC(expectedCounts, 9, 20);
    LCD.WriteRC("Motor Percent: ", 10, 1);
    LCD.WriteRC(percent, 10, 20);
    LCD.WriteRC("Actual LE Counts: ", 11, 1);
    LCD.WriteRC(left_encoder.Counts(), 11, 20);
    LCD.WriteRC("Actual RE Counts: ", 12, 1);
    LCD.WriteRC(right_encoder.Counts(), 12, 20);
}

/*******************************************************
 * @brief Turns left a certain amount of degrees
 * 
 * @param percent - Percent for the motors to run at. Negative for reverse.
 * @param degrees - Degrees to rotate.
 */
void turn_left_degrees(int percent, float degrees) {

    // Calculates desired counts based on the radius of the wheels and the robot
    float expectedCounts = COUNT_PER_INCH * ((degrees * PI) / 180.0) * (ROBOT_WIDTH / 2);

    // Clears space for movement data and status
    LCD.SetFontColor(BACKGROUND_COLOR);
    LCD.FillRectangle(0,100,319,239);
    LCD.SetFontColor(FONT_COLOR);
    LCD.DrawHorizontalLine(100, 0, 319);

    // Writes out status to the screen
    LCD.WriteRC("Turning Left...", 7, 2);

    // Resets encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

    // Sets both motors to specific percentage
    right_motor.SetPercent(percent);
    left_motor.SetPercent(-percent - BACKWARDS_CALIBRATOR);

    // Keeps running until average motor counts are in proper range
    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < expectedCounts);

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();
    
    //Print out data
    LCD.WriteRC("Theoretical Counts: ", 9, 1);
    LCD.WriteRC(expectedCounts, 9, 20);
    LCD.WriteRC("Motor Percent: ", 10, 1);
    LCD.WriteRC(percent, 10, 20);
    LCD.WriteRC("Actual LE Counts: ", 11, 1);
    LCD.WriteRC(left_encoder.Counts(), 11, 20);
    LCD.WriteRC("Actual RE Counts: ", 12, 1);
    LCD.WriteRC(right_encoder.Counts(), 12, 20);
}

/*******************************************************
 * @brief Corrects the heading using RPS, pulses to correct heading
 * 
 * @param heading Heading to correct to in degrees
 * @param secondsToCheck Time allotted before timeout
 */
void RPS_correct_heading(float heading, double secondsToCheck) {
    /*
     * Determines the direction to turn to get to the desired heading faster
     * 1 -> CW
     * -1 -> CCW
     */ 
    int direction = 0;

    // Difference between the actual heading and the desired one
    float difference;
    if ((heading - RPS.Heading()) > 180) {
        difference = abs((heading) - (RPS.Heading() + 360)); 
    } else if ((RPS.Heading() - heading) > 180) {
        difference = abs((heading + 360) - (RPS.Heading())); 
    } else {
        difference = abs((heading + 360) - (RPS.Heading() + 360)); 
    }

    double startTime = TimeNow();
    
    // Determine the direction of the motors based on the orientation of the QR code
    int power = RPS_TURN_PULSE_PERCENT;

    // Check if receiving proper RPS coordinates and whether the robot is within an acceptable range
    while(((RPS.Heading() >= 0) && (difference > RPS_TURN_THRESHOLD)) && (TimeNow() - startTime < secondsToCheck))
    {
        // Checks which way to turn to turn the least
        if (RPS.Heading() < heading) {
            direction = 1;
        } else {
            direction = -1;
        }

        if (abs(RPS.Heading() - heading) > 180) {
            direction = -direction;
        }

        // Pulses towards the ideal position
        if (direction == 1) {

            // PULSES COUNTERCLOCKWISE
            // Set both motors to desired percent
            right_motor.SetPercent(RPS_TURN_PULSE_PERCENT);
            left_motor.SetPercent(-RPS_TURN_PULSE_PERCENT - BACKWARDS_CALIBRATOR);

            // Wait for the correct number of seconds
            Sleep(RPS_TURN_PULSE_TIME);

            // Turn off motors
            right_motor.Stop();
            left_motor.Stop();
            
        } else if (direction == -1) {

            // PULSES CLOCKWISE
            // Set both motors to desired percent
            right_motor.SetPercent(-RPS_TURN_PULSE_PERCENT - BACKWARDS_CALIBRATOR);
            left_motor.SetPercent(RPS_TURN_PULSE_PERCENT);

            // Wait for the correct number of seconds
            Sleep(RPS_TURN_PULSE_TIME);

            // Turn off motors
            right_motor.Stop();
            left_motor.Stop();

        }

        if (TimeNow() - startTime > secondsToCheck) {
            break;
        }

        // Waits a tiny bit before checking RPS again
        Sleep(RPS_DELAY_TIME);

        // Updates direction to turn to
        if ((heading - RPS.Heading()) > 180) {
            difference = abs((heading) - (RPS.Heading() + 360)); 
        } else if ((RPS.Heading() - heading) > 180) {
            difference = abs((heading + 360) - (RPS.Heading())); 
        } else {
            difference = abs((heading + 360) - (RPS.Heading() + 360)); 
        }

        show_RPS_data();
    }
}

/*******************************************************
 * @brief Checks and corrects the x-coord of the robot using RPS. Makes sure the robot is facing 
 * east/west to correct movement.
 * 
 * @param x_coord Desired x-coord of the robot
 * @param secondsToCheck Time to check before timeout
 */
void RPS_check_x(float x_coord, double secondsToCheck) {

    // Directions the robot needs to be facing to correct x-coord
    enum { EAST, WEST };

    // Initial heading of the robot
    float orientation = RPS.Heading();

    // Direction the robot is corrected to
    int direction;

    // Half of the time given to check is given to correcting orientation, half is given to correcting translational position
    double halfTimeToCheck = secondsToCheck / 2;

    double startTime = TimeNow();

    // Makes sure robot can be seen by RPS
    write_status("Correcting x with RPS");
    
    // Adjusts robot to be facing east/west based on initial orientation
    // Takes half time to check orientation. If orientation can't be corrected then don't perform corrections
    bool orientationRecorded = false; // Checks if time has been taken to check orientation
    while ((TimeNow() - startTime < halfTimeToCheck) && !orientationRecorded) {
        if ((orientation <= 90) || (orientation >= 270)) {
            RPS_correct_heading(0, halfTimeToCheck);
            direction = EAST;
            orientationRecorded = true;
        } else if ((orientation > 90) && (orientation < 270)) {
            RPS_correct_heading(180, halfTimeToCheck);
            direction = WEST;
            orientationRecorded = true;
        }
    }

    if (orientationRecorded) {
        // Determine the direction of the motors based on the direction the robot is facing
        int power = RPS_TRANSLATIONAL_PULSE_PERCENT;
        if(direction == WEST){
            power = -RPS_TRANSLATIONAL_PULSE_PERCENT;
        }
        
        startTime = TimeNow();

        // Check if receiving proper RPS coordinates and whether the robot is within an acceptable range
        while ((RPS.X() > 0) && (abs(RPS.X() - x_coord) > RPS_TRANSLATIONAL_THRESHOLD) && (TimeNow() - startTime < halfTimeToCheck))  
        {
            if(RPS.X() > x_coord)
            {
                // Pulse the motors for a short duration in the correct direction
                move_forward_seconds(-power, RPS_TRANSLATIONAL_PULSE_TIME);
            }
            else if(RPS.X() < x_coord)
            {
                // Pulse the motors for a short duration in the correct direction
                move_forward_seconds(power, RPS_TRANSLATIONAL_PULSE_TIME);
            }

            if (TimeNow() - startTime > secondsToCheck) {
                break;
            }

            Sleep(RPS_DELAY_TIME);

            show_RPS_data();

        }
    }
    else {
        write_status("ERROR. RPS NOT READING.");
    }
    
}

/*******************************************************
 * @brief Checks and corrects the y-coord of the robot using RPS. Makes sure the robot is facing 
 * north/south to correct movement.
 * 
 * @param y_coord Desired y-coord of the robot
 * @param secondsToCheck Time to check before timeout
 */
void RPS_check_y(float y_coord, double secondsToCheck) {

    // Directions the robot needs to be facing to correct y-coord
    enum { NORTH, SOUTH };

    // Initial heading of the robot
    float orientation = RPS.Heading();

    // Direction the robot is corrected to
    int direction;

    // Half of the time given to check is given to correcting orientation, half is given to correcting translational position
    double halfTimeToCheck = secondsToCheck / 2;

    double startTime = TimeNow();

    // Makes sure robot can be seen by RPS
    write_status("Correcting y with RPS");

    // Adjusts robot to be facing east/west based on initial orientation
    // Takes half time to check orientation. If orientation can't be corrected then don't perform corrections
    bool orientationRecorded = false;
    while ((TimeNow() - startTime < halfTimeToCheck) && !orientationRecorded) {
        if ((orientation >= 0) && (orientation <= 180)) {
            RPS_correct_heading(90, halfTimeToCheck);
            direction = NORTH;
            orientationRecorded = true;
        } else if ((orientation > 180) && (orientation < 360)) {
            RPS_correct_heading(270, halfTimeToCheck);
            direction = SOUTH;
            orientationRecorded = true;
        }
    }

    if (orientationRecorded) {
        // Determine the direction of the motors based on the direction the robot is facing
        int power = RPS_TRANSLATIONAL_PULSE_PERCENT;
        if(direction == SOUTH){
            power = -RPS_TRANSLATIONAL_PULSE_PERCENT;
        }

        startTime = TimeNow();

        // Check if receiving proper RPS coordinates and whether the robot is within an acceptable range
        while(((RPS.Y() > 0) && (abs(RPS.Y() - y_coord) > RPS_TRANSLATIONAL_THRESHOLD)) && (TimeNow() - startTime < halfTimeToCheck)) 
        {
            if(RPS.Y() > y_coord)
            {
                // Pulse the motors for a short duration in the correct direction
                move_forward_seconds(-power, RPS_TRANSLATIONAL_PULSE_TIME);
            }
            else if(RPS.Y() < y_coord)
            {
                // Pulse the motors for a short duration in the correct direction
                move_forward_seconds(power, RPS_TRANSLATIONAL_PULSE_TIME);
            }

            if (TimeNow() - startTime > secondsToCheck) {
                break;
            }

            Sleep(RPS_DELAY_TIME);

            show_RPS_data();
        }
    }
    else {
        write_status("ERROR. RPS NOT READING.");
    }
}

/*******************************************************************/
// PID STUFF

/*******************************************************
 * @brief Resets all PID global variables to initial values.
 * 
 */
void ResetPIDVariables() {
    
    // Resets all variables to inital state
    PID_Linear_SpeedR = 0;
    PID_New_CountsR = 0;
    PID_Last_CountsR = 0;
    PID_New_TimeR = 0;
    PID_Last_TimeR = 0;
    PID_New_Speed_ErrorR = 0;
    PID_Last_Speed_ErrorR = 0;
    PID_OLD_MOTOR_POWERR = 0;
    PID_NEW_MOTOR_POWERR = 0;
    PID_Linear_SpeedR = 0;
    PTermR = 0;
    ITermR = 0;
    DTermR = 0;

    PID_Linear_SpeedL = 0;
    PID_New_CountsL = 0;
    PID_Last_CountsL = 0;
    PID_New_TimeL = 0;
    PID_Last_TimeL = 0;
    PID_New_Speed_ErrorL = 0;
    PID_Last_Speed_ErrorL = 0;
    PID_OLD_MOTOR_POWERL = 0;
    PID_NEW_MOTOR_POWERL = 0;
    PID_Linear_SpeedL = 0;
    PTermL = 0;
    ITermL = 0;
    DTermL = 0;
    
    // Records initial time
    PID_TIME = TimeNow();

    // Resets encoders
    left_encoder.ResetCounts();
    right_encoder.ResetCounts();

    Sleep(0.15);
}

/*******************************************************
 * @brief Makes adjustments to right motor based on expected speed, counts, and current motor speed.
 * 
 * @param expectedSpeed Expected speed for motor in inches per second
 * @return float Correction value used to change motor in move functions
 */
float RightPIDAdjustment(double expectedSpeed) {

    // Finds change in counts since last time
    PID_Last_CountsR = PID_New_CountsR;
    PID_New_CountsR = right_encoder.Counts();
    
    // Finds change in time since last time
    PID_Last_TimeR = PID_New_TimeR;
    PID_New_TimeR = TimeNow();

    // Finds actual velocity
    PID_Linear_SpeedR = (PID_DISTANCE_PER_COUNT * ((PID_New_CountsR - PID_Last_CountsR) / (PID_New_TimeR - PID_Last_TimeR)));

    // Finds error
    PID_New_Speed_ErrorR = expectedSpeed - PID_Linear_SpeedR;

    // Adds error to error sum
    PID_Error_SumR += PID_New_Speed_ErrorR;

    // Calculates PTerm
    PTermR = PID_New_Speed_ErrorR * PConstR;

    // Calculates ITerm
    ITermR = PID_Error_SumR * IConstR;

    // Calculates DTerm
    DTermR = (PID_New_Speed_ErrorR - PID_Last_Speed_ErrorR) * DConstR;

    // Saves past error
    PID_Last_Speed_ErrorR = PID_New_Speed_ErrorR;

    return (PID_OLD_MOTOR_POWERR + PTermR + ITermR + DTermR);
}

/*******************************************************
 * @brief Makes adjustments to left motor based on expected speed, counts, and current motor speed.
 * 
 * @param expectedSpeed Expected speed for motor in inches per second
 * @return float Correction value used to change motor in move functions
 */
float LeftPIDAdjustment(double expectedSpeed) {
    
    // Finds change in counts since last time
    PID_Last_CountsL = PID_New_CountsL;
    PID_New_CountsL = left_encoder.Counts();
    
    // Finds change in time since last time
    PID_Last_TimeL = PID_New_TimeL;
    PID_New_TimeL = TimeNow();

    // Finds actual velocity
    PID_Linear_SpeedL = (PID_DISTANCE_PER_COUNT * ((PID_New_CountsL - PID_Last_CountsL) / (PID_New_TimeL - PID_Last_TimeL)));

    // Finds error
    PID_New_Speed_ErrorL = expectedSpeed - PID_Linear_SpeedL;

    // Adds error to error sum
    PID_Error_SumL += PID_New_Speed_ErrorL;

    // Calculates PTerm
    PTermL = PID_New_Speed_ErrorL * PConstL;

    // Calculates ITerm
    ITermL = PID_Error_SumL * IConstL;

    // Calculates DTerm
    DTermL = (PID_New_Speed_ErrorL - PID_Last_Speed_ErrorL) * DConstL;

    // Saves past error
    PID_Last_Speed_ErrorL = PID_New_Speed_ErrorL;

    return (PID_OLD_MOTOR_POWERL + PTermL + ITermL + DTermL);
}

/*******************************************************
 * @brief Uses PID to move forward a number of inches at a specific speed in inches per second.
 * 
 * @param in_per_sec Speed to move forward at in inches per second
 * @param inches Inches to move forward
 */
void move_forward_PID(float in_per_sec, float inches) {

    ResetPIDVariables();

    // Moves forward until average counts are above inches
    while ((((left_encoder.Counts() + right_encoder.Counts()) / 2) * PID_DISTANCE_PER_COUNT) < inches) {
        
        // Calculates corrections to make
        PID_NEW_MOTOR_POWERR = RightPIDAdjustment(in_per_sec);
        PID_NEW_MOTOR_POWERL = LeftPIDAdjustment(in_per_sec);
        
        // Applies corrections
        right_motor.SetPercent(PID_NEW_MOTOR_POWERR);
        left_motor.SetPercent(PID_NEW_MOTOR_POWERL);

        // Records old motor values for future corrections
        PID_OLD_MOTOR_POWERR = PID_NEW_MOTOR_POWERR;
        PID_OLD_MOTOR_POWERL = PID_NEW_MOTOR_POWERL;

        Sleep(SLEEP_PID);
    }

    right_motor.Stop();
    left_motor.Stop();
    
}

/*******************************************************
 * @brief Initiates both servos, sets min/max values and 
 * turns it to starting rotation.
 */
void initiate_servos() {
    
    // Calibrates base servo
    base_servo.SetMin(BASE_SERVO_MIN);
    base_servo.SetMax(BASE_SERVO_MAX);

    // Calibrate on-arm servo
    on_arm_servo.SetMin(ON_ARM_SERVO_MIN);
    on_arm_servo.SetMax(ON_ARM_SERVO_MAX);

    // Sets base servo to initial degree
    base_servo.SetDegree(85.);
    on_arm_servo.SetDegree(8.);
}

/*******************************************************
 * @brief Detects the color using the CdS cell
 *
 * @param timeToDetect time the robot takes to detect if it doesn't see the color right away
 * 
 * @return int color Color detected.
 *          0 -> Red
 *          1 -> Blue
 */
int detect_color(int timeToDetect) {
    LCD.Clear();

    // Color to be returned
    int color = -1;

    // Initiates variables to find average value
    double startTime = TimeNow();
    double sum = 0.0;
    int numValues = 0;
    double averageValue = sum / numValues; // Average value of CdS cell values

    // 0 if color hasn't been detected yet, 1 if otherwise
    int colorFound = 0;

    // Reads values for four seconds OR until color is found
    while (( (TimeNow() - startTime) < timeToDetect) && !colorFound) {
        
        // Takes the average value read
        sum += CdS_cell.Value();
        numValues++;
        averageValue = sum / numValues;

        // Detects color using CdS_cell values
        if (CdS_cell.Value() < 0.345) {
            color = 0;
            colorFound = 1;
        } else if (CdS_cell.Value() > 0.345) {
            color = 1;
            colorFound = 1;
        } else {
            color = 0; // Default is red
        }

        // Prints out info
        LCD.SetBackgroundColor(BACKGROUND_COLOR);
        LCD.SetFontColor(FONT_COLOR);
        LCD.Clear();

        LCD.WriteRC("Reading color: ", 1, 3);
        LCD.WriteRC(TimeNow() - startTime, 1, 18); // Time elapsed

        LCD.WriteRC("CdS Value: ", 3, 4);
        LCD.WriteRC(CdS_cell.Value(), 3, 15); // CdS cell value

        LCD.WriteRC("Color: ", 5, 7);

        // Prints which color is recognized
        switch (color)
        {
        case 0:
            LCD.SetFontColor(RED);
            LCD.WriteRC("Red", 5, 15);
            break;

        case 1: 
            LCD.SetFontColor(BLUE);
            LCD.WriteRC("Blue", 5, 15);
            break;
        
        default:
            LCD.WriteRC("Other", 5, 15);
            break;
        }

        // Prints out the average value
        LCD.SetFontColor(FONT_COLOR);
        LCD.WriteRC("Average Value: ", 7, 6);
        LCD.WriteRC(averageValue, 7, 15);
        Sleep(0.1);
    }

    return color;
}

/*******************************************************
 * @brief Presses jukebox buttons based on color.
 */
void press_jukebox_buttons() {
    
    /*
     * Detects the color of the jukebox
     * 
     * 0 -> Red      
     * 1 -> Blue
     */ 
    int color = detect_color(4);
    Sleep(0.5);
    move_forward_inches(-FORWARD_SPEED, 2); // Makes room for arm

    // Space for turn is the amount of space to move forward after aligning with buttons
    float spaceForTurn = 2; // Initially 2.75

    // Time to move forward to press buttons
    float secondsFromButtons = 0.9;

    // Responds to the jukebox light appropriately
    if (color == 0) { // On right path (red light)

        // Moves on_arm_servo out of the way
        on_arm_servo.SetDegree(8);

        // Goes down red button path
        turn_right_degrees(TURN_SPEED, 35);
        
        move_forward_inches(FORWARD_SPEED, spaceForTurn);
        turn_left_degrees(TURN_SPEED, 35);

        // Moves base servo down to press
        base_servo.SetDegree(4);
        Sleep(0.5);

        RPS_correct_heading(RPS_270_Degrees, 4);

        move_forward_seconds(20, secondsFromButtons + 0.75); // Moves forward until buttons

        move_forward_seconds(-20, secondsFromButtons); // Reverses from buttons

        // Moves up base servo 
        base_servo.SetDegree(85);

        // Returns to CdS cell over light
        turn_right_degrees(TURN_SPEED, 35);

        move_forward_inches(-FORWARD_SPEED, spaceForTurn);
        turn_left_degrees(TURN_SPEED, 35);
        

    } else if (color == 1) { // On left path (blue light)

        // Moves on_arm_servo out of the way
        on_arm_servo.SetDegree(0);

        // Goes down blue button path
        turn_left_degrees(TURN_SPEED, 35);

        move_forward_inches(FORWARD_SPEED, spaceForTurn);
        turn_right_degrees(TURN_SPEED, 35);

        // Moves base servo down to press
        base_servo.SetDegree(4);

        RPS_correct_heading(RPS_270_Degrees, 2);

        move_forward_seconds(20, secondsFromButtons + 0.75); // Moves forward until buttons

        move_forward_seconds(-20, secondsFromButtons); // Reverses from buttons

        // Moves up base servo 
        base_servo.SetDegree(85);
        on_arm_servo.SetDegree(180);

        // Returns to CdS cell over light
        turn_left_degrees(TURN_SPEED, 35);

        move_forward_inches(-FORWARD_SPEED, spaceForTurn);
        turn_right_degrees(TURN_SPEED, 35);

    } else {
        LCD.Write("ERROR: COLOR NOT READ SUCCESFULLY");
    }
}

/*******************************************************
 * @brief Algorithm for flipping the hot plate when robot 
 * is at y=55. Facing directly at it.
 * 
 */
void flip_burger() {

    write_status("Flipping hot plate");

    //***********
    // Initial flip

    // Sets initial arm positions
    base_servo.SetDegree(85);
    on_arm_servo.SetDegree(8);

    Sleep(0.5);

    // Lowers base servo and moves it under hot plate
    base_servo.SetDegree(0);
    Sleep(1.0);
    move_forward_inches(FORWARD_SPEED, 1.15); // Initially 1.35
    
    Sleep(0.5);

    // Raises arm and moves forward consecutively
    base_servo.SetDegree(20); // First lift
    Sleep(0.25);
    move_forward_inches(FORWARD_SPEED, 2);
    Sleep(1.0);

    base_servo.SetDegree(45); // Second lift
    move_forward_inches(FORWARD_SPEED, 1.25);

    turn_right_degrees(TURN_SPEED, 30); // Turns right to help flip burger
    Sleep(0.5);

    on_arm_servo.SetDegree(145); // Second arm finishes push

    Sleep(1.0);

    //***********
    // Return flip

    write_status("Flipping other side");

    // Resets position
    on_arm_servo.SetDegree(8.); // Resets on arm servo position
    turn_left_degrees(TURN_SPEED, 30); // Readjusts angle

    // Flips around to hit burger plate
    on_arm_servo.SetDegree(50);
    base_servo.SetDegree(55);
    move_forward_inches(-FORWARD_SPEED, 1); // Accounted for in last move forward call here
    turn_left_degrees(40, 360);
    on_arm_servo.SetDegree(180);

    Sleep(0.5);

    // Correct heading. y=60.5 in front of first flip
    RPS_correct_heading(RPS_90_Degrees, 2);
    
    // Moves up base servo
    base_servo.SetDegree(85);

    // Moves backwards to 56.45
    move_forward_inches(-FORWARD_SPEED, 2.05); // Initially 4.05
}

/*******************************************************
 * @brief Flips the correct ice cream lever. 
 * 
 * @pre RPS must be initialized.
 */
void flip_ice_cream_lever() {

    // Distance to move forward towards ice cream lever
    float distToLever = 5.5; // Initially 5.25

    // Distance between levers
    float distBtwLevers = 4;

    // Time to sleep after pressing levers
    float leverTimeSleep = 6.6;
    
    if (RPS.GetIceCream() == 0) { // VANILLA

        // Moves on_arm_servo up to avoid interference from sides
        on_arm_servo.SetDegree(90);

        write_status("Navigating to vanilla lever ");
        turn_left_degrees(TURN_SPEED, 90);
        move_forward_inches(FORWARD_SPEED, distBtwLevers);
        turn_right_degrees(TURN_SPEED, 90);

        write_status("Pushing lever down");
        base_servo.SetDegree(85);
        move_forward_inches(FORWARD_SPEED, distToLever);
        base_servo.SetDegree(40);
        Sleep(leverTimeSleep);

        // Reverses from lever
        move_forward_inches(-FORWARD_SPEED, distToLever);

        write_status("Pushing lever up");

        // Makes sure on arm servo is out of the way
        on_arm_servo.SetDegree(180);

        base_servo.SetDegree(0); 
        move_forward_inches(FORWARD_SPEED, distToLever);
        base_servo.SetDegree(50);
        move_forward_inches(-FORWARD_SPEED, distToLever);

        turn_left_degrees(TURN_SPEED, 45);
        move_forward_inches(FORWARD_SPEED, 1);
        turn_left_degrees(TURN_SPEED, 45);
        move_forward_inches(-FORWARD_SPEED, distBtwLevers);
        turn_right_degrees(TURN_SPEED, 90);
        

    } else if (RPS.GetIceCream() == 1) { // TWIST

        // Moves on_arm_servo up to avoid interference from sides
        on_arm_servo.SetDegree(90);

        write_status("Pushing lever down");
        base_servo.SetDegree(85);
        move_forward_inches(FORWARD_SPEED, distToLever);
        base_servo.SetDegree(40);
        Sleep(leverTimeSleep);

        // Reverses from lever and gets arms out of the way
        base_servo.SetDegree(85);
        on_arm_servo.SetDegree(180);
        move_forward_inches(-FORWARD_SPEED, distToLever);

        write_status("Pushing lever up");
        base_servo.SetDegree(0); 
        move_forward_inches(FORWARD_SPEED, distToLever);
        base_servo.SetDegree(50);
        move_forward_inches(-FORWARD_SPEED, distToLever);

        // Moves to align with ramp
        turn_left_degrees(TURN_SPEED, 45);
        move_forward_inches(FORWARD_SPEED, 1);
        turn_right_degrees(TURN_SPEED, 45);

    } else if (RPS.GetIceCream() == 2) { // CHOCOLATE

        // Moves on_arm_servo up to avoid interference from sides
        on_arm_servo.SetDegree(90);

        write_status("Navigating to chocolate lever ");
        turn_left_degrees(TURN_SPEED, 90);
        move_forward_inches(-FORWARD_SPEED, distBtwLevers);
        turn_right_degrees(TURN_SPEED, 90);

        write_status("Pushing lever down");
        base_servo.SetDegree(85);
        move_forward_inches(FORWARD_SPEED, distToLever);
        base_servo.SetDegree(40);
        Sleep(leverTimeSleep);

        // Reverses from lever
        move_forward_inches(-20, distToLever);

        write_status("Pushing lever up");

        // Makes sure on arm servo is out of the way
        on_arm_servo.SetDegree(180);

        base_servo.SetDegree(0); 
        move_forward_inches(FORWARD_SPEED, distToLever);
        base_servo.SetDegree(50);
        move_forward_inches(-FORWARD_SPEED, distToLever);

        turn_left_degrees(TURN_SPEED, 45);
        move_forward_inches(FORWARD_SPEED, 1);
        turn_left_degrees(TURN_SPEED, 45);
        move_forward_inches(FORWARD_SPEED, distBtwLevers);
        turn_right_degrees(TURN_SPEED, 90);

    } else {
        write_status("ERROR. ICE CREAM LEVER NOT SPECIFIED.");
    }

    base_servo.SetDegree(85);

}

/*******************************************************
 * @brief Clears room for status and prints it to screen 
 * without clearing the screen
 * 
 * @param status Status to be printed
 */
void write_status(const char status[]) {

    // Clears space and writes status to ccreen
    LCD.SetFontColor(BACKGROUND_COLOR);
    LCD.FillRectangle(0, 17, 319, 17);
    LCD.SetFontColor(FONT_COLOR);
    LCD.WriteRC(status, 1, 2);
}

/*******************************************************
 * @brief Shows the current RPS data.
 * 
 * @pre RPS must be initialized.
 */
void show_RPS_data() {

    // Clears space for movement data and status
    LCD.SetFontColor(BACKGROUND_COLOR);
    LCD.FillRectangle(0,100,319,239);
    LCD.SetFontColor(FONT_COLOR);
    LCD.DrawHorizontalLine(100, 0, 319);

    // Writes the data from the RPS
    write_status("Reading RPS Data");

    LCD.WriteRC("Heading: ", 7, 1);
    LCD.WriteRC(RPS.Heading(), 7, 10);

    LCD.WriteRC("X Value: ", 8, 1);
    LCD.WriteRC(RPS.X(), 8, 10);

    LCD.WriteRC("Y Value: ", 9, 1);
    LCD.WriteRC(RPS.Y(), 9, 10);

    LCD.WriteRC("Time: ", 10, 1);
    LCD.WriteRC(RPS.Time(), 10, 10);

    LCD.WriteRC("Course: ", 11, 1);
    LCD.WriteRC(RPS.CurrentRegionLetter(), 11, 10);
}

/*******************************************************
 * @brief Runs the specified course.
 * 
 * @param courseNumber Course number to runs
 */
void run_course(int courseNumber) {

    // Used for timeouts for some functions
    float startTime = TimeNow();

    /*
     * NOTE: Status messages from movement functions only clear the 
     * portion of the screen that they use. They only do this beforehand.
     * 
     * write_status() is used to print what the robot is doing without 
     * clearing the movement status (turn left/right etc..)
     */

    // Creates room for status messages
    LCD.Clear();
    
    switch (courseNumber)
    {
    case TEST_COURSE_1: // Test course 1

        // Moves left/right upon touch
        write_status("Running Test 1");

        int xGarb, yGarb;

        Sleep(1.0);
        while(true) {
            write_status("Press to turn left.");
            while(!LCD.Touch(&xGarb, &yGarb));
            turn_left_degrees(TURN_SPEED, 90);
            while(!LCD.Touch(&xGarb, &yGarb));
            turn_right_degrees(TURN_SPEED, 90);
        }
        write_status("Complete.");
        break;

    case TEST_COURSE_2: // Test course 1

        // Moves forward indefinitely
        write_status("Running Test 2");

        int xTrash2, yTrash2;

        Sleep(1.0);
        write_status("Press to move forward");
        
        while(!LCD.Touch(&xTrash2, &yTrash2));
        while(true) {
            move_forward_inches(FORWARD_SPEED, 9999);
            while(!LCD.Touch(&xTrash2, &yTrash2));
        }

        break;

    case TEST_COURSE_3: // Test course 1

        // Moves base/on_arm servos upon touch
        write_status("Running Test 3");

        float degreesToTurnBase, degreesToTurnArm;
        int xTrash, yTrash;
        degreesToTurnBase = 90.0;
        degreesToTurnArm = 90.0;

        base_servo.SetDegree(degreesToTurnBase);
        on_arm_servo.SetDegree(degreesToTurnArm);

        LCD.DrawHorizontalLine(40, 0, 319);
        LCD.DrawHorizontalLine(140, 0, 319);
        LCD.DrawVerticalLine(160, 0, 239);
        
        while (true) {

            while(!LCD.Touch(&xTrash, &yTrash));

                LCD.SetFontColor(BACKGROUND_COLOR);
                LCD.FillRectangle(0, 0, 159, 39);
                LCD.SetFontColor(FONT_COLOR);

                // Base Servo
                if (xTrash < 160) {
                    if (yTrash < 120) {
                        if ((degreesToTurnBase + 2.5) >= 0) {
                            degreesToTurnBase += 2.5;
                            base_servo.SetDegree(degreesToTurnBase);
                        }
                    } else {
                        if ((degreesToTurnBase - 2.5) <= 180) {
                            degreesToTurnBase - 2.5;
                            base_servo.SetDegree(degreesToTurnBase);
                        }
                    }
                    
                    LCD.WriteRC("Base:", 1, 0);
                    LCD.WriteRC(degreesToTurnBase, 1, 6);

                } else { // Arm servo
                    if (yTrash < 120) {
                        if ((degreesToTurnArm + 2.5) >= 0) {
                            degreesToTurnArm += 2.5;
                            on_arm_servo.SetDegree(degreesToTurnArm);
                        }
                    } else {
                        if ((degreesToTurnArm - 2.5) <= 180) {
                            degreesToTurnArm - 2.5;
                            on_arm_servo.SetDegree(degreesToTurnArm);
                        }
                    }

                    LCD.WriteRC("Arm:", 1, 14);
                    LCD.WriteRC(degreesToTurnArm, 1, 19);
                }    
        }

        break;

    case CALIBRATE_SERVOS:
        write_status("Calibrating Servos");
        Sleep(1.0);
        write_status("L -> base | R -> arm");

        int xTrash420, yTrash69;

        // Waits for touch. If touch is on left then base_servo is calibrated, and vice versa
        while(!LCD.Touch(&xTrash420, &yTrash69));

        LCD.DrawVerticalLine(160, 20, 239);

        if (xTrash420 < 160) {
            base_servo.TouchCalibrate();
        } else if (xTrash420 > 160) {
            on_arm_servo.TouchCalibrate();
        }

        break;

    case PERF_COURSE_1: // Performance Test 1

        write_status("Running Perf. Test 1");

        Sleep(1.0);
        
        /***************************************************/

        write_status("Moving towards jukebox");

        // Heads from button to center
        move_forward_inches(20, 8.0 + DIST_AXIS_CDS); // Direct: 7.5 inches 
        Sleep(1.0);

        // Moves towards jukebox
        turn_left_degrees(20, 43);
        Sleep(1.0);

        move_forward_inches(20, 12);
        Sleep(1.0);

        turn_left_degrees(20, 89);
        Sleep(1.0);

        //Reverses to move CdS cell over jukebox light
        move_forward_inches(-20, 0.75 + DIST_AXIS_CDS);
    
       /***************************************************/

        write_status("Pressing jukebox buttons");
        
        // Presses jukebox buttons
        press_jukebox_buttons();
        
        /***************************************************/

        // Moves forward to move wheel axis over jukebox light
        move_forward_inches(20, DIST_AXIS_CDS);

        write_status("Moving towards ramp");

        // Moves to center (aligns with ramp)
        turn_left_degrees(20, 85);
        Sleep(1.0);
        move_forward_inches(20, 9);
        Sleep(1.0);
        turn_left_degrees(20, 90);
        Sleep(1.0);
        
        write_status("Moving up ramp");

        // Moves up ramp
        move_forward_inches(35, 35); // 11 + 10 + 14
        Sleep(1.0);

        write_status("Moving down ramp");
        
        // Moves down ramp
        move_forward_inches(-35, 35);
        Sleep(1.0);

        write_status("Towards final button");

        // Heads toward final button
        turn_right_degrees(20, 90);
        Sleep(1.0);
        move_forward_inches(20, 2.9);
        Sleep(1.0);
        turn_right_degrees(20, 45);
        Sleep(1.0);
        move_forward_inches(20, 7.5);
        Sleep(1.0);

        write_status("Woo?");
        
        break;
        

    case PERF_COURSE_2: // Performance Test 2

        write_status("Running Performance Test 2");

        Sleep(1.0);

        write_status("Aligning with ramp");
        move_forward_inches(20, 11.55 + DIST_AXIS_CDS);
        turn_right_degrees(20, 45);

        write_status("Moving up ramp");
        move_forward_inches(40, 31.75 + DIST_AXIS_CDS);

        write_status("Moving towards sink");
        turn_right_degrees(20, 90);
        move_forward_inches(-20, 10.5); // Reverses
        turn_left_degrees(20, 90);
        move_forward_inches(-20, 8);
    
        write_status("Dropping tray");

        base_servo.SetDegree(85.);
        Sleep(1.0);
        base_servo.SetDegree(105.);
        Sleep(2.5);
        base_servo.SetDegree(85.);

        write_status("Moving away from sink");
        move_forward_inches(20, 8);
        turn_right_degrees(20, 90);
        move_forward_inches(20, 10.5);
        turn_left_degrees(20, 185);

        write_status("Moving towards ticket");
        move_forward_inches(-20, 13.15);
        turn_left_degrees(20, 90);

        write_status("Sliding ticket");
        on_arm_servo.SetDegree(45);
        Sleep(1.0);
        base_servo.SetDegree(0);

        move_forward_inches(20, 5.7);

        on_arm_servo.SetDegree(180);

        Sleep(1.0);

        move_forward_inches(-20, 23);

    
        break;

    case PERF_COURSE_3: // Performance Test 3

        write_status("Running Performance Test 3");
        Sleep(1.0);

        write_status("Aligning with ramp");
        move_forward_inches(20, 11.55 + DIST_AXIS_CDS);
        turn_right_degrees(20, 45);
        RPS_correct_heading(90, 3);

        write_status("Moving up ramp");
        move_forward_inches(40, 33.26 + DIST_AXIS_CDS); // Initially 35.26
        RPS_check_y(55, 3); // On top of ramp y-coord
    
        write_status("Moving towards hot plate");
        turn_right_degrees(20, 90);
        RPS_correct_heading(0, 3);
        RPS_check_x(18.6, 3); // On top of ramp x-coord
    
        // PROBLEM AREA. MOVES PRECISELY IN FRONT OF BURGER PLATE
        move_forward_inches(20, 8); // Initially 5.5
        RPS_check_x(27.8, 3); // In front of burger plate x
        turn_left_degrees(20, 90);
        RPS_correct_heading(90, 3);
        RPS_check_y(55, 3);

        Sleep(2.0);

        // Flips burger when robot is ~13 inches in front, facing towards it
        flip_burger();

        write_status("Moving towards ice cream lever");
        RPS_correct_heading(90, 3);
        RPS_check_y(55, 3);
        turn_left_degrees(20, 90);
        RPS_correct_heading(180, 3);
        RPS_check_x(29.1, 3);
        move_forward_inches(20, 3); // Moves forward a bit to get in better RPS range
        RPS_correct_heading(180, 3);
        move_forward_inches(20, 3.5 + DIST_AXIS_CDS); // Initially 12.9
        RPS_correct_heading(180, 3);
        turn_right_degrees(20, 45);
        RPS_correct_heading(135, 3);

        // Flips ice cream lever, about 3 inches in front of it (including base servo arm)
        flip_ice_cream_lever();
    
        break;

    case PERF_COURSE_4: // Performance Test 4
        
        LCD.Write("Running Performance Test 4");
        // Center of top coords
        // 18.1 52.5 (Heading 90)
        // 15.4 49.7 (Heading left)
        Sleep(1.0);

        write_status("Aligning with ramp");
        move_forward_inches(FORWARD_SPEED, 11.75 + DIST_AXIS_CDS); // Initially 11.55, then 12.05
        turn_right_degrees(TURN_SPEED, 45);

        write_status("Moving up ramp");
        // Subtracts three to avoid dead zone
        move_forward_inches(40, 30.26 + DIST_AXIS_CDS); // Initially 35.26
        RPS_check_y(52.25, 3); // On top of ramp y-coord, initially 55

        turn_left_degrees(TURN_SPEED, 90);
        RPS_check_x(15.45, 3); // Initially 15.1

        turn_right_degrees(TURN_SPEED, 90);
        move_forward_inches(FORWARD_SPEED, 4.20); // Initially 3.25
        turn_left_degrees(TURN_SPEED, 45);

        //Flips ice cream lever, about 3 inches in front of it (including base servo arm)
        flip_ice_cream_lever();

        write_status("Moving towards final button");
        turn_right_degrees(TURN_SPEED, 45);
        move_forward_inches(-20, 3);
        RPS_correct_heading(90, 3);
        move_forward_inches(-20, 31.46 + DIST_AXIS_CDS); // Initially 35.26
        turn_left_degrees(TURN_SPEED, 45);
        move_forward_inches(-FORWARD_SPEED, 20);

        break;

    case IND_COMP: // Individual Competition
        write_status("Running Individual Competition");

        /*********************************************************************/
        // Jukebox

            //************
            write_status("Moving towards jukebox");

            // Heads from button to center
            move_forward_inches(FORWARD_SPEED, 9 + DIST_AXIS_CDS); // Direct: 7.5 inches 

            // Moves towards jukebox
            turn_left_degrees(TURN_SPEED, 45);

            // Moves on_arm_servo out of the way
            on_arm_servo.SetDegree(90); 

            // Over CdS cell
            move_forward_inches(FORWARD_SPEED, 11.5 - 1.0607);
            RPS_check_x(RPS_Top_Level_X_Reference - 7.8, 2); // Initially 8.8 left of top x reference

            // Face jukebox
            turn_left_degrees(TURN_SPEED, 90);

            //Reverses to move CdS cell over jukebox light and make room for arm
            move_forward_inches(-FORWARD_SPEED, DIST_AXIS_CDS + 0.25 - 1.0607); // 0.5 wasn't initially there
            RPS_check_y(RPS_Top_Level_Y_Reference - 34, 2); // 35 below top y reference 18.3. Initially 33.7

            //************
            write_status("Pressing jukebox buttons");
            
            // Presses jukebox buttons, returning to CdS cell over jukebox light
            press_jukebox_buttons();

            // Sets on_arm_servo into initial position
            on_arm_servo.SetDegree(180);

            // Moves back forward to axis over jukebox light
            move_forward_inches(FORWARD_SPEED, DIST_AXIS_CDS);
            
            //************

            write_status("Moving towards ramp");

            // Moves to center (aligns with ramp)
            turn_left_degrees(TURN_SPEED, 90);
            move_forward_inches(FORWARD_SPEED, 9.25); // Initially 9
            turn_left_degrees(TURN_SPEED, 90);

        /*********************************************************************/
        // Ramp

            // Moves up ramp 9 inches from jukebox light
            // OR 11.75 + DIST_AXIS_CDS from starting light
            write_status("Moving up ramp");

            // Checks that it is positioned straight
            RPS_correct_heading(RPS_90_Degrees, 2);

            // Subtracts three to avoid dead zone
            // Gets to that place on top of the ramp (52.25, 15.45)
            move_forward_inches(RAMP_SPEED, 30.26 + DIST_AXIS_CDS); // Initially 30.26 + DIST_AXIS_CDS. Took off because no longer moves forward after jukebox

            // Checks x and y coordinate after going up
            // RPS_check_y(RPS_Top_Level_Y_Reference, 2); 

            // Checks x (may need to edit)
            //turn_left_degrees(TURN_SPEED, 90); 
            turn_right_degrees(TURN_SPEED, 90); // Initially 180 degrees to correct for RPS check
            RPS_check_x(RPS_Top_Level_X_Reference + 4.65, 2);
            


        /*********************************************************************/
        // Sink

            // Reverses towards sink
            move_forward_inches(-FORWARD_SPEED, 9.25); 

            // Aligns and backs up to edge of sink (~8 inches away)
            turn_left_degrees(TURN_SPEED, 90);
            move_forward_seconds(-40, 1);
            
            write_status("Dropping tray");

            // Moves servos to drop tray
            base_servo.SetDegree(85.);
            base_servo.SetDegree(105.);
            Sleep(0.5); // Lets tray fall
            base_servo.SetDegree(85.);

            write_status("Moving away from sink");

            // Drives away from sink
            move_forward_inches(FORWARD_SPEED, 7.75);

            // Moves towards that one spot on top (facing rightwards)
            turn_right_degrees(TURN_SPEED, 90);
            //RPS_correct_heading(RPS_0_Degrees, 2); // IN DEADZONE
            move_forward_inches(FORWARD_SPEED, 9.25);


        /*********************************************************************/
        // Ticket

            // From that one spot on top (facing right)
            write_status("Moving towards ticket");

            // Turns to face left (to be able to reverse towards ticket)
            turn_left_degrees(30, 180);
            //RPS_check_x(RPS_Top_Level_X_Reference, 2); // 15.45

            // Reverses towards ticket
            move_forward_inches(-FORWARD_SPEED, 13.25); // Initially 13.65
            RPS_check_x(RPS_Top_Level_X_Reference + 13.25, 2);

            // Facing ticket
            turn_left_degrees(TURN_SPEED, 90);

            // Slides ticket from y=52.25
            write_status("Sliding ticket");
            on_arm_servo.SetDegree(43); // Initially 45
            base_servo.SetDegree(0);
            RPS_check_y(RPS_Top_Level_Y_Reference - 4.65, 2); // 52.25 - 4.65

            move_forward_inches(20, 4.75); // Inserts arm into ticket slot, initially 0.25

            // Reverses away from ticket
            on_arm_servo.SetDegree(180);
            move_forward_inches(-20, 4.75);

        /*********************************************************************/
        // Hot Plate

            // From in front of ticket
            write_status("Moving towards hot plate");
            
            // Resets arm positions
            on_arm_servo.SetDegree(8);
            base_servo.SetDegree(85);

            // Moves towards the front
            turn_right_degrees(TURN_SPEED, 90);
            move_forward_inches(FORWARD_SPEED, 6); // Initially 5.85
            RPS_check_x(RPS_Top_Level_X_Reference + 7.65, 4); // Initially 7.8
            turn_right_degrees(TURN_SPEED, 90);

            // Currently at y=52.25, needs to be at y=55
            move_forward_inches(FORWARD_SPEED, 2.75); // 2.75 initially
            RPS_check_y(RPS_Top_Level_Y_Reference + 2.75, 4);

            /* 
                * Flips burger when y=55 and facing towards it
                * Finishes at y=56.45 in front of first plate
                */
            flip_burger();

            RPS_check_y(RPS_Top_Level_Y_Reference + 4, 2); // Initially 55.95, initially plus 3.7

            turn_left_degrees(TURN_SPEED, 90);

            // In front of initial plate, 4.05 inches from front, heading=0
            RPS_check_x(RPS_Top_Level_X_Reference + 7.4, 2); // Initially 21.7
            

        /*********************************************************************/
        // Ice cream lever

            // From after flip_burger() (at y=55 in front of reverse plate (5.8 inches right from front))
            // Needs to be at y=56.45 and x=15.45 (LEFT) (Can't check x though at y=56.45 since DEAD ZONE)
            write_status("Moving towards ice cream");
            
            move_forward_inches(20, 5); // Moves to x=15.45, initially 5.75
            //RPS_check_x(15.45); // IN DEADZONE

            // Faces towards levers
            turn_right_degrees(TURN_SPEED, 45);

            /*
            * Flips correct ice cream lever when y=56.45 (VERTICALLY) and x=15.45 (LEFT)
            * Must be facing towards ice cream levers.
            * Finishes where it started.
            */
            flip_ice_cream_lever();

        /*********************************************************************/
        // Final button

            // From after flip_ice_cream_lever()
            // y=56.45, x=15.45 FACING LEVERS
            write_status("Moving towards final button");

            // Turns to reverse down ramp
            turn_right_degrees(TURN_SPEED, 45);

            // Reverses back out of dead zone to check heading
            move_forward_inches(-FORWARD_SPEED, 4.20);
            RPS_correct_heading(RPS_90_Degrees, 4);

            // Moves down ramp
            move_forward_inches(-FORWARD_SPEED, 30.26);

            // Heads towards final button
            turn_left_degrees(TURN_SPEED, 45);
            move_forward_inches(-FORWARD_SPEED, 20);


        break;

    case FINAL_COMP: // Final Competition

        write_status("Running Final Competition");

        /*********************************************************************/
        // Jukebox

            //************
            write_status("Moving towards jukebox");

            // Heads from button to center
            move_forward_inches(FORWARD_SPEED, 9 + DIST_AXIS_CDS); // Direct: 7.5 inches 

            // Moves towards jukebox
            turn_left_degrees(TURN_SPEED, 45);

            // Moves on_arm_servo out of the way
            on_arm_servo.SetDegree(90); 

            // Over CdS cell
            move_forward_inches(FORWARD_SPEED, 11.5 - 1.0607);
            RPS_check_x(RPS_Top_Level_X_Reference - 8.2, 1); // Initially 8.8 left of top x reference

            // Face jukebox
            turn_left_degrees(TURN_SPEED, 90);

            //Reverses to move CdS cell over jukebox light and make room for arm
            move_forward_inches(-FORWARD_SPEED, DIST_AXIS_CDS + 0.25 - 1.0607); // 0.5 wasn't initially there
            RPS_check_y(RPS_Top_Level_Y_Reference - 33.75, 2); // 35 below top y reference 18.3. Initially 33.7

            //************
            write_status("Pressing jukebox buttons");
            
            // Presses jukebox buttons, returning to CdS cell over jukebox light
            press_jukebox_buttons();

            // Sets on_arm_servo into initial position
            on_arm_servo.SetDegree(180);

            // Moves back forward to axis over jukebox light
            move_forward_inches(FORWARD_SPEED, DIST_AXIS_CDS);
            
            //************

            write_status("Moving towards ramp");

            // Moves to center (aligns with ramp)
            turn_left_degrees(TURN_SPEED, 90);
            move_forward_inches(FORWARD_SPEED, 9.25); // Initially 9
            turn_left_degrees(TURN_SPEED, 90);

        /*********************************************************************/
        // Ramp

            // Moves up ramp 9 inches from jukebox light
            // OR 11.75 + DIST_AXIS_CDS from starting light
            write_status("Moving up ramp");

            // Checks that it is positioned straight
            RPS_correct_heading(RPS_90_Degrees, 2);

            // Subtracts three to avoid dead zone
            // Gets to that place on top of the ramp (52.25, 15.45)
            // move_forward_inches(RAMP_SPEED, 30.26 + DIST_AXIS_CDS); // Initially 30.26 + DIST_AXIS_CDS. Took off because no longer moves forward after jukebox
            move_forward_PID(5, 30.26 + DIST_AXIS_CDS);

            // Checks x and y coordinate after going up
            // RPS_check_y(RPS_Top_Level_Y_Reference, 2); 

            // Checks x (may need to edit)
            //turn_left_degrees(TURN_SPEED, 90); 
            turn_right_degrees(TURN_SPEED, 90); // Initially 180 degrees to correct for RPS check
            RPS_check_x(RPS_Top_Level_X_Reference + 4.65, 8);
            


        /*********************************************************************/
        // Sink

            // Reverses towards sink
            move_forward_inches(-FORWARD_SPEED, 8.5); 

            // Aligns and backs up to edge of sink (~8 inches away)
            turn_left_degrees(TURN_SPEED, 90);
            move_forward_seconds(-40, 1);
            
            write_status("Dropping tray");

            // Moves servos to drop tray
            base_servo.SetDegree(85.);
            base_servo.SetDegree(105.);
            Sleep(0.5); // Lets tray fall
            base_servo.SetDegree(85.);

            write_status("Moving away from sink");

            // Drives away from sink
            move_forward_inches(FORWARD_SPEED, 7.75);

            // Moves towards that one spot on top (facing rightwards)
            turn_right_degrees(TURN_SPEED, 90);
            //RPS_correct_heading(RPS_0_Degrees, 2); // IN DEADZONE
            move_forward_inches(FORWARD_SPEED, 8.5);


        /*********************************************************************/
        // Ticket

            // From that one spot on top (facing right)
            write_status("Moving towards ticket");

            // Turns to face left (to be able to reverse towards ticket)
            turn_left_degrees(30, 180);
            //RPS_check_x(RPS_Top_Level_X_Reference, 2); // 15.45

            // Reverses towards ticket
            move_forward_inches(-FORWARD_SPEED, 13); // Initially 13.65
            RPS_check_x(RPS_Top_Level_X_Reference + 13, 6);

            // Facing ticket
            turn_left_degrees(TURN_SPEED, 90);

            // Slides ticket from y=52.25
            write_status("Sliding ticket");
            on_arm_servo.SetDegree(45); // Initially 45
            base_servo.SetDegree(0);
            RPS_check_y(RPS_Top_Level_Y_Reference - 4.65, 8); // 52.25 - 4.65

            move_forward_inches(20, 5.25); // Inserts arm into ticket slot, initially 0.25

            // Reverses away from ticket
            on_arm_servo.SetDegree(180);
            Sleep(0.25);
            move_forward_inches(-20, 5.25);

        /*********************************************************************/
        // Hot Plate

            // From in front of ticket
            write_status("Moving towards hot plate");
            
            // Resets arm positions
            on_arm_servo.SetDegree(8);
            base_servo.SetDegree(85);

            // Moves towards the front
            turn_right_degrees(TURN_SPEED, 90);
            move_forward_inches(FORWARD_SPEED, 6.5); // Initially 6.25
            RPS_check_x(RPS_Top_Level_X_Reference + 7.05, 2); // Initially 7.15
            turn_right_degrees(TURN_SPEED, 90);

            // Currently at y=52.25, needs to be at y=55
            RPS_correct_heading(90, 3);
            move_forward_inches(FORWARD_SPEED, 2.75); // 2.75 initially
            RPS_check_y(RPS_Top_Level_Y_Reference + 2.75, 4);

            /* 
                * Flips burger when y=55 and facing towards it
                * Finishes at y=56.45 in front of first plate
                */
            flip_burger();

            RPS_check_y(RPS_Top_Level_Y_Reference + 4, 4); // Initially 55.95, initially plus 3.7

            turn_left_degrees(TURN_SPEED, 90);

            // In front of initial plate, 4.05 inches from front, heading=0
            RPS_check_x(RPS_Top_Level_X_Reference + 7.4, 4); // Initially 21.7
            

        /*********************************************************************/
        // Ice cream lever

            // From after flip_burger() (at y=55 in front of reverse plate (5.8 inches right from front))
            // Needs to be at y=56.45 and x=15.45 (LEFT) (Can't check x though at y=56.45 since DEAD ZONE)
            write_status("Moving towards ice cream");
            
            move_forward_inches(20, 4.5); // Moves to x=15.45, initially 5
            //RPS_check_x(15.45); // IN DEADZONE

            // Faces towards levers
            turn_right_degrees(TURN_SPEED, 45);

            /*
                * Flips correct ice cream lever when y=56.45 (VERTICALLY) and x=15.45 (LEFT)
                * Must be facing towards ice cream levers.
                * Finishes where it started.
                */
            flip_ice_cream_lever();

        /*********************************************************************/
        // Final button

            // From after flip_ice_cream_lever()
            // y=56.45, x=15.45 FACING LEVERS
            write_status("Moving towards final button");

            // Turns to reverse down ramp
            turn_right_degrees(TURN_SPEED, 45);

            // Reverses back out of dead zone to check heading
            move_forward_inches(-FORWARD_SPEED, 4.20);
            RPS_correct_heading(RPS_90_Degrees, 3);

            // Moves down ramp
            move_forward_inches(-RAMP_SPEED, 30.26);

            // Heads towards final button
            turn_left_degrees(TURN_SPEED, 45);
            move_forward_inches(-50, 30);


        break;
    
    default:
        LCD.WriteRC("ERROR: NO COURSE SPECIFIED", 1, 0);
        break;
    }
}

/*****************************************************************
 * main
 */
int main() {

    // Initiates servos 25.3 58.3
    initiate_servos();

    // Initializes RPS
    RPS.InitializeTouchMenu();

    // Clears the screen
    LCD.SetBackgroundColor(BACKGROUND_COLOR);
    LCD.SetFontColor(FONT_COLOR);
    LCD.Clear();

    // Gets RPS heading values to decrease inconsistencies from course to course
    // Clears screen to a yellow screen until touch is detected
    update_RPS_Heading_values(60, true, true, true);

    // Waits until start light is read
    read_start_light(45);

    // Runs specified course number.
    run_course(FINAL_COMP);

    return 0;
}