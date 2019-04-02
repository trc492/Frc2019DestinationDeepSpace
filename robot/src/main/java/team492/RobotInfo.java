/*
 * Copyright (c) 2018 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package team492;

import frclib.FrcPixyCam;

public class RobotInfo
{
    //
    // Compiler switches
    //

    //
    // Field dimensions in inches.
    //
    public static final double FIELD_LENGTH                     = 54*12.0;
    public static final double FIELD_WIDTH                      = 27*12.0;
    
    public static final double EXCHANGE_WIDTH                   = 21.0; // 1ft 9in
    public static final double SWITCH_FENCE_HEIGHT              = 18.75; // 1ft 6.75in
    public static final double SWITCH_FENCE_WIDTH               = 153.5; // 12ft 9.5in
    
    public static final double CUBE_HEIGHT                      = 11.0;
    public static final double CUBE_WIDTH                       = 13.0;
    public static final double CUBE_DEPTH                       = 13.0;

    public static final double BATTERY_CAPACITY_WATT_HOUR       = 18.0*12.0;

    //
    // Robot dimensions.
    //
    public static final double ROBOT_LENGTH                     = 35.0;
    public static final double ROBOT_WIDTH                      = 39.0;
    public static final double ROBOT_HEIGHT                     = 55.0;

    //
    // Joystick ports.
    //
    public static final int JSPORT_LEFT_DRIVESTICK              = 0;
    public static final int JSPORT_RIGHT_DRIVESTICK             = 1;
    public static final int JSPORT_OPERATORSTICK                = 2;

    //
    // CAN IDs.
    //
    public static final int CANID_LEFTFRONTWHEEL                = 3;    // 40A: Orange
    public static final int CANID_RIGHTFRONTWHEEL               = 4;    // 40A: Yellow
    public static final int CANID_LEFTREARWHEEL                 = 5;    // 40A: Green
    public static final int CANID_RIGHTREARWHEEL                = 6;    // 40A: Blue
    public static final int CANID_WINCH_MASTER                  = 7;    // 40A: Purple
    public static final int CANID_WINCH_SLAVE                   = 8;    // 40A: Gray
    public static final int CANID_ELEVATOR                      = 9;    // 30A: White
    public static final int CANID_LEFT_PICKUP                   = 10;   // 30A: Orange
    public static final int CANID_RESERVED                      = 11;   // 30A: Yellow
    public static final int CANID_RIGHT_PICKUP                  = 12;   // 30A: Green

    public static final int CANID_PDP                           = 16;
    public static final int CANID_PCM1                          = 17;
    public static final int CANID_PCM2                          = 18;

    //
    // PDP Channels.
    //
    public static final int PDP_CHANNEL_LEFT_FRONT_WHEEL        = 0;
    public static final int PDP_CHANNEL_LEFT_REAR_WHEEL         = 1;
    public static final int PDP_CHANNEL_WINCH_MASTER            = 2;
    public static final int PDP_CHANNEL_ELEVATOR                = 3;
    public static final int PDP_CHANNEL_LED                     = 9;
    public static final int PDP_CHANNEL_RIGHT_PICKUP            = 10;
    public static final int PDP_CHANNEL_LEFT_PICKUP             = 11;
    public static final int PDP_CHANNEL_WINCH_SLAVE             = 13;
    public static final int PDP_CHANNEL_RIGHT_REAR_WHEEL        = 14;
    public static final int PDP_CHANNEL_RIGHT_FRONT_WHEEL       = 15;

    //
    // Analog Input ports.
    //
    public static final int AIN_LEFT_SONAR_SENSOR               = 0;
    public static final int AIN_RIGHT_SONAR_SENSOR              = 1;
    public static final int AIN_FRONT_SONAR_SENSOR              = 2;
    public static final int AIN_PRESSURE_SENSOR                 = 3;

    //
    // Digital Input/Output ports.
    //
    public static final int DIO_LEFT_SONAR_PING                 = 0;
    public static final int DIO_RIGHT_SONAR_PING                = 1;
    public static final int DIO_FRONT_SONAR_PING                = 2;
    public static final int DIO_LEFT_PROXIMITY_SENSOR           = 7;
    public static final int DIO_RIGHT_PROXIMITY_SENSOR          = 8;
    public static final int DIO_CUBE_PROXIMITY_SENSOR           = 9;

    //
    // PWM Channels.
    //
    public static final int PWM_REV_BLINKIN                     = 0;

    //
    // Relay channels.
    //
    public static final int RELAY_RINGLIGHT_POWER               = 0;    // 20A: Purple

    //
    // Solenoid channels.
    //
    public static final int SOL_CUBEPICKUP_ARM_RETRACT          = 0;    // Brown
    public static final int SOL_CUBEPICKUP_ARM_EXTEND           = 1;    // Red
    public static final int SOL_CUBEPICKUP_CLAW_RETRACT         = 2;    // Orange
    public static final int SOL_CUBEPICKUP_CLAW_EXTEND          = 3;    // Yellow
    public static final int SOL_LEFT_FLIPPER_RETRACT            = 4;    // Green
    public static final int SOL_LEFT_FLIPPER_EXTEND             = 5;    // Blue
    public static final int SOL_RIGHT_FLIPPER_RETRACT           = 6;    // Purple
    public static final int SOL_RIGHT_FLIPPER_EXTEND            = 7;    // White

    //
    // Vision subsystem.
    //
    public static final int PIXYCAM_WIDTH                       = 320;
    public static final int PIXYCAM_HEIGHT                      = 200;
    public static final int PIXY_POWER_CUBE_SIGNATURE           = 1;
    public static final int PIXY_BRIGHTNESS                     = 80;
    public static final double PIXY_CAM_OFFSET                  = 8.0;
    public static final PixyVision.Orientation PIXY_ORIENTATION = PixyVision.Orientation.NORMAL_LANDSCAPE;
    public static final int PIXYCAM_I2C_ADDRESS                 = FrcPixyCam.DEF_I2C_ADDRESS;

    public static final int USBCAM_WIDTH                        = 320;
    public static final int USBCAM_HEIGHT                       = 240;
    public static final int USBCAM_FRAME_RATE                   = 15;
    public static final int USBCAM_BRIGHTNESS                   = 20;

    public static final double CAMERA_DEPTH                     = -17; // Inches from center of EE to center of camera, + = forward
    public static final double CAMERA_OFFSET                    = 3; // Inches from center of EE to center of camera, + = right
    public static final double CAMERA_DATA_TIMEOUT              = 0.5; // 500ms

    //
    // Ultrasonic sensors.
    //
    public static final double SONAR_INCHES_PER_VOLT            = 1.0/0.0098; //9.8mV per inch
    public static final double SONAR_ERROR_THRESHOLD            = 50.0; //value should not jump 50-in per time slice.
    public static final double SONAR_LEFT_DISTANCE_OFFSET       = -5.0;
    public static final double SONAR_RIGHT_DISTANCE_OFFSET      = -5.0;
    public static final double SONAR_FRONT_DISTANCE_OFFSET      = -5.0;

    //
    // DriveBase subsystem.
    //
    public static final double DRIVE_STALL_TIMEOUT              = 0.5;
    public static final double DRIVE_SLOW_XSCALE                = 3.0;
    public static final double DRIVE_SLOW_YSCALE                = 3.0;
    public static final double DRIVE_SLOW_TURNSCALE             = 3.0;

    public static final double DRIVE_WHEEL_RADIUS_IN            = 4.0;
    public static final double DRIVE_MOTOR_ROTATIONS_PER_WHEEL_ROTATION= 12.0;
    public static final double MOTOR_MAX_TORQUE                 = 343.4; //oz-in
    public static final double MAX_WHEEL_FORCE_OZ               = (MOTOR_MAX_TORQUE * DRIVE_MOTOR_ROTATIONS_PER_WHEEL_ROTATION) / DRIVE_WHEEL_RADIUS_IN;
    public static final double DRIVE_ENCODER_COUNTS_PER_ROTATION= 1440.0;
    public static final double DRIVE_GYRO_ASSIST_KP             = 1.5;
    public static final double DRIVE_MAX_ROTATION_RATE          = 6.5;      //radians per second
    
    public static final double DRIVE_MAX_XPID_POWER             = 0.7;
    public static final double DRIVE_MAX_YPID_POWER             = 0.7;
    public static final double DRIVE_MAX_TURNPID_POWER          = 0.7;

    // 2017-03-21: 0.0152347136491642, 0.15, 0.0, 0.0
    // 0.7 power is pretty gud fam
    // 2-20-2018: 0.0148258400720388, 0.15, 0.0, 0.0 -- competition robot
    // 3-28-2018: 0.00874986860034917173913043478261, 0.25, 0.0, 0.015  -- competition robot
    public static final double ENCODER_X_INCHES_PER_COUNT       = 0.00874986860034917173913043478261;//0.0129836759876149;//0.0148258400720388;//0.0144546649145861;//0.0152347136491642;//0.0264367338026265;
    public static final double ENCODER_X_KP                     = 0.25;
    public static final double ENCODER_X_KI                     = 0.0;
    public static final double ENCODER_X_KD                     = 0.015;
    public static final double ENCODER_X_KF                     = 0.0;
    public static final double ENCODER_X_TOLERANCE              = 1.0;

    // 2-20-2018: 0.0171099037270041, 0.04, 0.0, 0.0077 -- competition robot
    // 2-24-2018: 0.0172358143438125, 0.02, 0.0, 0.004 --practice robot
    
    // 3-16-2018: 0.0172358143438125; 0.03, 0.0, 0.001 -- practice robot
    // 3-28-2018: 0.01557713764963807531380753138075, 0.02, 0.0, 0.0015   -- competition robot
    // 0.01557713764963807531380753138075
    // 0.0168605528551718
    // don't worry this is for tank drive im not crazy i swear
    public static final double ENCODER_Y_INCHES_PER_COUNT       = 0.0176933159; // 0.0175344339;
    public static final double ENCODER_Y_KP                     = 0.01;
    public static final double ENCODER_Y_KI                     = 0.0;
    public static final double ENCODER_Y_KD                     = 0.003;
    public static final double ENCODER_Y_KF                     = 0.0;
    public static final double ENCODER_Y_TOLERANCE              = 2.0;

    // 2017-04-05: 0.03, 0.0, 0.003
    // 2-20-2018: 0.02, 0.0, 0.0025
    // 2-20-2018: 0.015, 0.0, 0.0012 -- competition robot
    //2-24-2018: 0.017, 0.0, 0.0017  - practice robot constants
    //2-27-2018: 0.015, 0.0, 0.0    -- practice robot
    //3-16-2018: 0.016, 0.0, 0.001    -- practice robot
    //3-17-2018: 0.012, 0.0, 0.001    -- practice robot
    //3-24-2018: 0.018, 0.0, 0.0016   -- competition robot
    //3-28-2018: 0.02, 0.0, 0.00175   -- competition robot
    public static final double GYRO_TURN_KP                     = 0.02;
    public static final double GYRO_TURN_KI                     = 0.0;
    public static final double GYRO_TURN_KD                     = 0.00175;
    public static final double GYRO_TURN_KF                     = 0.0;
    public static final double GYRO_TURN_TOLERANCE              = 2.0;

    //
    // Elevator subsystem.
    //
    //2-20-2018: 0.0071644803229062, 0.08, 0.0, 0.001
    public static final double ELEVATOR_INCHES_PER_COUNT        = 0.0071644803229062;//0.00577778;   // 39 inches in 6750 ticks
    public static final double ELEVATOR_KP                      = 0.2;      // this too
    public static final double ELEVATOR_KI                      = 0.001;    // hopefully not this
    public static final double ELEVATOR_KD                      = 0.0;      // this too
    public static final double ELEVATOR_TOLERANCE               = 1.0;      // this too
    public static final double ELEVATOR_GRAVITY_COMPENSATION    = 0.0;      // was 0.08 before added counter-balance
    public static final double ELEVATOR_CAL_POWER               = 0.3;      // this too
    public static final double ELEVATOR_POSITION_OFFSET         = 8.0;
    public static final double ELEVATOR_PID_FLOOR               = 6.0;
    public static final double ELEVATOR_PID_CEILING             = 82.0;     //need calibration

    public static final double ELEVATOR_STALL_MIN_POWER         = 0.3;
    public static final double ELEVATOR_STALL_TIMEOUT           = 0.5;
    public static final double ELEVATOR_STALL_RESET_TIMEOUT     = 0.5;
    public static final double ELEVATOR_MIN_HEIGHT              = ELEVATOR_POSITION_OFFSET;
    public static final double ELEVATOR_MAX_HEIGHT              = ELEVATOR_PID_CEILING - 1.0;
    public static final double ELEVATOR_FLOOR_PICKUP_HEIGHT     = ELEVATOR_MIN_HEIGHT;  // Lowest point on elevator
    public static final double ELEVATOR_OFF_GROUND              = ELEVATOR_MIN_HEIGHT + 8.0; // 16 in
    public static final double ELEVATOR_CRUISE_HEIGHT           = 35.0;
    public static final double ELEVATOR_SWITCH_HEIGHT           = SWITCH_FENCE_HEIGHT + CUBE_HEIGHT + 5.0; // fence height + cube height + 5 in buffer
    public static final double ELEVATOR_SCALE_LOW               = 48.0 + CUBE_HEIGHT + 5.0; // 4ft + cube height + 5 in buffer
    public static final double ELEVATOR_SCALE_MED               = 60.0 + CUBE_HEIGHT + 5.0; // 5ft + cube height + 5 in buffer
    public static final double ELEVATOR_SCALE_HIGH              = 72.0 + CUBE_HEIGHT + 5.0; // 6ft + cube height + 5 in buffer
    public static final double ELEVATOR_EXCHANGE_HEIGHT         = ELEVATOR_MIN_HEIGHT + 2.0;
    public static final double[] ELEVATOR_HEIGHTS = new double[] { ELEVATOR_OFF_GROUND,
                                                                   ELEVATOR_SWITCH_HEIGHT,
                                                                   ELEVATOR_SCALE_LOW,
                                                                   ELEVATOR_SCALE_MED,
                                                                   ELEVATOR_SCALE_HIGH };
    //
    // CubePickup subsystem.
    //
    public static final double PICKUP_FREE_SPIN_CURRENT         = 10.0;
    public static final double PICKUP_STALL_CURRENT             = 45.0;
    public static final double PICKUP_TELEOP_POWER              = 0.6;
    public static final double PICKUP_HOLD_CUBE_POWER           = 0.1;

    //
    // Flipper subsystem.
    //
    public static final double FLIPPER_EXTEND_PERIOD            = 0.75;

    //
    // AutoAssist subsystems.
    //
    public static final double EXCHANGE_ALIGN_STRAFE_DIST       = 60.0;     // 5 feet
    public static final double EXCHANGE_ALIGN_TIMEOUT           = 4.0;      // 4 seconds
    public static final double EXCHANGE_ALIGN_WALL_DIST         = 12.0;     // 1 foot
    public static final double EXCHANGE_ALIGN_SENSOR_OFFSET     = -8.0; // TUNE THIS. Inches offset from center of cube pickup
    public static final double AUTO_PICKUP_MOVE_POWER           = 0.6;      // 60% power
    public static final double FIND_CUBE_X_TOLERANCE            = 1.0;      // 1-in
    public static final double FIND_CUBE_STRAFE_POWER           = 0.6;      // 60% power
    public static final double AUTO_PICKUP_CUBE_DISTANCE        = 60.0;//24.0;

    //
    // CmdPowerUpAuto variables.
    //
    public static final double AUTO_DISTANCE_TO_SWITCH          = 145.0;
    public static final double FINAL_FRONT_SCALE_APPROACH_DISTANCE= 38.0;    // TODO: need to tune this
    public static final double FINAL_SIDE_SCALE_APPROACH_DISTANCE= 0.0;
    public static final double RIGHT_SWITCH_LOCATION            = 102.0;
    public static final double LEFT_SWITCH_LOCATION             = -102.0;
    public static final double ADVANCE_TO_SECOND_CUBE_DISTANCE  = 64.0;
    public static final double STRAFE_TO_SECOND_CUBE_DISTANCE   = 36.0;
    public static final double SCALE_FRONT_POSITION             = 75.0;
    public static final double SCALE_SIDE_POSITION              = 130.0;
    public static final double FIRST_ELEVATOR_HEIGHT            = 30.0;
    public static final double ADVANCE_AROUND_SCALE_DISTANCE    = 77.0;
    public static final double SWITCH_STRAFE_DISTANCE           = 26.0;
    public static final double MAX_CUBE_DISTANCE                = 20.0;
    public static final double SWITCH_SONAR_DISTANCE_THRESHOLD  = 16.0;
    public static final double OPPOSITE_SWITCH_OVERSHOOT        = 35.0;
    public static final double STRAFE_FROM_SWITCH_DISTANCE      = 24.0;
    public static final double POSITION_TO_STRAFE_DISTANCE      = 108.0;
    
    //
    // CmdScaleAuto constants.
    //
    public static final double SWITCH_TO_WALL_DISTANCE          = 85.25;
    public static final double SCALE_TO_WALL_DISTANCE           = 71.57;
    public static final double ROBOT_TO_SCALE_DISTANCE          = 36.0; //30.0
    // CodeReview: This is duplicating FWD_DISTANCE_3, please use one or the other or alias them.
    public static final double ALLIANCE_WALL_TO_SCALE_DISTANCE  = 324.0;
    public static final double DRIVE_ACROSS_FIELD_DISTANCE      = 220.0;//SWITCH_FENCE_WIDTH + SWITCH_TO_WALL_DISTANCE;
    public static final double CUBE_PICKUP_DROP_POWER           = 1.0;
    public static final double DROP_CUBE_TIMEOUT                = 0.5;

    //
    // FrcAuto constants.
    //
    public static final double FWD_DISTANCE_1                   = 10.0;
    public static final double FWD_DISTANCE_2                   = 60.0;
    public static final double FWD_DISTANCE_3                   = 235.0;
    public static enum Position {LEFT_POS, MID_POS, RIGHT_POS};
    public static final Position LEFT_START_POS                 = Position.LEFT_POS;
    public static final Position MID_START_POS                  = Position.MID_POS;
    public static final Position RIGHT_START_POS                = Position.RIGHT_POS;

}   // class RobotInfo
