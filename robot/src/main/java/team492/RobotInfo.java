/*
 * Copyright (c) 2019 Titan Robotics Club (http://www.titanrobotics.com)
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
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHEPIXYRWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package team492;

import frclib.FrcPixyCam1;

public class RobotInfo
{
    //
    // Field dimensions in inches.
    //
    public static final double FIELD_LENGTH                     = 54*12.0;
    public static final double FIELD_WIDTH                      = 27*12.0;

    public static final double BATTERY_CAPACITY_WATT_HOUR       = 18.0*12.0;

    //
    // Robot dimensions.
    //

    //
    // Joystick ports.
    //
    public static final int JSPORT_LEFT_DRIVESTICK              = 0;
    public static final int JSPORT_RIGHT_DRIVESTICK             = 1;
    public static final int JSPORT_OPERATORSTICK                = 2;
    public static final int JSPORT_BUTTON_PANEL                 = 3;

    //
    // CAN IDs.
    //
    public static final int CANID_LEFTFRONTWHEEL                = 3;    // 40A: Orange
    public static final int CANID_RIGHTFRONTWHEEL               = 4;    // 40A: Yellow
    public static final int CANID_LEFTREARWHEEL                 = 5;    // 40A: Green
    public static final int CANID_RIGHTREARWHEEL                = 6;    // 40A: Blue
    public static final int CANID_LEFT_DRIVE_MASTER             = 7;    // 40A: Purple
    public static final int CANID_RIGHT_DRIVE_MASTER            = 8;    // 40A: Gray
    public static final int CANID_ELEVATOR                      = 9;    // 40A: White
    public static final int CANID_PICKUP_PITCH                  = 10;   // 40A: Orange
    public static final int CANID_RESERVED                      = 11;   // 30A: Yellow
    public static final int CANID_PICKUP                        = 12;   // 30A: Green

    public static final int CANID_PIGEON                        = 15;
    public static final int CANID_PDP                           = 16;
    public static final int CANID_PCM1                          = 17;
    public static final int CANID_PCM2                          = 18;

    //
    // PDP Channels.
    //
    public static final int PDP_CHANNEL_RIGHT_REAR_WHEEL        = 0;
    public static final int PDP_CHANNEL_RIGHT_DRIVE_MASTER      = 1;
    public static final int PDP_CHANNEL_PICKUP_PITCH            = 2;
    public static final int PDP_CHANNEL_RIGHT_FRONT_WHEEL       = 3;
    public static final int PDP_CHANNEL_PICKUP                  = 4;
    public static final int PDP_CHANNEL_PIGEON                  = 8;
    public static final int PDP_CHANNEL_BLINKIN                 = 9;
    public static final int PDP_CHANNEL_RING_LIGHT              = 10;
    public static final int PDP_CHANNEL_RESERVED                = 11;
    public static final int PDP_CHANNEL_LEFT_FRONT_WHEEL        = 12;
    public static final int PDP_CHANNEL_ELEVATOR                = 13;
    public static final int PDP_CHANNEL_LEFT_DRIVE_MASTER       = 14;
    public static final int PDP_CHANNEL_LEFT_REAR_WHEEL         = 15;

    //
    // Analog Input ports.
    //
    public static final int AIN_PRESSURE_SENSOR                 = 0;

    //
    // Digital Input/Output ports.
    //
    public static int DIO_CARGO_PROXIMITY_SENSOR                = 0;


    //
    // PWM Channels.
    //
    public static final int PWM_REV_BLINKIN                     = 0;

    //
    // Relay channels.
    //
    public static final int RELAY_RINGLIGHT_POWER               = 0;    // 20A: Green

    //
    // Solenoid channels.
    //
    public static final int SOL_HATCH_DEPLOYER_EXTEND           = 4;
    public static final int SOL_HATCH_DEPLOYER_RETRACT          = 3;

    //
    // Vision subsystem.
    //
    public static final int PIXYCAM_WIDTH                       = 320;
    public static final int PIXYCAM_HEIGHT                      = 200;
    public static final int PIXY_BRIGHTNESS                     = 80;
    public static final double PIXY_CAM_OFFSET                  = 8.0;
    public static final PixyVision.Orientation PIXY_ORIENTATION = PixyVision.Orientation.NORMAL_LANDSCAPE;
    public static final int PIXYCAM_I2C_ADDRESS                 = FrcPixyCam1.DEF_I2C_ADDRESS;
    public static final int PIXY_TARGET_SIGNATURE               = 1;
    //
    // Pixy line following subsystem
    // PIXY2_LINE_TRACKING_HEIGHT and PIXY2_LINE_TRACKING_WIDTH are the dimensions of the Pixy2's line-tracking resolution.
    // Source: https://docs.pixycam.com/wiki/doku.php?id=wiki:v2:line_api
    //
    public static final double PIXY2_LINE_TRACKING_HEIGHT       = 51.0; // in pixels.
    public static final double PIXY2_LINE_TRACKING_WIDTH        = 78.0; // in pixels.
    // These should be in real-world robot coordinates. Needs calibration after camera is actually mounted in position.
    // TODO: Tune all of this
    public static final double PIXYCAM_WORLD_TOPLEFT_X          = 0;   // in real-world units.
    public static final double PIXYCAM_WORLD_TOPLEFT_Y          = 0;   // in real-world units.
    public static final double PIXYCAM_WORLD_TOPRIGHT_X         = 0;   // in real-world units.
    public static final double PIXYCAM_WORLD_TOPRIGHT_Y         = 0;   // in real-world units.
    public static final double PIXYCAM_WORLD_BOTTOMLEFT_X       = 0;   // in real-world units.
    public static final double PIXYCAM_WORLD_BOTTOMLEFT_Y       = 0;   // in real-world units.
    public static final double PIXYCAM_WORLD_BOTTOMRIGHT_X      = 0;   // in real-world units.
    public static final double PIXYCAM_WORLD_BOTTOMRIGHT_Y      = 0;   // in real-world units.

    public static final double CAMERA_DEPTH                     = 4; // Inches from center of EE to center of camera, + = backward
    public static final double CAMERA_OFFSET                    = 0; // Inches from center of EE to center of camera, + = right

    //
    // Ultrasonic sensors.
    //
    public static final double SONAR_INCHES_PER_VOLT            = 1.0/0.0098; //9.8mV per inch
    public static final double SONAR_ERROR_THRESHOLD            = 50.0; //value should not jump 50-in per time slice.

    //
    // DriveBase subsystem.
    //
    public static final double DRIVE_STALL_TIMEOUT              = 0.5;
    public static final double DRIVE_SLOW_XSCALE                = 3.0;
    public static final double DRIVE_SLOW_YSCALE                = 3.0;
    public static final double DRIVE_SLOW_TURNSCALE             = 3.0;

    public static final double DRIVE_GYRO_ASSIST_KP             = 1.5;
    public static final double DRIVE_MAX_ROTATION_RATE          = 6.5;      //radians per second

    public static final double DRIVE_MAX_XPID_POWER             = 1.0;
    public static final double DRIVE_MAX_YPID_POWER             = 1.0;
    public static final double DRIVE_MAX_TURNPID_POWER          = 1.0;

    public static final double ENCODER_X_INCHES_PER_COUNT       = 0.0179162073;
    public static final double ENCODER_X_KP                     = 0.25;
    public static final double ENCODER_X_KI                     = 0.0;
    public static final double ENCODER_X_KD                     = 0.015;
    public static final double ENCODER_X_KF                     = 0.0;
    public static final double ENCODER_X_TOLERANCE              = 1.0;

    public static final double ENCODER_Y_INCHES_PER_COUNT       = 2.125436537;
    public static final double ENCODER_Y_KP                     = 0.01;
    public static final double ENCODER_Y_KI                     = 0.0;
    public static final double ENCODER_Y_KD                     = 0.003;
    public static final double ENCODER_Y_KF                     = 0.0;
    public static final double ENCODER_Y_TOLERANCE              = 2.0;

    public static final double GYRO_TURN_KP                     = 0.02;
    public static final double GYRO_TURN_KI                     = 0.0;
    public static final double GYRO_TURN_KD                     = 0.00175;
    public static final double GYRO_TURN_KF                     = 0.0;
    public static final double GYRO_TURN_TOLERANCE              = 2.0;

    //
    // Pickup subsystem
    //
    // TODO: Tune all of this
    public static final double PICKUP_DEGREES_PER_COUNT         = 1.0;
    public static final double PICKUP_KP                        = 0.0;
    public static final double PICKUP_KI                        = 0.0;
    public static final double PICKUP_KD                        = 0.0;
    public static final double PICKUP_TOLERANCE                 = 2.0; // 2 degrees
    public static final double PICKUP_CALIBRATE_POWER           = 0.0;
    public static final double PICKUP_MIN_POS                   = 0.0; // Parallel to ground
    public static final double PICKUP_MAX_POS                   = 90.0; // Perpendicular to ground
    public static final double PICKUP_PID_FLOOR                 = PICKUP_MIN_POS - 2.0;
    public static final double PICKUP_PID_CEILING               = PICKUP_MAX_POS + 2.0;
    public static final double PICKUP_GRAVITY_COMP              = 0.0;
    public static final double PICKUP_STALL_MIN_POWER           = 0.3;
    public static final double PICKUP_STALL_TIMEOUT             = 0.5;
    public static final double PICKUP_STALL_RESET_TIMEOUT       = 0.5;
    public static final double PICKUP_CURRENT_THRESHOLD         = 2.5; // Free=1.5,startup=2-4,cargopickup=5.5

    public static final double PICKUP_CARGO_PICKUP_TIMEOUT      = 5.0; // in seconds
    public static final double PICKUP_CARGO_PICKUP_POWER        = 0.7;
    public static final double PICKUP_CARGO_DEPLOY_POWER        = -1.0;

    //
    // Elevator subsystem
    //
    // TODO: Tune all of this
    public static final double ELEVATOR_INCHES_PER_COUNT        = 1.0;
    public static final double ELEVATOR_KP                      = 0.0;
    public static final double ELEVATOR_KI                      = 0.0;
    public static final double ELEVATOR_KD                      = 0.0;
    public static final double ELEVATOR_TOLERANCE               = 1.0; // 1 in
    public static final double ELEVATOR_CALIBRATE_POWER         = 0.0;
    public static final double ELEVATOR_MIN_POS                 = 6.0;
    public static final double ELEVATOR_MAX_POS                 = 60.0;
    public static final double ELEVATOR_PID_FLOOR               = ELEVATOR_MIN_POS - 2.0;
    public static final double ELEVATOR_PID_CEILING             = ELEVATOR_MAX_POS + 2.0;
    public static final double ELEVATOR_GRAVITY_COMP            = 0.0;
    public static final double ELEVATOR_STALL_MIN_POWER         = 0.3;
    public static final double ELEVATOR_STALL_TIMEOUT           = 0.5;
    public static final double ELEVATOR_STALL_RESET_TIMEOUT     = 0.5;

    public static final double ELEVATOR_DRIVE_POS               = 20.0;

    public static final double ELEVATOR_POS_CARGO_ROCKET_LOW    = 20.0; // I'm just spitballing here, man.
    public static final double ELEVATOR_POS_CARGO_ROCKET_MED    = 40.0;
    public static final double ELEVATOR_POS_CARGO_ROCKET_HIGH   = 60.0;

    public static final double ELEVATOR_POS_HATCH_ROCKET_LOW    = 20.0;
    public static final double ELEVATOR_POS_HATCH_ROCKET_MED    = 40.0;
    public static final double ELEVATOR_POS_HATCH_ROCKET_HIGH   = 60.0;

    public static final double ELEVATOR_POS_CARGO_SHIP          = ELEVATOR_POS_CARGO_ROCKET_LOW;
    public static final double ELEVATOR_POS_HATCH_SHIP          = ELEVATOR_POS_HATCH_ROCKET_LOW;

    public static final double ELEVATOR_POS_CARGO_PICKUP        = ELEVATOR_POS_CARGO_ROCKET_LOW;
    public static final double ELEVATOR_POS_HATCH_PICKUP        = ELEVATOR_POS_HATCH_ROCKET_LOW;

    public static final double HAB_1_DRIVE_OFF_DIST             = 55.0;
    public static final double DRIVE_TO_CARGO_SHIP_FRONT_DIST   = 100.0;
}   // class RobotInfo
