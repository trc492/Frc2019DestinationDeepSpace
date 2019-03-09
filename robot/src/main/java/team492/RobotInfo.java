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
import trclib.TrcUtil;

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
    public static final int SOL_HATCH_DEPLOYER_EXTEND           = 2;
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
    public static final double PIXY2_LINE_TRACKING_HEIGHT        = 51.0; // in pixels.
    public static final double PIXY2_LINE_TRACKING_WIDTH         = 78.0; // in pixels.
    public static final double PIXY2_LINE_TRACK_MID_WIDTH_OFFST  = 0.0;  // in pixels.
    public static final double PIXY2_LINE_TRACK_MID_HEIGHT_OFFST = 0.0;  // in pixels.
    // These should be in real-world robot coordinates. Needs calibration after camera is actually mounted in position.
    // TODO: Tune all of this
    public static final double PIXYCAM_WORLD_TOPLEFT_X          = -61.0;   // in real-world units.
    public static final double PIXYCAM_WORLD_TOPLEFT_Y          = 83.0;   // in real-world units.
    public static final double PIXYCAM_WORLD_TOPRIGHT_X         = 33.0;   // in real-world units.
    public static final double PIXYCAM_WORLD_TOPRIGHT_Y         = 83.0;   // in real-world units.
    public static final double PIXYCAM_WORLD_BOTTOMLEFT_X       = -39.5;   // in real-world units.
    public static final double PIXYCAM_WORLD_BOTTOMLEFT_Y       = 19.0;   // in real-world units.
    public static final double PIXYCAM_WORLD_BOTTOMRIGHT_X      = 12.0;   // in real-world units.
    public static final double PIXYCAM_WORLD_BOTTOMRIGHT_Y      = 19.0;   // in real-world units.

    public static final double CAMERA_DEPTH                     = 16; // Inches from center of EE to center of camera, + = backward
    public static final double CAMERA_OFFSET                    = 3; // Inches from center of EE to center of camera, + = right

    //
    // Ultrasonic sensors.
    //
    public static final double SONAR_INCHES_PER_VOLT            = 1.0/0.0098; //9.8mV per inch
    public static final double SONAR_ERROR_THRESHOLD            = 50.0; //value should not jump 50-in per time slice.

    //
    // DriveBase subsystem.
    //
    public static final double DRIVE_STALL_TIMEOUT              = 0.5;

    public static final double DRIVE_SLOW_XSCALE                = 0.5;
    public static final double DRIVE_SLOW_YSCALE                = 0.5;
    public static final double DRIVE_SLOW_TURNSCALE             = 0.4;

    public static final double DRIVE_MEDIUM_XSCALE              = 0.75;
    public static final double DRIVE_MEDIUM_YSCALE              = 0.75;
    public static final double DRIVE_MEDIUM_TURNSCALE           = 0.6;

    public static final double DRIVE_FAST_XSCALE                = 1.0;
    public static final double DRIVE_FAST_YSCALE                = 1.0;
    public static final double DRIVE_FAST_TURNSCALE             = 0.8;

    public static final double DRIVE_GYRO_ASSIST_KP             = 1.5;
    public static final double DRIVE_MAX_ROTATION_RATE          = 6.5;      //radians per second

    public static final double DRIVE_MAX_XPID_POWER             = 0.5;
    public static final double DRIVE_MAX_YPID_POWER             = 0.6;
    public static final double DRIVE_MAX_TURNPID_POWER          = 1.0;

    public static final double DRIVE_MAX_XPID_RAMP_RATE         = 0.5;
    public static final double DRIVE_MAX_YPID_RAMP_RATE         = 0.6;
    public static final double DRIVE_MAX_TURNPID_RAMP_RATE      = 1.0;

    public static final double ENCODER_X_INCHES_PER_COUNT       = 1.971078567;
    public static final double ENCODER_X_KP                     = 0.012;
    public static final double ENCODER_X_KI                     = 0.0;
    public static final double ENCODER_X_KD                     = 0.0;
    public static final double ENCODER_X_KF                     = 0.0;
    public static final double ENCODER_X_TOLERANCE              = 1.0;

    public static final double ENCODER_X_KP_SMALL               = 0.024;
    public static final double ENCODER_X_KI_SMALL               = 0.0;
    public static final double ENCODER_X_KD_SMALL               = 0.0;
    public static final double ENCODER_X_KF_SMALL               = 0.0;
    public static final double ENCODER_X_TOLERANCE_SMALL        = 1.0;

    // comp robot: 0.02/0.0/0.0016
    public static final double ENCODER_Y_INCHES_PER_COUNT       = 2.360546194;
    public static final double ENCODER_Y_KP                     = 0.01;
    public static final double ENCODER_Y_KI                     = 0.0;
    public static final double ENCODER_Y_KD                     = 0.001;
    public static final double ENCODER_Y_KF                     = 0.0;
    public static final double ENCODER_Y_TOLERANCE              = 2.0;

    // Comp robot: 0.015/0.0/0.001
    public static final double GYRO_TURN_KP                     = 0.008;
    public static final double GYRO_TURN_KI                     = 0.0;
    public static final double GYRO_TURN_KD                     = 0.0007;
    public static final double GYRO_TURN_KF                     = 0.0;
    public static final double GYRO_TURN_TOLERANCE              = 2.0;

    //
    // Pickup subsystem
    //
    // TODO: Tune all of this
    public static final double PICKUP_DEGREES_PER_COUNT         = 0.0111609546;
    public static final double PICKUP_KP                        = 0.03;
    public static final double PICKUP_KI                        = 0.0;
    public static final double PICKUP_KD                        = 0.0;
    public static final double PICKUP_TOLERANCE                 = 2.0; // 2 degrees
    public static final double PICKUP_CALIBRATE_POWER           = 0.3;
    public static final double PICKUP_MIN_POS                   = 0.0; // Perpendicular to ground
    public static final double PICKUP_MAX_POS                   = 90.0; // Parallel to ground
    public static final double PICKUP_PID_FLOOR                 = PICKUP_MIN_POS - 2.0;
    public static final double PICKUP_PID_CEILING               = PICKUP_MAX_POS + 2.0;
    public static final double PICKUP_STALL_MIN_POWER           = 0.8;
    public static final double PICKUP_STALL_TIMEOUT             = 1.0;
    public static final double PICKUP_STALL_RESET_TIMEOUT       = 0.2;
    public static final double PICKUP_CURRENT_THRESHOLD         = 3.0; // Free=2.6,startup=7,cargopickup=5.5

    public static final double PICKUP_MASS                      = 4.08; // kilograms
    public static final double PICKUP_WEIGHT                    = PICKUP_MASS * TrcUtil.EARTH_GRAVITATIONAL_CONSTANT;//N
    public static final double PICKUP_CG_DISTANCE               = 0.286;// half of axle -> center of mass in meters
    public static final double PITCH_MOTOR_STALL_TORQUE         = 0.71; // Nm
    public static final double PITCH_MOTOR_GEAR_RATIO           = 974.0;
    public static final double PITCH_MOTOR_SHAFT_MAX_TORQUE     = PITCH_MOTOR_STALL_TORQUE * PITCH_MOTOR_GEAR_RATIO; // Max torque available from motor
    public static final double PICKUP_MAX_TORQUE                = PICKUP_WEIGHT * PICKUP_CG_DISTANCE; // Max torque required to hold up
    public static final double PICKUP_PERCENT_TORQUE            = PICKUP_MAX_TORQUE / PITCH_MOTOR_SHAFT_MAX_TORQUE;

    public static final double PICKUP_HATCH_PICKUP_POS          = 0.4;
    public static final double PICKUP_GROUND_CARGO_POS          = 54;
    public static final double PICKUP_PERP_TO_GROUND_POS        = PICKUP_MIN_POS;
    public static final double PICKUP_PARALLEL_TO_GROUND_POS    = PICKUP_MAX_POS;

    public static final double PICKUP_CARGO_PICKUP_TIMEOUT      = 5.0; // in seconds
    public static final double PICKUP_CARGO_PICKUP_POWER        = 1.0;
    public static final double PICKUP_CARGO_DEPLOY_POWER        = -1.0;

    //
    // Elevator subsystem
    //
    // TODO: Tune all of this
    // for mag encoder: 0.0019605117
    public static final double ELEVATOR_INCHES_PER_COUNT        = 0.0152322925;
    public static final double ELEVATOR_KP                      = 0.4;
    public static final double ELEVATOR_KI                      = 0.0;
    public static final double ELEVATOR_KD                      = 0.0;
    public static final double ELEVATOR_TOLERANCE               = 1.0; // 1 in
    public static final double ELEVATOR_CALIBRATE_POWER         = 0.5;
    public static final double ELEVATOR_MIN_POS                 = 5.25;
    public static final double ELEVATOR_MAX_POS                 = 71;
    public static final double ELEVATOR_PID_FLOOR               = ELEVATOR_MIN_POS - 2.0;
    public static final double ELEVATOR_PID_CEILING             = ELEVATOR_MAX_POS + 2.0;
    public static final double ELEVATOR_GRAVITY_COMP            = 0.05;
    public static final double ELEVATOR_STALL_MIN_POWER         = 0.8;
    public static final double ELEVATOR_STALL_TIMEOUT           = 1.0;
    public static final double ELEVATOR_STALL_RESET_TIMEOUT     = 0.5;

    public static final double ELEVATOR_POS_CARGO_ROCKET_LOW    = 14.5;
    public static final double ELEVATOR_POS_CARGO_ROCKET_MED    = 42.25;
    public static final double ELEVATOR_POS_CARGO_ROCKET_HIGH   = ELEVATOR_MAX_POS;
    public static final double[] ELEVATOR_CARGO_ROCKET_POSITIONS = new double[] { ELEVATOR_POS_CARGO_ROCKET_LOW,
        ELEVATOR_POS_CARGO_ROCKET_MED, ELEVATOR_POS_CARGO_ROCKET_HIGH };

    public static final double ELEVATOR_POS_HATCH_ROCKET_LOW    = ELEVATOR_MIN_POS;
    public static final double ELEVATOR_POS_HATCH_ROCKET_MED    = 35.5;
    public static final double ELEVATOR_POS_HATCH_ROCKET_HIGH   = 64.875;
    public static final double[] ELEVATOR_HATCH_ROCKET_POSITIONS = new double[] { ELEVATOR_POS_HATCH_ROCKET_LOW,
        ELEVATOR_POS_HATCH_ROCKET_MED, ELEVATOR_POS_HATCH_ROCKET_HIGH };

    public static final double ELEVATOR_DRIVE_POS               = ELEVATOR_POS_HATCH_ROCKET_MED;

    public static final double ELEVATOR_POS_HATCH_SHIP          = ELEVATOR_POS_HATCH_ROCKET_LOW;

    public static final double ELEVATOR_POS_HATCH_PICKUP        = ELEVATOR_POS_HATCH_ROCKET_LOW;

    public static final double ROCKET_SIDE_ANGLE                = 61.23; // Angle in degrees between side wall and angled rocket side.

    public static final double HAB_1_DRIVE_OFF_DIST             = 55.0;
    public static final double DRIVE_TO_CARGO_SHIP_FRONT_DIST   = 100.0;
}   // class RobotInfo
