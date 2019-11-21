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

import trclib.TrcPidController;

public class RobotInfo
{
    //
    // Field dimensions in inches.
    //
    public static final double FIELD_LENGTH = 54 * 12.0;
    public static final double FIELD_WIDTH = 27 * 12.0;

    public static final double BATTERY_CAPACITY_WATT_HOUR = 18.0 * 12.0;

    public static final double BATTERY_NOMINAL_VOLTAGE = 12.0;

    //
    // Robot dimensions.
    //
    public static final double ROBOT_WIDTH = 30;
    public static final double ROBOT_LENGTH = 30;

    //
    // Joystick ports.
    //
    public static final int JSPORT_LEFT_DRIVESTICK = 0;
    public static final int JSPORT_RIGHT_DRIVESTICK = 1;
    public static final int JSPORT_OPERATORSTICK = 2;
    public static final int JSPORT_BUTTON_PANEL = 3;
    public static final int JSPORT_SWITCH_PANEL = 4;

    //
    // CAN IDs.
    //
    public static final int CANID_LEFTFRONT_STEER = 3;    // 40A: Orange
    public static final int CANID_RIGHTFRONT_STEER = 4;    // 40A: Yellow
    public static final int CANID_LEFTREAR_STEER = 5;    // 40A: Green
    public static final int CANID_RIGHTREAR_STEER = 6;    // 40A: Blue

    public static final int CANID_LEFTFRONT_DRIVE = 13;    // 40A: Orange
    public static final int CANID_RIGHTFRONT_DRIVE = 14;    // 40A: Yellow
    public static final int CANID_LEFTREAR_DRIVE = 15;    // 40A: Green
    public static final int CANID_RIGHTREAR_DRIVE = 16;    // 40A: Blue

    public static final int CANID_PDP = 26;
    public static final int CANID_PCM1 = 17;
    public static final int CANID_PCM2 = 18;

    //
    // PDP Channels.
    //
    public static final int PDP_CHANNEL_RIGHT_REAR_WHEEL = 0;
    public static final int PDP_CHANNEL_RIGHT_FRONT_WHEEL = 3;
    public static final int PDP_CHANNEL_LEFT_FRONT_WHEEL = 12;
    public static final int PDP_CHANNEL_LEFT_REAR_WHEEL = 15;

    //
    // DriveBase subsystem.
    //
    public static final double DRIVE_STALL_TIMEOUT = 0.5;

    public static final double DRIVE_SLOW_XSCALE = 0.5;
    public static final double DRIVE_SLOW_YSCALE = 0.5;
    public static final double DRIVE_SLOW_TURNSCALE = 0.4;

    public static final double DRIVE_MEDIUM_XSCALE = 0.75;
    public static final double DRIVE_MEDIUM_YSCALE = 0.75;
    public static final double DRIVE_MEDIUM_TURNSCALE = 0.6;

    public static final double DRIVE_FAST_XSCALE = 1.0;
    public static final double DRIVE_FAST_YSCALE = 1.0;
    public static final double DRIVE_FAST_TURNSCALE = 0.8;

    public static final double DRIVE_MAX_XPID_POWER = 0.5;
    public static final double DRIVE_MAX_YPID_POWER = 0.6;
    public static final double DRIVE_MAX_TURNPID_POWER = 1.0;

    public static final double DRIVE_MAX_XPID_RAMP_RATE = 0.5;
    public static final double DRIVE_MAX_YPID_RAMP_RATE = 0.6;
    public static final double DRIVE_MAX_TURNPID_RAMP_RATE = 1.0;

    public static final double STEER_DEGREES_PER_TICK = 360.0 / 4096.0;
    public static final double STEER_MAX_REQ_VEL = 1000.0; // deg/sec. max commanded velocity, not necessarily max vel
    public static final double STEER_MAX_ACCEL = 5000; // deg/sec^2
    // ((theoretical max rpm * speed loss constant / gear ratio) / 60 sec/min) * 360 deg/rev
    public static final double STEER_MAX_VEL = ((18700 * 0.81 / 56.67) / 60.0) * 360.0; // deg/sec
    public static final double STEER_MAX_VEL_TICKS_PER_100MS = (STEER_MAX_VEL / STEER_DEGREES_PER_TICK) / 10.0; // ticks/100ms
    public static final double STEER_TOLERANCE = 2.0; // only used for pid, not magic

    public static final TrcPidController.PidCoefficients magicSteerCoeff = new TrcPidController.PidCoefficients(2.0, 0.01, 0,
        1023.0 / STEER_MAX_VEL_TICKS_PER_100MS, 5.0 / STEER_DEGREES_PER_TICK);
    public static final TrcPidController.PidCoefficients pidSteerCoeff = new TrcPidController.PidCoefficients(0, 0, 0,
        0);

    public static final double ENCODER_INCHES_PER_COUNT = 2.355935875;
    public static final double ENCODER_KP = 0.011;
    public static final double ENCODER_KI = 0.0;
    public static final double ENCODER_KD = 0.001;
    public static final double ENCODER_KF = 0.0;
    public static final double ENCODER_TOLERANCE = 2.0;

    // Comp robot: 0.015/0.0/0.001
    // practice robot: 0.008/0.0/0.0007
    // 3/24 comp robot: 0.0055/0.0/0.00008
    public static final double GYRO_TURN_KP = 0.0055;
    public static final double GYRO_TURN_KI = 0.0;
    public static final double GYRO_TURN_KD = 0.00008;
    public static final double GYRO_TURN_KF = 0.0;
    public static final double GYRO_TURN_TOLERANCE = 2.0;

    // 3/24 compr robot: 0.0085/0.0/0.0
    public static final double GYRO_TURN_KP_SMALL = 0.0085;
    public static final double GYRO_TURN_KI_SMALL = 0.0;
    public static final double GYRO_TURN_KD_SMALL = 0.0;
    public static final double GYRO_TURN_KF_SMALL = 0.0;
    public static final double GYRO_TURN_TOLERANCE_SMALL = 1.5;
}   // class RobotInfo
