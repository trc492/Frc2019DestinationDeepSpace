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
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package team492;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frclib.FrcAHRSGyro;
import frclib.FrcCANSparkMax;
import frclib.FrcEmic2TextToSpeech;
import frclib.FrcI2cLEDPanel;
import frclib.FrcJoystick;
import frclib.FrcPdp;
import frclib.FrcRobotBase;
import frclib.FrcRobotBattery;
import hallib.HalDashboard;
import team492.PixyVision.TargetInfo;
import trclib.TrcEmic2TextToSpeech.Voice;
import trclib.TrcMecanumDriveBase;
import trclib.TrcPidController;
import trclib.TrcPidController.PidCoefficients;
import trclib.TrcPidDrive;
import trclib.TrcPixyCam2.Vector;
import trclib.TrcRobot.RunMode;
import trclib.TrcRobotBattery;
import trclib.TrcUtil;

import java.util.Date;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TrcRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the
 * resource directory.
 */
public class Robot extends FrcRobotBase
{
    public static final String programName = "DestinationDeepSpace";

    public static final boolean USE_TRACELOG = true;
    public static final boolean USE_NAV_X = true;
    public static final boolean USE_TEXT_TO_SPEECH = false;
    public static final boolean USE_MESSAGE_BOARD = false;
    public static final boolean USE_GYRO_ASSIST = false;
    public static final boolean USE_RASPI_VISION = true;
    public static final boolean USE_PIXY_I2C = true;
    public static final boolean USE_PIXY_V2 = true;
    public static final boolean USE_PIXY_LINE_TARGET = true;

    private static final boolean DEBUG_POWER_CONSUMPTION = false;
    private static final boolean DEBUG_DRIVE_BASE = false;
    private static final boolean DEBUG_PID_DRIVE = false;
    private static final boolean DEBUG_SUBSYSTEMS = false;
    private static final boolean DEBUG_PIXY = true;

    private static final double DASHBOARD_UPDATE_INTERVAL = 0.1;
    private static final double SPEAK_PERIOD_SECONDS = 20.0; // Speaks once every this # of second.
    private static final double IDLE_PERIOD_SECONDS = 300.0;

    public DriverStation ds = DriverStation.getInstance();
    public HalDashboard dashboard = HalDashboard.getInstance();

    public double targetHeading = 0.0;

    private double nextUpdateTime = TrcUtil.getCurrentTime();

    // FMS provided the following info:
    //  - event name
    //  - match type
    //  - match number
    //  - alliance
    //  - location
    //  - replay number???

    public String eventName = "Unknown";
    public MatchType matchType = MatchType.None;
    public int matchNumber = 0;
    public Alliance alliance = Alliance.Red;
    public int location = 1;
    public String gameSpecificMessage = null;
    public boolean traceLogOpened = false;
    //
    // Inputs.
    //
    public FrcJoystick leftDriveStick = null;
    public FrcJoystick rightDriveStick = null;
    public FrcJoystick operatorStick = null;
    //
    // Sensors.
    //
    public FrcPdp pdp = null;
    public TrcRobotBattery battery = null;
    public FrcAHRSGyro gyro = null;
    public AnalogInput pressureSensor = null;

    //
    // Primary robot subystems
    //
    public Pickup pickup;
    public Elevator elevator;

    public CmdAutoDeploy autoDeploy;

    //
    // VisionTargetPipeline subsystem.
    //
    public PixyVision pixy = null;
    public RaspiVision vision = null;
    //
    // Miscellaneous subsystem.
    //
    public FrcEmic2TextToSpeech tts = null;
    private double nextTimeToSpeakInSeconds = 0.0;  //0 means disabled, no need to speak;
    public FrcI2cLEDPanel messageBoard = null;
    //
    // DriveBase subsystem.
    //
    public FrcCANSparkMax leftFrontWheel;
    public FrcCANSparkMax leftRearWheel;
    public FrcCANSparkMax rightFrontWheel;
    public FrcCANSparkMax rightRearWheel;
    public TrcMecanumDriveBase driveBase;

    public TrcPidController encoderXPidCtrl;
    public TrcPidController encoderYPidCtrl;
    public TrcPidController gyroTurnPidCtrl;
    public TrcPidDrive pidDrive;
    //
    // Define our subsystems for Auto and TeleOp modes.
    //
    public double driveTime;
    public double drivePower;
    public double driveDistance;
    public double turnDegrees;
    public double drivePowerLimit;
    public TrcPidController.PidCoefficients tunePidCoeff;

    /**
     * Constructor.
     */
    public Robot()
    {
        super(programName);
    }   //Robot

    /**
     * This function is run when the robot is first started up and should be used for any initialization code.
     */
    @Override
    public void robotInit()
    {
        //
        // Inputs.
        //
        leftDriveStick = new FrcJoystick("leftDriveStick", RobotInfo.JSPORT_LEFT_DRIVESTICK);
        rightDriveStick = new FrcJoystick("rightDriveStick", RobotInfo.JSPORT_RIGHT_DRIVESTICK);
        operatorStick = new FrcJoystick("operatorStick", RobotInfo.JSPORT_OPERATORSTICK);
        //
        // Sensors.
        //
        pdp = new FrcPdp(RobotInfo.CANID_PDP);
        battery = new FrcRobotBattery(pdp);
        if (USE_NAV_X)
        {
            gyro = new FrcAHRSGyro("NavX", SPI.Port.kMXP);
        }
        pressureSensor = new AnalogInput(RobotInfo.AIN_PRESSURE_SENSOR);

        //
        // Vision subsystem.
        //
        if(USE_PIXY_I2C)
        {
            pixy = new PixyVision(
                "PixyCam", this, USE_PIXY_V2, RobotInfo.PIXY_TARGET_SIGNATURE, RobotInfo.PIXY_BRIGHTNESS,
                RobotInfo.PIXY_ORIENTATION, I2C.Port.kMXP, RobotInfo.PIXYCAM_I2C_ADDRESS);
        }

        if (USE_RASPI_VISION)
        {
            vision = new RaspiVision();
        }

        //
        // Miscellaneous subsystems.
        //
        if (USE_TEXT_TO_SPEECH)
        {
            tts = new FrcEmic2TextToSpeech("TextToSpeech", SerialPort.Port.kMXP, 9600);
            tts.setEnabled(true);
            tts.selectVoice(Voice.FrailFrank);
            tts.setVolume(0.72);
        }

        if (USE_MESSAGE_BOARD)
        {
            messageBoard = new FrcI2cLEDPanel("messageBoard", I2C.Port.kOnboard);
        }

        //
        // DriveBase subsystem.
        //
        leftFrontWheel = new FrcCANSparkMax("LeftFrontWheel", RobotInfo.CANID_LEFTFRONTWHEEL, true);
        leftRearWheel = new FrcCANSparkMax("LeftRearWheel", RobotInfo.CANID_LEFTREARWHEEL, true);
        rightFrontWheel = new FrcCANSparkMax("RightFrontWheel", RobotInfo.CANID_RIGHTFRONTWHEEL, true);
        rightRearWheel = new FrcCANSparkMax("RightRearWheel", RobotInfo.CANID_RIGHTREARWHEEL, true);
        pdp.registerEnergyUsed(RobotInfo.PDP_CHANNEL_LEFT_FRONT_WHEEL, "LeftFrontWheel");
        pdp.registerEnergyUsed(RobotInfo.PDP_CHANNEL_LEFT_REAR_WHEEL, "LeftRearWheel");
        pdp.registerEnergyUsed(RobotInfo.PDP_CHANNEL_RIGHT_FRONT_WHEEL, "RightFrontWheel");
        pdp.registerEnergyUsed(RobotInfo.PDP_CHANNEL_RIGHT_REAR_WHEEL, "RightRearWheel");

        //
        // Initialize each drive motor controller.
        //
        leftFrontWheel.setInverted(false);
        leftRearWheel.setInverted(false);
        rightFrontWheel.setInverted(true);
        rightRearWheel.setInverted(true);

        leftFrontWheel.setPositionSensorInverted(false);
        leftRearWheel.setPositionSensorInverted(false);
        rightFrontWheel.setPositionSensorInverted(false);
        rightRearWheel.setPositionSensorInverted(false);

        //
        // Initialize DriveBase subsystem.
        //
        driveBase = new TrcMecanumDriveBase(leftFrontWheel, leftRearWheel, rightFrontWheel, rightRearWheel, gyro);
        driveBase.setPositionScales(RobotInfo.ENCODER_X_INCHES_PER_COUNT, RobotInfo.ENCODER_Y_INCHES_PER_COUNT);

        //
        // Create PID controllers for DriveBase PID drive.
        //
        encoderXPidCtrl = new TrcPidController(
            "encoderXPidCtrl",
            new PidCoefficients(
                RobotInfo.ENCODER_X_KP, RobotInfo.ENCODER_X_KI, RobotInfo.ENCODER_X_KD, RobotInfo.ENCODER_X_KF),
            RobotInfo.ENCODER_X_TOLERANCE,
            driveBase::getXPosition);
        encoderYPidCtrl = new TrcPidController(
            "encoderYPidCtrl",
            new PidCoefficients(
                RobotInfo.ENCODER_Y_KP, RobotInfo.ENCODER_Y_KI, RobotInfo.ENCODER_Y_KD, RobotInfo.ENCODER_Y_KF),
            RobotInfo.ENCODER_Y_TOLERANCE,
            driveBase::getYPosition);
        gyroTurnPidCtrl = new TrcPidController(
            "gyroTurnPidCtrl",
            new PidCoefficients(
                RobotInfo.GYRO_TURN_KP, RobotInfo.GYRO_TURN_KI, RobotInfo.GYRO_TURN_KD, RobotInfo.GYRO_TURN_KF),
            RobotInfo.GYRO_TURN_TOLERANCE,
            driveBase::getHeading);
        gyroTurnPidCtrl.setAbsoluteSetPoint(true);
        pidDrive = new TrcPidDrive("pidDrive", driveBase, encoderXPidCtrl, encoderYPidCtrl, gyroTurnPidCtrl);
        pidDrive.setStallTimeout(RobotInfo.DRIVE_STALL_TIMEOUT);
        pidDrive.setMsgTracer(globalTracer);

        encoderXPidCtrl.setOutputLimit(RobotInfo.DRIVE_MAX_XPID_POWER);
        encoderYPidCtrl.setOutputLimit(RobotInfo.DRIVE_MAX_YPID_POWER);
        gyroTurnPidCtrl.setOutputLimit(RobotInfo.DRIVE_MAX_TURNPID_POWER);

        //
        // Create other hardware subsystems.
        //
        elevator = new Elevator();
        pickup = new Pickup();

        //
        // AutoAssist commands.
        //
        autoDeploy = new CmdAutoDeploy(this);


        //
        // Create Robot Modes.
        //
        setupRobotModes(
            new FrcTeleOp(this),
            new FrcAuto(this),
            new FrcTest(this),
            new FrcDisabled(this));
    }   //robotInit

    public void robotStartMode(RunMode runMode, RunMode prevMode)
    {
        final String funcName = "robotStartMode";

        if (tts != null)
        {
            if (runMode == RunMode.DISABLED_MODE)
            {
                // Robot is safe.
                // Note: "disaibled" is not a typo. It forces the speech board to pronounce it correctly.
                tts.speak("Robot disaibled");
                nextTimeToSpeakInSeconds = TrcUtil.getCurrentTime() + IDLE_PERIOD_SECONDS;
            }
            else
            {
                // Robot is unsafe
                tts.speak("Robot enabled, stand clear");
                nextTimeToSpeakInSeconds = TrcUtil.getCurrentTime() + SPEAK_PERIOD_SECONDS;
            }
        }

        if (runMode != RunMode.DISABLED_MODE)
        {
            openTraceLog(runMode == RunMode.AUTO_MODE? "FrcAuto":
                         runMode == RunMode.TELEOP_MODE? "FrcTeleOp": "FrcTest");
            setTraceLogEnabled(true);

            Date now = new Date();
            globalTracer.traceInfo(funcName, "[%.3f] %s: ***** %s *****",
                Robot.getModeElapsedTime(), now.toString(), runMode);

            pdp.setTaskEnabled(true);
            battery.setEnabled(true);
            setVisionEnabled(true);
            driveBase.resetOdometry();
            targetHeading = 0.0;

            dashboard.clearDisplay();

            if (runMode == RunMode.AUTO_MODE || runMode == RunMode.TEST_MODE)
            {
                driveTime = HalDashboard.getNumber("Test/DriveTime", 5.0);
                drivePower = HalDashboard.getNumber("Test/DrivePower", 0.2);
                driveDistance = HalDashboard.getNumber("Test/DriveDistance", 6.0);
                turnDegrees = HalDashboard.getNumber("Test/TurnDegrees", 90.0);
                drivePowerLimit = HalDashboard.getNumber("Test/DrivePowerLimit", 0.5);
                if (runMode == RunMode.TEST_MODE)
                {
                    tunePidCoeff = new TrcPidController.PidCoefficients(
                        HalDashboard.getNumber("Test/TuneKp", RobotInfo.GYRO_TURN_KP),
                        HalDashboard.getNumber("Test/TuneKi", RobotInfo.GYRO_TURN_KI),
                        HalDashboard.getNumber("Test/TuneKd", RobotInfo.GYRO_TURN_KD),
                        HalDashboard.getNumber("Test/TuneKf", 0.0));
                }
            }
        }
    }   //robotStartMode

    public void robotStopMode(RunMode runMode, RunMode nextMode)
    {
        final String funcName = "robotStopMode";

        if (runMode != RunMode.DISABLED_MODE)
        {
            setVisionEnabled(false);
            pdp.setTaskEnabled(false);
            battery.setEnabled(false);

            for (int i = 0; i < FrcPdp.kPDPChannels; i++)
            {
                String channelName = pdp.getChannelName(i);
                if (channelName != null)
                {
                    globalTracer.traceInfo(
                        funcName, "[PDP-%02d] %s: EnergyUsed=%.3f Wh", i, channelName, pdp.getEnergyUsed(i));
                }
            }

            double totalEnergy = battery.getTotalEnergy();
            globalTracer.traceInfo(
                funcName, "TotalEnergy=%.3fWh (%.2f%%)",
                totalEnergy, totalEnergy*100.0/RobotInfo.BATTERY_CAPACITY_WATT_HOUR);
            setTraceLogEnabled(false);
            closeTraceLog();
        }
    }   //robotStopMode

    public void getFMSInfo()
    {
        eventName = ds.getEventName();
        if (eventName.length() == 0)
        {
            eventName = "Unknown";
        }
        matchType = ds.getMatchType();
        matchNumber = ds.getMatchNumber();
    }

    public void getGameInfo()
    {
        alliance = ds.getAlliance();
        location = ds.getLocation();
        gameSpecificMessage = ds.getGameSpecificMessage();
    }

    public void openTraceLog(String defaultName)
    {
        if (USE_TRACELOG && !traceLogOpened)
        {
            String fileName;

            if (ds.isFMSAttached())
            {
                getFMSInfo();
                fileName = String.format("%s_%s%03d", eventName, matchType, matchNumber);
            }
            else
            {
                fileName = defaultName;
            }

            traceLogOpened = globalTracer.openTraceLog("/home/lvuser/tracelog", fileName);
        }
    }

    public void closeTraceLog()
    {
        if (traceLogOpened)
        {
            globalTracer.closeTraceLog();
            traceLogOpened = false;
        }
    }

    public void setTraceLogEnabled(boolean enabled)
    {
        if (traceLogOpened)
        {
            globalTracer.setTraceLogEnabled(enabled);
        }
    }

    public void setVisionEnabled(boolean enabled)
    {
        final String funcName = "setVisionEnabled";

        if (pixy != null)
        {
            pixy.setEnabled(enabled);
            globalTracer.traceInfo(funcName, "Pixy is %s!", enabled? "enabled": "disabled");
        }
    }   //setVisionEnabled

    public void updateDashboard(RunMode runMode)
    {
        final String funcName = "updateDashboard";
        double currTime = Robot.getModeElapsedTime();

        if (currTime >= nextUpdateTime)
        {
            nextUpdateTime = currTime + DASHBOARD_UPDATE_INTERVAL;

            if (DEBUG_POWER_CONSUMPTION)
            {
                HalDashboard.putNumber("Power/pdpTotalCurrent", pdp.getTotalCurrent());
                HalDashboard.putNumber("Power/totalEnergy", battery.getTotalEnergy());
                HalDashboard.putData("Power/pdpInfo", pdp.getPdpSendable());
                if (runMode == RunMode.TELEOP_MODE)
                {
                    globalTracer.traceInfo(funcName, "[%.3f] Battery: currVoltage=%.2f, lowestVoltage=%.2f",
                        currTime, battery.getVoltage(), battery.getLowestVoltage());
                    globalTracer.traceInfo(
                        funcName, "[%.3f] Total=%.2fA",
                        currTime,
                        pdp.getTotalCurrent());
                }
            }

            if (DEBUG_DRIVE_BASE)
            {
                double xPos = driveBase.getXPosition();
                double yPos = driveBase.getYPosition();
                double heading = driveBase.getHeading();

                HalDashboard.putNumber("DriveBase/xPos", xPos);
                HalDashboard.putNumber("DriveBase/yPos", yPos);
                HalDashboard.putData("DriveBase/heading", gyro.getGyroSendable());

                HalDashboard.putData("DriveBase/Mecanum_Drive", createMecanumDriveInfo());

                //
                // DriveBase debug info.
                //
                double lfEnc = leftFrontWheel.getPosition();
                double rfEnc = rightFrontWheel.getPosition();
                double lrEnc = leftRearWheel.getPosition();
                double rrEnc = rightRearWheel.getPosition();

                dashboard.displayPrintf(8, "DriveBase: lf=%.0f, rf=%.0f, lr=%.0f, rr=%.0f, avg=%.0f",
                    lfEnc, rfEnc, lrEnc, rrEnc, (lfEnc + rfEnc + lrEnc + rrEnc)/4.0);
                dashboard.displayPrintf(9, "DriveBase: X=%.1f, Y=%.1f, Heading=%.1f", xPos, yPos, heading);

                if (DEBUG_PID_DRIVE)
                {
                    encoderXPidCtrl.displayPidInfo(10);
                    encoderYPidCtrl.displayPidInfo(12);
                    gyroTurnPidCtrl.displayPidInfo(14);
                }
            }

            if (DEBUG_SUBSYSTEMS)
            {
                if (DEBUG_PIXY)
                {
                    if (pixy != null && pixy.isEnabled())
                    {
                        if (USE_PIXY_LINE_TARGET)
                        {
                            Vector[] vectors = pixy.getLineVectors();
                            if (vectors == null)
                            {
                                dashboard.displayPrintf(11, "Pixy: line not found");
                            }
                            else
                            {
                                dashboard.displayPrintf(11, "Pixy: %s", vectors[0]);
                            }
                        }
                        else
                        {
                            PixyVision.TargetInfo targetInfo = pixy.getTargetInfo();
                            if (targetInfo == null)
                            {
                                dashboard.displayPrintf(11, "Pixy: Target not found!");
                            }
                            else
                            {
                                dashboard.displayPrintf(11, "Pixy: xDistance=%.1f, yDistance=%.1f, angle=%.1f",
                                    targetInfo.xDistance, targetInfo.yDistance, targetInfo.angle);
                                dashboard.displayPrintf(12, "x=%d, y=%d, width=%d, height=%d",
                                    targetInfo.rect.x, targetInfo.rect.y, targetInfo.rect.width, targetInfo.rect.height);
                            }
                        }
                    }
                }
            }
        }
    }   //updateDashboard

    private Sendable createMecanumDriveInfo()
    {
        return new Sendable() {
            private String name, subsystem;
            @Override
            public String getName() { return name; }
            @Override
            public void setName(String name) { this.name = name;}
            @Override
            public String getSubsystem() { return subsystem; }
            @Override
            public void setSubsystem(String subsystem) { this.subsystem = subsystem; }

            @Override
            public void initSendable(SendableBuilder builder)
            {
                builder.setSmartDashboardType("MecanumDrive");
                builder.addDoubleProperty("Front Left Motor Speed", leftFrontWheel::getPower, null);
                builder.addDoubleProperty("Front Right Motor Speed", rightFrontWheel::getPower, null);
                builder.addDoubleProperty("Rear Left Motor Speed", leftRearWheel::getPower, null);
                builder.addDoubleProperty("Rear Right Motor Speed", rightRearWheel::getPower, null);
            }
        };
    }

    public void announceSafety()
    {
        double currTime = TrcUtil.getCurrentTime();

        if (tts != null && nextTimeToSpeakInSeconds > 0.0 && currTime >= nextTimeToSpeakInSeconds)
        {
            tts.speak("Stand clear");
            nextTimeToSpeakInSeconds = currTime + SPEAK_PERIOD_SECONDS;
        }
    }   //announceSafety

    public void announceIdling()
    {
        double currTime = TrcUtil.getCurrentTime();

        if (tts != null && nextTimeToSpeakInSeconds > 0.0 && currTime >= nextTimeToSpeakInSeconds)
        {
            tts.speak("Robot is idle, please turn off.");
            nextTimeToSpeakInSeconds = currTime + SPEAK_PERIOD_SECONDS;
        }
    }   //announceIdling

    public void traceStateInfo(double elapsedTime, String stateName, double xDistance, double yDistance, double heading)
    {
        final String funcName = "traceStateInfo";

        if (battery != null)
        {
            globalTracer.traceInfo(
                    funcName,
                    "[%5.3f] >>>>> %s: xPos=%6.2f/%6.2f,yPos=%6.2f/%6.2f,heading=%6.1f/%6.1f,volt=%5.2fV(%5.2fV)",
                    elapsedTime, stateName,
                    driveBase.getXPosition(), xDistance, driveBase.getYPosition(), yDistance,
                    driveBase.getHeading(), heading, battery.getVoltage(), battery.getLowestVoltage());
        }
        else
        {
            globalTracer.traceInfo(
                    funcName,
                    "[%5.3f] >>>>> %s: xPos=%6.2f/%6.2f,yPos=%6.2f/%6.2f,heading=%6.1f/%6.1f",
                    elapsedTime, stateName,
                    driveBase.getXPosition(), xDistance, driveBase.getYPosition(), yDistance,
                    driveBase.getHeading(), heading);
        }
   }   //traceStateInfo

    //
    // Getters for sensor data.
    //

    public double getPressure()
    {
        return 50.0*pressureSensor.getVoltage() - 25.0;
    }   //getPressure

    public Double getPixyTargetAngle()
    {
        final String funcName = "getPixyTargetAngle";
        TargetInfo targetInfo = pixy.getTargetInfo();

        if (targetInfo != null)
        {
            globalTracer.traceInfo(funcName, "Found cube: x=%.1f, y=%1.f, angle=%.1f",
                targetInfo.xDistance, targetInfo.yDistance, targetInfo.angle);
        }
        else
        {
            globalTracer.traceInfo(funcName, "Cube not found!");
        }

        return targetInfo != null? targetInfo.angle: null;
    }

    public Double getPixyTargetX()
    {
        final String funcName = "getPixyTargetX";
        TargetInfo targetInfo = pixy.getTargetInfo();

        if (targetInfo != null)
        {
            globalTracer.traceInfo(funcName, "Found cube: x=%.1f, y=%.1f, angle=%.1f",
                targetInfo.xDistance, targetInfo.yDistance, targetInfo.angle);
        }
        else
        {
            globalTracer.traceInfo(funcName, "Cube not found!");
        }

        return targetInfo != null? targetInfo.xDistance: null;
    }

    public Double getPixyTargetY()
    {
        final String funcName = "getPixyTargetY";
        TargetInfo targetInfo = pixy.getTargetInfo();

        if (targetInfo != null)
        {
            globalTracer.traceInfo(funcName, "Found cube: x=%.1f, y=%1.f, angle=%.1f",
                targetInfo.xDistance, targetInfo.yDistance, targetInfo.angle);
        }
        else
        {
            globalTracer.traceInfo(funcName, "Cube not found!");
        }

        return targetInfo != null? targetInfo.yDistance: null;
    }

}   //class Robot
