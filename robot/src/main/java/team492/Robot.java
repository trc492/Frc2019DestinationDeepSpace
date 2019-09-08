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

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
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
import frclib.FrcRemoteVisionProcessor;
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
    public enum DriveMode
    {
        HOLONOMIC_MODE, TANK_MODE, ARCADE_MODE
    } // enum DriveMode

    public static final String programName = "DestinationDeepSpace";

    public static final boolean USE_TRACELOG = true;
    public static final boolean USE_NAV_X = true;
    public static final boolean USE_TEXT_TO_SPEECH = false;
    public static final boolean USE_MESSAGE_BOARD = false;
    public static final boolean USE_GYRO_ASSIST = false;
    public static final boolean USE_VISION_TARGETING = true;
    public static final boolean USE_PIXY_I2C = false;
    public static final boolean USE_PIXY_V2 = false;
    public static final boolean USE_PIXY_LINE_TARGET = false;
    public static final boolean USE_STREAM_CAMERA = true;

    private static final boolean DEBUG_POWER_CONSUMPTION = false;
    private static final boolean DEBUG_DRIVE_BASE = false;
    private static final boolean DEBUG_PID_DRIVE = false;
    private static final boolean DEBUG_SUBSYSTEMS = false;
    private static final boolean DEBUG_PIXY = false;
    private static final boolean DEBUG_VISION_TARGET = false;

    private static final double DASHBOARD_UPDATE_INTERVAL = 0.1;
    private static final double SPEAK_PERIOD_SECONDS = 20.0; // Speaks once every this # of second.
    private static final double IDLE_PERIOD_SECONDS = 300.0;

    public DriverStation ds = DriverStation.getInstance();
    public HalDashboard dashboard = HalDashboard.getInstance();

    public double targetHeading = 0.0;

    public boolean driveInverted = false;

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
    public FrcJoystick buttonPanel = null;
    public FrcJoystick switchPanel = null;
    //
    // Sensors.
    //
    public FrcPdp pdp = null;
    public TrcRobotBattery battery = null;
    public FrcAHRSGyro gyro = null;
    public AnalogInput pressureSensor = null;
    //
    // VisionTargetPipeline subsystem.
    //
    public PixyVision pixy = null;
    public VisionTargeting vision = null;
    //
    // Miscellaneous subsystem.
    //
    public FrcEmic2TextToSpeech tts = null;
    private double nextTimeToSpeakInSeconds = 0.0;  //0 means disabled, no need to speak;
    public FrcI2cLEDPanel messageBoard = null;
    public LEDIndicator indicator;
    //
    // DriveBase subsystem.
    //
    public FrcCANSparkMax leftFrontWheel;
    public FrcCANSparkMax leftRearWheel;
    public FrcCANSparkMax rightFrontWheel;
    public FrcCANSparkMax rightRearWheel;
    public TrcMecanumDriveBase driveBase;
    public DriveMode driveMode;

    public TrcPidController encoderXPidCtrl;
    public TrcPidController encoderYPidCtrl;
    public TrcPidController gyroTurnPidCtrl;
    public TrcPidDrive pidDrive;

    public TrcPidController visionXPidCtrl = null;
    public TrcPidController visionYPidCtrl = null;
    public TrcPidController visionTurnPidCtrl = null;
    public TrcPidDrive visionPidDrive = null;

    //
    // Primary robot subystems
    //
    public Pickup pickup;
    public Elevator elevator;
    public Climber climber;
    public boolean driveClimberWheels = false;
    public boolean actuatorEnabled = false;
    public boolean climbingButDriving = false;

    public TaskAutoAlign autoAlign;
    public TaskHeadingAlign autoHeadingAlign;
    //
    // Define our subsystems for Auto and TeleOp modes.
    //
    public double driveTime;
    public double drivePower;
    public double driveDistance;
    public double turnDegrees;
    public double drivePowerLimit;
    public TrcPidController.PidCoefficients tunePidCoeff;

    private FrcAuto autoMode;

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
        buttonPanel = new FrcJoystick("buttonPanel", RobotInfo.JSPORT_BUTTON_PANEL);
        switchPanel = new FrcJoystick("switchPanel", RobotInfo.JSPORT_SWITCH_PANEL);

        leftDriveStick.setButtonEventTracer(globalTracer);
        rightDriveStick.setButtonEventTracer(globalTracer);
        operatorStick.setButtonEventTracer(globalTracer);
        buttonPanel.setButtonEventTracer(globalTracer);
        switchPanel.setButtonEventTracer(globalTracer);

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
        if (USE_PIXY_I2C)
        {
            pixy = new PixyVision("PixyCam", this, USE_PIXY_V2, RobotInfo.PIXY_TARGET_SIGNATURE,
                RobotInfo.PIXY_BRIGHTNESS, RobotInfo.PIXY_ORIENTATION, I2C.Port.kMXP, RobotInfo.PIXYCAM_I2C_ADDRESS);
        }

        if (USE_VISION_TARGETING)
        {
            vision = new VisionTargeting();
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
        driveInverted = false;
        leftFrontWheel = new FrcCANSparkMax("LeftFrontWheel", RobotInfo.CANID_LEFTFRONTWHEEL, true);
        leftRearWheel = new FrcCANSparkMax("LeftRearWheel", RobotInfo.CANID_LEFTREARWHEEL, true);
        rightFrontWheel = new FrcCANSparkMax("RightFrontWheel", RobotInfo.CANID_RIGHTFRONTWHEEL, true);
        rightRearWheel = new FrcCANSparkMax("RightRearWheel", RobotInfo.CANID_RIGHTREARWHEEL, true);
        setHalfBrakeModeEnabled(true); // Karkeys prefers front coast, back brake

        pdp.registerEnergyUsed(new FrcPdp.Channel(RobotInfo.PDP_CHANNEL_LEFT_FRONT_WHEEL, "LeftFrontWheel"),
            new FrcPdp.Channel(RobotInfo.PDP_CHANNEL_LEFT_REAR_WHEEL, "LeftRearWheel"),
            new FrcPdp.Channel(RobotInfo.PDP_CHANNEL_RIGHT_FRONT_WHEEL, "RightFrontWheel"),
            new FrcPdp.Channel(RobotInfo.PDP_CHANNEL_RIGHT_REAR_WHEEL, "RightRearWheel"));

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

        leftFrontWheel.motor.enableVoltageCompensation(RobotInfo.BATTERY_NOMINAL_VOLTAGE);
        rightFrontWheel.motor.enableVoltageCompensation(RobotInfo.BATTERY_NOMINAL_VOLTAGE);
        leftRearWheel.motor.enableVoltageCompensation(RobotInfo.BATTERY_NOMINAL_VOLTAGE);
        rightRearWheel.motor.enableVoltageCompensation(RobotInfo.BATTERY_NOMINAL_VOLTAGE);

        //
        // Initialize DriveBase subsystem.
        //
        driveBase = new TrcMecanumDriveBase(leftFrontWheel, leftRearWheel, rightFrontWheel, rightRearWheel, gyro);
        driveBase.setPositionScales(RobotInfo.ENCODER_X_INCHES_PER_COUNT, RobotInfo.ENCODER_Y_INCHES_PER_COUNT);
        driveMode = DriveMode.HOLONOMIC_MODE;
        //
        // Create PID controllers for DriveBase PID drive.
        //
        encoderXPidCtrl = new TrcPidController("encoderXPidCtrl",
            new PidCoefficients(RobotInfo.ENCODER_X_KP_SMALL, RobotInfo.ENCODER_X_KI_SMALL,
                RobotInfo.ENCODER_X_KD_SMALL, RobotInfo.ENCODER_X_KF_SMALL), RobotInfo.ENCODER_X_TOLERANCE_SMALL,
            driveBase::getXPosition);
        encoderYPidCtrl = new TrcPidController("encoderYPidCtrl",
            new PidCoefficients(RobotInfo.ENCODER_Y_KP, RobotInfo.ENCODER_Y_KI, RobotInfo.ENCODER_Y_KD,
                RobotInfo.ENCODER_Y_KF), RobotInfo.ENCODER_Y_TOLERANCE, driveBase::getYPosition);
        gyroTurnPidCtrl = new TrcPidController("gyroTurnPidCtrl",
            new PidCoefficients(RobotInfo.GYRO_TURN_KP, RobotInfo.GYRO_TURN_KI, RobotInfo.GYRO_TURN_KD,
                RobotInfo.GYRO_TURN_KF), RobotInfo.GYRO_TURN_TOLERANCE, driveBase::getHeading);
        gyroTurnPidCtrl.setAbsoluteSetPoint(true);
        pidDrive = new TrcPidDrive("pidDrive", driveBase, encoderXPidCtrl, encoderYPidCtrl, gyroTurnPidCtrl);
        pidDrive.setStallTimeout(RobotInfo.DRIVE_STALL_TIMEOUT);
        pidDrive.setMsgTracer(globalTracer);

        encoderXPidCtrl.setOutputLimit(RobotInfo.DRIVE_MAX_XPID_POWER);
        encoderYPidCtrl.setOutputLimit(RobotInfo.DRIVE_MAX_YPID_POWER);
        gyroTurnPidCtrl.setOutputLimit(RobotInfo.DRIVE_MAX_TURNPID_POWER);

        encoderXPidCtrl.setRampRate(RobotInfo.DRIVE_MAX_XPID_RAMP_RATE);
        encoderYPidCtrl.setRampRate(RobotInfo.DRIVE_MAX_YPID_RAMP_RATE);
        gyroTurnPidCtrl.setRampRate(RobotInfo.DRIVE_MAX_TURNPID_RAMP_RATE);

        if (USE_VISION_TARGETING)
        {
            visionXPidCtrl = new TrcPidController("visionXPidCtrl",
                new PidCoefficients(RobotInfo.VISION_X_KP, RobotInfo.VISION_X_KI, RobotInfo.VISION_X_KD),
                RobotInfo.VISION_X_TOLERANCE, this::getVisionX);
            visionYPidCtrl = new TrcPidController("visionYPidCtrl",
                new PidCoefficients(RobotInfo.VISION_Y_KP, RobotInfo.VISION_Y_KI, RobotInfo.VISION_Y_KD),
                RobotInfo.VISION_Y_TOLERANCE, this::getVisionY);
            visionTurnPidCtrl = new TrcPidController("visionTurnPidCtrl",
                new PidCoefficients(RobotInfo.VISION_TURN_KP, RobotInfo.VISION_TURN_KI, RobotInfo.VISION_TURN_KD),
                RobotInfo.VISION_TURN_TOLERANCE, this::getVisionYaw);
            visionXPidCtrl.setInverted(true);
            visionXPidCtrl.setAbsoluteSetPoint(true);
            visionYPidCtrl.setInverted(true);
            visionYPidCtrl.setAbsoluteSetPoint(true);
            visionTurnPidCtrl.setInverted(true);
            visionTurnPidCtrl.setAbsoluteSetPoint(true);
            visionPidDrive = new TrcPidDrive("visionPidDrive", driveBase, visionXPidCtrl, visionYPidCtrl,
                visionTurnPidCtrl);
            visionPidDrive.setMsgTracer(globalTracer);
        }

        //
        // Create other hardware subsystems.
        //
        elevator = new Elevator(this);
        pickup = new Pickup(this);
        climber = new Climber(this);
        indicator = new LEDIndicator(this);

        if (USE_STREAM_CAMERA)
        {
            UsbCamera camera = CameraServer.getInstance().startAutomaticCapture("DriverDisplay", 0);
            camera.setResolution(160, 120);
        }

        //
        // AutoAssist commands.
        //
        autoAlign = new TaskAutoAlign(this);
        autoHeadingAlign = new TaskHeadingAlign(this);

        //
        // Create Robot Modes.
        //
        autoMode = new FrcAuto(this);
        setupRobotModes(new FrcTeleOp(this), autoMode, new FrcTest(this), new FrcDisabled(this));

        pdp.registerEnergyUsedForAllUnregisteredChannels();
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
            openTraceLog(
                runMode == RunMode.AUTO_MODE ? "FrcAuto" : runMode == RunMode.TELEOP_MODE ? "FrcTeleOp" : "FrcTest");
            setTraceLogEnabled(true);

            Date now = new Date();
            globalTracer
                .traceInfo(funcName, "[%.3f] %s: ***** %s *****", Robot.getModeElapsedTime(), now.toString(), runMode);

            driveInverted = false;
            climbingButDriving = false;

            pdp.setTaskEnabled(true);
            battery.setEnabled(true);
            setVisionEnabled(true);
            driveBase.resetOdometry(true, false);
            driveBase.setOdometryEnabled(true);
            targetHeading = 0.0;

            dashboard.clearDisplay();

            indicator.reset();

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
            globalTracer.traceInfo(funcName, "mode=%s,heading=%.1f", runMode.name(), driveBase.getHeading());
            driveBase.setOdometryEnabled(false);
            setVisionEnabled(false);
            cancelAllAuto();
            battery.setEnabled(false);
            pdp.setTaskEnabled(false);

            for (int i = 0; i < FrcPdp.kPDPChannels; i++)
            {
                String channelName = pdp.getChannelName(i);
                if (channelName != null)
                {
                    globalTracer
                        .traceInfo(funcName, "[PDP-%02d] %s: EnergyUsed=%.3f Wh", i, channelName, pdp.getEnergyUsed(i));
                }
            }

            double totalEnergy = battery.getTotalEnergy();
            globalTracer.traceInfo(funcName, "TotalEnergy=%.3fWh (%.2f%%)", totalEnergy,
                totalEnergy * 100.0 / RobotInfo.BATTERY_CAPACITY_WATT_HOUR);
            setTraceLogEnabled(false);
            closeTraceLog();

            indicator.reset();
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

    public void enableSmallGains()
    {
        encoderXPidCtrl.setPidCoefficients(
            new PidCoefficients(RobotInfo.ENCODER_X_KP_SMALL, RobotInfo.ENCODER_X_KI_SMALL,
                RobotInfo.ENCODER_X_KD_SMALL, RobotInfo.ENCODER_X_KF_SMALL));
        encoderXPidCtrl.setTargetTolerance(RobotInfo.ENCODER_X_TOLERANCE_SMALL);
        gyroTurnPidCtrl.setPidCoefficients(
            new PidCoefficients(RobotInfo.GYRO_TURN_KP_SMALL, RobotInfo.GYRO_TURN_KI_SMALL,
                RobotInfo.GYRO_TURN_KD_SMALL));
        gyroTurnPidCtrl.setTargetTolerance(RobotInfo.GYRO_TURN_TOLERANCE_SMALL);
    }

    public void enableBigGains()
    {
        encoderXPidCtrl.setPidCoefficients(
            new PidCoefficients(RobotInfo.ENCODER_X_KP, RobotInfo.ENCODER_X_KI, RobotInfo.ENCODER_X_KD,
                RobotInfo.ENCODER_X_KF));
        encoderXPidCtrl.setTargetTolerance(RobotInfo.ENCODER_X_TOLERANCE);
        gyroTurnPidCtrl.setPidCoefficients(
            new PidCoefficients(RobotInfo.GYRO_TURN_KP, RobotInfo.GYRO_TURN_KI, RobotInfo.GYRO_TURN_KD));
        gyroTurnPidCtrl.setTargetTolerance(RobotInfo.GYRO_TURN_TOLERANCE);
    }

    /**
     * Stops the lower level subsystems. This does NOT stop any auto/auto assist commands.
     */
    public void stopSubsystems()
    {
        pidDrive.cancel();
        driveBase.stop();
        elevator.setPower(0.0);
        pickup.setPitchPower(0.0);
        pickup.setPickupPower(0.0);
    }

    public void setVisionEnabled(boolean enabled)
    {
        final String funcName = "setVisionEnabled";

        if (pixy != null)
        {
            pixy.setEnabled(enabled);
            globalTracer.traceInfo(funcName, "Pixy is %s!", enabled ? "enabled" : "disabled");
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
                    globalTracer.traceInfo(funcName, "[%.3f] Battery: currVoltage=%.2f, lowestVoltage=%.2f", currTime,
                        battery.getVoltage(), battery.getLowestVoltage());
                    globalTracer.traceInfo(funcName, "[%.3f] Total=%.2fA", currTime, pdp.getTotalCurrent());
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

                dashboard
                    .displayPrintf(8, "DriveBase: lf=%.0f, rf=%.0f, lr=%.0f, rr=%.0f, avg=%.0f", lfEnc, rfEnc, lrEnc,
                        rrEnc, (lfEnc + rfEnc + lrEnc + rrEnc) / 4.0);
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
                            Vector vector = pixy.getLineVector();
                            if (vector == null)
                            {
                                dashboard.displayPrintf(11, "Pixy: line not found");
                            }
                            else
                            {
                                dashboard.displayPrintf(11, "Pixy: %s", vector);
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
                                dashboard.displayPrintf(12, "x=%d, y=%d, width=%d, height=%d", targetInfo.rect.x,
                                    targetInfo.rect.y, targetInfo.rect.width, targetInfo.rect.height);
                            }
                        }
                    }
                }

                if (DEBUG_VISION_TARGET && vision != null)
                {
                    FrcRemoteVisionProcessor.RelativePose pose = vision.getLastPose();
                    if (pose != null)
                    {
                        dashboard.displayPrintf(13, "VisionTarget: x=%.1f,y=%.1f,objectYaw=%.1f", pose.x, pose.y,
                            pose.objectYaw);
                    }
                    else
                    {
                        dashboard.displayPrintf(13, "VisionTarget: No target found!");
                    }
                }
            }
        }
    }   //updateDashboard

    private Sendable createMecanumDriveInfo()
    {
        return new Sendable()
        {
            private String name, subsystem;

            @Override
            public String getName()
            {
                return name;
            }

            @Override
            public void setName(String name)
            {
                this.name = name;
            }

            @Override
            public String getSubsystem()
            {
                return subsystem;
            }

            @Override
            public void setSubsystem(String subsystem)
            {
                this.subsystem = subsystem;
            }

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

    /**
     * Checks if any auto processes are running, be it auto mode or auto assist, etc.
     *
     * @return True if any auto processes are active, false otherwise.
     */
    public boolean isAutoActive()
    {
        return autoMode.isAutoActive() || autoAlign.isActive() || climber.isActive() || autoHeadingAlign.isActive();
    }

    public void cancelAllAuto()
    {
        if (autoMode.isAutoActive())
        {
            autoMode.cancel();
        }

        if (autoAlign.isActive())
        {
            autoAlign.cancel();
        }

        if (climber.isActive())
        {
            climber.cancel();
        }

        if (autoHeadingAlign.isActive())
        {
            autoHeadingAlign.cancel();
        }
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
            globalTracer.traceInfo(funcName,
                "[%5.3f] >>>>> %s: xPos=%6.2f/%6.2f,yPos=%6.2f/%6.2f,heading=%6.1f/%6.1f,volt=%5.2fV(%5.2fV)",
                elapsedTime, stateName, driveBase.getXPosition(), xDistance, driveBase.getYPosition(), yDistance,
                driveBase.getHeading(), heading, battery.getVoltage(), battery.getLowestVoltage());
        }
        else
        {
            globalTracer.traceInfo(funcName, "[%5.3f] >>>>> %s: xPos=%6.2f/%6.2f,yPos=%6.2f/%6.2f,heading=%6.1f/%6.1f",
                elapsedTime, stateName, driveBase.getXPosition(), xDistance, driveBase.getYPosition(), yDistance,
                driveBase.getHeading(), heading);
        }
    }   //traceStateInfo

    public void setHalfBrakeModeEnabled(boolean enabled)
    {
        if (enabled)
        {
            leftFrontWheel.setBrakeModeEnabled(driveInverted);
            rightFrontWheel.setBrakeModeEnabled(driveInverted);
            leftRearWheel.setBrakeModeEnabled(!driveInverted);
            rightRearWheel.setBrakeModeEnabled(!driveInverted);
        }
        else
        {
            leftFrontWheel.setBrakeModeEnabled(true);
            rightFrontWheel.setBrakeModeEnabled(true);
            leftRearWheel.setBrakeModeEnabled(true);
            rightRearWheel.setBrakeModeEnabled(true);
        }
    }

    //
    // Getters for sensor data.
    //

    public double getPressure()
    {
        return (pressureSensor.getVoltage() - 0.5) * 50.0;
        //        return pressureSensor.getScaledValue();
    }   //getPressure

    public Double getPixyTargetAngle()
    {
        final String funcName = "getPixyTargetAngle";
        TargetInfo targetInfo = pixy.getTargetInfo();

        if (targetInfo != null)
        {
            globalTracer.traceInfo(funcName, "Found cube: x=%.1f, y=%1.f, angle=%.1f", targetInfo.xDistance,
                targetInfo.yDistance, targetInfo.angle);
        }
        else
        {
            globalTracer.traceInfo(funcName, "Cube not found!");
        }

        return targetInfo != null ? targetInfo.angle : null;
    }

    public Double getPixyTargetX()
    {
        final String funcName = "getPixyTargetX";
        TargetInfo targetInfo = pixy.getTargetInfo();

        if (targetInfo != null)
        {
            globalTracer.traceInfo(funcName, "Found cube: x=%.1f, y=%.1f, angle=%.1f", targetInfo.xDistance,
                targetInfo.yDistance, targetInfo.angle);
        }
        else
        {
            globalTracer.traceInfo(funcName, "Cube not found!");
        }

        return targetInfo != null ? targetInfo.xDistance : null;
    }

    public Double getPixyTargetY()
    {
        final String funcName = "getPixyTargetY";
        TargetInfo targetInfo = pixy.getTargetInfo();

        if (targetInfo != null)
        {
            globalTracer.traceInfo(funcName, "Found cube: x=%.1f, y=%1.f, angle=%.1f", targetInfo.xDistance,
                targetInfo.yDistance, targetInfo.angle);
        }
        else
        {
            globalTracer.traceInfo(funcName, "Cube not found!");
        }

        return targetInfo != null ? targetInfo.yDistance : null;
    }

    public double getVisionX()
    {
        FrcRemoteVisionProcessor.RelativePose pose = vision.getLastPose();
        if (pose != null)
        {
            return pose.x;
        }
        return 0.0;
    }

    public double getVisionY()
    {
        FrcRemoteVisionProcessor.RelativePose pose = vision.getLastPose();
        if (pose != null)
        {
            double outputLimit = Math.abs(rightDriveStick.getYWithDeadband(true));
            visionYPidCtrl.setOutputRange(-outputLimit, outputLimit);
            return pose.y;
        }
        return 0.0;
        // Alternate implementation:
        // This implementation will allow the joystick Y to fully control the Y direction of the PID drive.
        // So vision target has nothing to do with the Y direction.
        // return -rightDriveStick.getYWithDeadband(true)/RobotInfo.VISION_Y_KP;
    }

    public double getVisionYaw()
    {
        FrcRemoteVisionProcessor.RelativePose pose = vision.getLastPose();
        if (pose != null)
        {
            return pose.objectYaw;
        }
        return 0.0;
    }

}   //class Robot
