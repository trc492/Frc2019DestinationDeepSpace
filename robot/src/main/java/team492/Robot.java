/*
 * Copyright (c) 2017 Titan Robotics Club (http://www.titanrobotics.com)
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

import org.opencv.core.Rect;

import com.ctre.CANTalon.FeedbackDevice;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.Relay.Value;
import frclib.FrcAHRSGyro;
import frclib.FrcCANTalon;
import frclib.FrcChoiceMenu;
import frclib.FrcEmic2TextToSpeech;
import frclib.FrcFaceDetector;
import frclib.FrcGyro;
import frclib.FrcPneumatic;
import frclib.FrcRobotBase;
import frclib.FrcRobotBattery;
import hallib.HalDashboard;
import team492.PixyVision.TargetInfo;
import trclib.TrcDbgTrace;
import trclib.TrcDriveBase;
import trclib.TrcEmic2TextToSpeech.Voice;
import trclib.TrcGyro;
import trclib.TrcPidController;
import trclib.TrcPidDrive;
import trclib.TrcRobotBattery;
import trclib.TrcUtil;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TrcRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the
 * resource directory.
 */
public class Robot extends FrcRobotBase implements TrcPidController.PidInput
{
    public static final String programName = "FirstSteamWorks";
    private static final String moduleName = "Robot";

    public static final boolean USE_TRACELOG = true;
    public static final boolean USE_NAV_X = true;
    public static final boolean USE_SPI_GYRO = false;
    public static final boolean USE_ANALOG_GYRO = false;
    public static final boolean USE_GRIP_VISION = false;
    public static final boolean USE_AXIS_CAMERA = false;
    public static final boolean USE_FACE_DETECTOR = false;
    public static final boolean USE_FRONT_PIXY = true;
    public static final boolean USE_FRONT_PIXY_UART = false;
    public static final boolean USE_REAR_PIXY = false;
    public static final boolean USE_TEXT_TO_SPEECH = false;
    public static final boolean USE_ACCELEROMETER = false;

    private static final boolean DEBUG_DRIVE_BASE = false;
    private static final boolean DEBUG_PID_DRIVE = false;
    private static final boolean DEBUG_GRIP_VISION = false;
    private static final boolean DEBUG_FACE_DETECTION = false;
    private static final boolean DEBUG_WINCH = true;
    private static final boolean DEBUG_PIXY = true;
    private static final double DASHBOARD_UPDATE_INTERVAL = 0.1;

    public static enum MatchType
    {
        Practice,
        Qualification,
        QuarterFinal,
        SemiFinal,
        Final
    }   //enum MatchType

    public static enum Alliance
    {
        RED_ALLIANCE,
        BLUE_ALLIANCE
    }   //enum Alliance

    public HalDashboard dashboard = HalDashboard.getInstance();
    public TrcDbgTrace tracer = TrcDbgTrace.getGlobalTracer();

    public double targetHeading = 0.0;

    private double nextUpdateTime = TrcUtil.getCurrentTime();

    //
    // Sensors.
    //
    public TrcRobotBattery battery = null;
    public TrcGyro gyro = null;
    public AnalogInput pressureSensor = null;
    public AnalogInput ultrasonicSensor = null;
    private double lastUltrasonicDistance = 0.0;

    //
    // VisionTarget subsystem.
    //
    public GripVision gripVision = null;
    public FrcFaceDetector faceDetector = null;
    public PixyVision frontPixy = null;
    public PixyVision rearPixy = null;

    //
    // Sound subsystem.
    //
    public FrcEmic2TextToSpeech tts = null;

    //
    // DriveBase subsystem.
    //
    public FrcCANTalon leftFrontWheel;
    public FrcCANTalon leftRearWheel;
    public FrcCANTalon rightFrontWheel;
    public FrcCANTalon rightRearWheel;
    public TrcDriveBase driveBase;

    public TrcPidController encoderXPidCtrl;
    public TrcPidController encoderYPidCtrl;
    public TrcPidController gyroTurnPidCtrl;
    public TrcPidDrive pidDrive;

    public TrcPidController sonarDrivePidCtrl;
    public TrcPidController visionTurnPidCtrl;
    public TrcPidDrive visionPidDrive;
    public TrcPidDrive sonarPidDrive;
    public TrcPidDrive visionPidTurn;

    //
    // Define our subsystems for Auto and TeleOp modes.
    //
    public Relay ringLightsPower;
    public Relay flashLightsPower;
    public FrcPneumatic mailbox;
    public GearPickup gearPickup;
    public Winch winch;

    //
    // Menus.
    //
    public FrcChoiceMenu<MatchType> matchTypeMenu;
    public FrcChoiceMenu<Alliance> allianceMenu;

    public MatchType matchType;
    public int matchNumber;
    public Alliance alliance;
    public double driveTime;
    public double drivePower;
    public double driveDistance;
    public double drivePowerLimit;
    public double turnDegrees;
    public double ultrasonicTarget;
    public double visionTurnTarget;
    public double tuneKp;
    public double tuneKi;
    public double tuneKd;

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
        // Sensors.
        //
        battery = new FrcRobotBattery(RobotInfo.CANID_PDP);
        if (USE_NAV_X)
        {
            gyro = new FrcAHRSGyro("NavX", SPI.Port.kMXP);
        }
        else if (USE_SPI_GYRO)
        {
            gyro = new FrcGyro("ADXRS450", new ADXRS450_Gyro());
        }
        else if (USE_ANALOG_GYRO)
        {
            gyro = new FrcGyro("AnalogGyro", new AnalogGyro(RobotInfo.AIN_ANALOG_GYRO));
        }
        pressureSensor = new AnalogInput(RobotInfo.AIN_PRESSURE_SENSOR);
        ultrasonicSensor = new AnalogInput(RobotInfo.AIN_ULTRASONIC_SENSOR);

        //
        // VisionTarget subsystem.
        //
        if (USE_GRIP_VISION)
        {
            CvSink videoIn;
            CvSource videoOut;

            if (USE_AXIS_CAMERA)
            {
                AxisCamera axisCam = CameraServer.getInstance().addAxisCamera("axis-camera.local");
                axisCam.setResolution(RobotInfo.CAM_WIDTH, RobotInfo.CAM_HEIGHT);
                axisCam.setFPS(RobotInfo.CAM_FRAME_RATE);
                axisCam.setBrightness(RobotInfo.CAM_BRIGHTNESS);
                videoIn = CameraServer.getInstance().getVideo(axisCam);
            }
            else
            {
                //
                // Use USB camera.
                //
                UsbCamera cam0 = CameraServer.getInstance().startAutomaticCapture("cam0", 0);
                cam0.setResolution(RobotInfo.CAM_WIDTH, RobotInfo.CAM_HEIGHT);
                cam0.setFPS(RobotInfo.CAM_FRAME_RATE);
                cam0.setBrightness(RobotInfo.CAM_BRIGHTNESS);
                videoIn = CameraServer.getInstance().getVideo(cam0);
            }
            videoOut = CameraServer.getInstance().putVideo("VisionTarget", RobotInfo.CAM_WIDTH, RobotInfo.CAM_HEIGHT);

            gripVision = new GripVision("GripVision", videoIn, videoOut);
        }
        else if (USE_FACE_DETECTOR)
        {
            UsbCamera cam0 = CameraServer.getInstance().startAutomaticCapture("cam0", 0);
            cam0.setResolution(RobotInfo.CAM_WIDTH, RobotInfo.CAM_HEIGHT);
            cam0.setFPS(RobotInfo.CAM_FRAME_RATE);
            CvSink videoIn = CameraServer.getInstance().getVideo(cam0);
            CvSource videoOut =
                CameraServer.getInstance().putVideo("FaceDetector", RobotInfo.CAM_WIDTH, RobotInfo.CAM_HEIGHT);

            faceDetector = new FrcFaceDetector(
                "FaceDetector", "/home/lvuser/cascade-files/haarcascade_frontalface_alt.xml", videoIn, videoOut);
        }
        else
        {
            if (USE_FRONT_PIXY)
            {
                if (USE_FRONT_PIXY_UART)
                {
                    frontPixy = new PixyVision(
                        "FrontPixy", this, RobotInfo.PIXY_LIFT_SIGNATURE, RobotInfo.PIXY_FRONT_BRIGHTNESS,
                        RobotInfo.PIXY_FRONT_ORIENTATION, SerialPort.Port.kMXP);
                }
                else
                {
                    frontPixy = new PixyVision(
                        "FrontPixy", this, RobotInfo.PIXY_LIFT_SIGNATURE, RobotInfo.PIXY_FRONT_BRIGHTNESS,
                        RobotInfo.PIXY_FRONT_ORIENTATION, I2C.Port.kMXP, RobotInfo.PIXYCAM_FRONT_I2C_ADDRESS);
                }
            }

            if (USE_REAR_PIXY)
            {
                rearPixy = new PixyVision(
                    "RearPixy", this, RobotInfo.PIXY_GEAR_SIGNATURE, RobotInfo.PIXY_REAR_BRIGHTNESS,
                    RobotInfo.PIXY_REAR_ORIENTATION, I2C.Port.kMXP, RobotInfo.PIXYCAM_REAR_I2C_ADDRESS);
            }
        }

        //
        // Sound subsystem.
        //
        if (USE_TEXT_TO_SPEECH)
        {
            tts = new FrcEmic2TextToSpeech("TextToSpeech", SerialPort.Port.kMXP, 9600);
            tts.setEnabled(true);
            tts.selectVoice(Voice.FrailFrank);
            tts.setVolume(1.0);
        }

        //
        // DriveBase subsystem.
        //
        leftFrontWheel = new FrcCANTalon("LeftFrontWheel", RobotInfo.CANID_LEFTFRONTWHEEL);
        leftRearWheel = new FrcCANTalon("LeftRearWheel", RobotInfo.CANID_LEFTREARWHEEL);
        rightFrontWheel = new FrcCANTalon("RightFrontWheel", RobotInfo.CANID_RIGHTFRONTWHEEL);
        rightRearWheel = new FrcCANTalon("RightRearWheel", RobotInfo.CANID_RIGHTREARWHEEL);

        //
        // Initialize each drive motor controller.
        //
        leftFrontWheel.setInverted(false);
        leftRearWheel.setInverted(false);
        rightFrontWheel.setInverted(true);
        rightRearWheel.setInverted(true);

        leftFrontWheel.motor.enableLimitSwitch(false, false);
        leftRearWheel.motor.enableLimitSwitch(false, false);
        rightFrontWheel.motor.enableLimitSwitch(false, false);
        rightRearWheel.motor.enableLimitSwitch(false, false);

        leftFrontWheel.setPositionSensorInverted(false);
        leftRearWheel.setPositionSensorInverted(false);
        rightFrontWheel.setPositionSensorInverted(true);
        rightRearWheel.setPositionSensorInverted(true);

        leftFrontWheel.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        leftRearWheel.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        rightFrontWheel.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        rightRearWheel.setFeedbackDevice(FeedbackDevice.QuadEncoder);

        //
        // Initialize DriveBase subsystem.
        //
        driveBase = new TrcDriveBase(leftFrontWheel, leftRearWheel, rightFrontWheel, rightRearWheel, gyro);
        driveBase.setXPositionScale(RobotInfo.ENCODER_X_INCHES_PER_COUNT);
        driveBase.setYPositionScale(RobotInfo.ENCODER_Y_INCHES_PER_COUNT);

        //
        // Create PID controllers for DriveBase PID drive.
        //
        encoderXPidCtrl = new TrcPidController(
            "encoderXPidCtrl",
            RobotInfo.ENCODER_X_KP, RobotInfo.ENCODER_X_KI, RobotInfo.ENCODER_X_KD, RobotInfo.ENCODER_X_KF,
            RobotInfo.ENCODER_X_TOLERANCE, RobotInfo.ENCODER_X_SETTLING, this);
        encoderYPidCtrl = new TrcPidController(
            "encoderYPidCtrl",
            RobotInfo.ENCODER_Y_KP, RobotInfo.ENCODER_Y_KI, RobotInfo.ENCODER_Y_KD, RobotInfo.ENCODER_Y_KF,
            RobotInfo.ENCODER_Y_TOLERANCE, RobotInfo.ENCODER_Y_SETTLING, this);
        gyroTurnPidCtrl = new TrcPidController(
            "gyroTurnPidCtrl",
            RobotInfo.GYRO_TURN_KP, RobotInfo.GYRO_TURN_KI, RobotInfo.GYRO_TURN_KD, RobotInfo.GYRO_TURN_KF,
            RobotInfo.GYRO_TURN_TOLERANCE, RobotInfo.GYRO_TURN_SETTLING, this);
        gyroTurnPidCtrl.setAbsoluteSetPoint(true);
        pidDrive = new TrcPidDrive("pidDrive", driveBase, encoderXPidCtrl, encoderYPidCtrl, gyroTurnPidCtrl);
        pidDrive.setStallTimeout(RobotInfo.DRIVE_STALL_TIMEOUT);
        pidDrive.setMsgTracer(tracer);

        sonarDrivePidCtrl = new TrcPidController(
            "sonarDrivePidCtrl",
            RobotInfo.SONAR_KP, RobotInfo.SONAR_KI, RobotInfo.SONAR_KD, RobotInfo.SONAR_KF,
            RobotInfo.SONAR_TOLERANCE, RobotInfo.SONAR_SETTLING, this);
        sonarDrivePidCtrl.setAbsoluteSetPoint(true);
        sonarDrivePidCtrl.setInverted(true);
        visionTurnPidCtrl = new TrcPidController(
            "visionTurnPidCtrl",
            RobotInfo.VISION_TURN_KP, RobotInfo.VISION_TURN_KI, RobotInfo.VISION_TURN_KD, RobotInfo.VISION_TURN_KF,
            RobotInfo.VISION_TURN_TOLERANCE, RobotInfo.VISION_TURN_SETTLING, this);
        visionTurnPidCtrl.setInverted(true);
        visionTurnPidCtrl.setAbsoluteSetPoint(true);
        visionPidDrive = new TrcPidDrive("visionPidDrive", driveBase, null, sonarDrivePidCtrl, visionTurnPidCtrl);
        visionPidDrive.setStallTimeout(RobotInfo.DRIVE_STALL_TIMEOUT);
        visionPidDrive.setMsgTracer(tracer);
        sonarPidDrive = new TrcPidDrive("sonarPidDrive", driveBase, null, sonarDrivePidCtrl, null);
        sonarPidDrive.setMsgTracer(tracer);
        visionPidTurn = new TrcPidDrive("cameraPidDrive", driveBase, null, null, gyroTurnPidCtrl);
        visionPidTurn.setMsgTracer(tracer);

        //
        // Create other subsystems.
        //
        ringLightsPower = new Relay(RobotInfo.RELAY_RINGLIGHT_POWER);
        ringLightsPower.setDirection(Direction.kForward);
        flashLightsPower = new Relay(RobotInfo.RELAY_FLASHLIGHT_POWER);
        flashLightsPower.setDirection(Direction.kForward);
        mailbox = new FrcPneumatic(
            "Mailbox", RobotInfo.CANID_PCM1, RobotInfo.SOL_MAILBOX_EXTEND, RobotInfo.SOL_MAILBOX_RETRACT);
        gearPickup = new GearPickup();
        winch = new Winch();

        //
        // Create Global Menus (can be used in all modes).
        //
        matchTypeMenu = new FrcChoiceMenu<>("Match Type");
        allianceMenu = new FrcChoiceMenu<>("Alliance");

        //
        // Populate Global Menus.
        //
        matchTypeMenu.addChoice("Practice", MatchType.Practice, true);
        matchTypeMenu.addChoice("Qualification", MatchType.Qualification, false);
        matchTypeMenu.addChoice("Quater-final", MatchType.QuarterFinal, false);
        matchTypeMenu.addChoice("Semi-final", MatchType.SemiFinal, false);
        matchTypeMenu.addChoice("Final", MatchType.Final, false);

        allianceMenu.addChoice("Red", Alliance.RED_ALLIANCE, true);
        allianceMenu.addChoice("Blue", Alliance.BLUE_ALLIANCE, false);

        //
        // Robot Modes.
        //
        setupRobotModes(new FrcTeleOp(this), new FrcAuto(this), new FrcTest(this), null);
    }   //robotInit

    public void robotStartMode()
    {
        battery.setEnabled(true);
        //
        // Retrieve Global Choices.
        //
        matchType = matchTypeMenu.getCurrentChoiceObject();
        matchNumber = (int)HalDashboard.getNumber("MatchNumber", 0.0);
        alliance = allianceMenu.getCurrentChoiceObject();
        driveTime = HalDashboard.getNumber("DriveTime", 5.0);
        drivePower = HalDashboard.getNumber("DrivePower", 0.2);
        driveDistance = HalDashboard.getNumber("DriveDistance", 6.0);
        drivePowerLimit = HalDashboard.getNumber("DrivePowerLimit", 0.5);
        turnDegrees = HalDashboard.getNumber("TurnDegrees", 90.0);
        ultrasonicTarget = HalDashboard.getNumber("UltrasonicTarget", 7.0);
        visionTurnTarget = HalDashboard.getNumber("VisionTurnTarget", 0.0);
        tuneKp = HalDashboard.getNumber("TuneKp", RobotInfo.GYRO_TURN_KP);
        tuneKi = HalDashboard.getNumber("TuneKi", RobotInfo.GYRO_TURN_KI);
        tuneKd = HalDashboard.getNumber("TuneKd", RobotInfo.GYRO_TURN_KD);
    }   //robotStartMode

    public void robotStopMode()
    {
        driveBase.stop();
        battery.setEnabled(false);
    }   //robotStopMode

    public double getPressure()
    {
        return 50.0*pressureSensor.getVoltage() - 25.0;
    }   //getPressure

    public double getUltrasonicDistance()
    {
        return ultrasonicSensor.getVoltage()/0.0098;
    }   //getUltrasonicDistance

    public void setVisionEnabled(boolean enabled)
    {
        if (gripVision != null)
        {
            ringLightsPower.set(enabled? Value.kOn: Value.kOff);
            gripVision.setVideoOutEnabled(enabled);
            gripVision.setEnabled(enabled);
            tracer.traceInfo("Vision", "Grip Vision is %s!", enabled? "enabled": "disabled");
        }
        else if (faceDetector != null)
        {
            faceDetector.setVideoOutEnabled(enabled);
            faceDetector.setEnabled(enabled);
            tracer.traceInfo("Vision", "Face Detector is %s!", enabled? "enabled": "disabled");
        }
        else
        {
            ringLightsPower.set(enabled? Value.kOn: Value.kOff);

            if (frontPixy != null)
            {
                frontPixy.setEnabled(enabled);
                tracer.traceInfo("Vision", "Front pixy is %s!", enabled? "enabled": "disabled");
            }

            if (rearPixy != null)
            {
                rearPixy.setEnabled(enabled);
                tracer.traceInfo("Vision", "Rear pixy is %s!", enabled? "enabled": "disabled");
            }
        }
    }   //setVisionEnabled

    public void updateDashboard()
    {
        double currTime = TrcUtil.getCurrentTime();

        if (currTime >= nextUpdateTime)
        {
            nextUpdateTime = currTime + DASHBOARD_UPDATE_INTERVAL;

            if (DEBUG_DRIVE_BASE)
            {
                //
                // DriveBase debug info.
                //
                dashboard.displayPrintf(8, "DriveBase: lf=%.0f, rf=%.0f, lr=%.0f, rr=%.0f",
                    leftFrontWheel.getPosition(), rightFrontWheel.getPosition(),
                    leftRearWheel.getPosition(), rightRearWheel.getPosition());
                dashboard.displayPrintf(9, "DriveBase: X=%.1f, Y=%.1f, Heading=%.1f",
                    driveBase.getXPosition(), driveBase.getYPosition(), driveBase.getHeading());

                if (DEBUG_PID_DRIVE)
                {
                    encoderXPidCtrl.displayPidInfo(10);
                    encoderYPidCtrl.displayPidInfo(12);
                    gyroTurnPidCtrl.displayPidInfo(14);
                }
                HalDashboard.putNumber("DriveBase.X", driveBase.getXPosition());
                HalDashboard.putNumber("DriveBase.Y", driveBase.getYPosition());
                HalDashboard.putNumber("DriveBase.Heading", driveBase.getHeading());
            }

            if (DEBUG_GRIP_VISION)
            {
                if (gripVision != null && gripVision.isEnabled())
                {
                    Rect[] targetRects = gripVision.getObjectRects();
                    tracer.traceInfo("GripVision", "Target is %s (%d)",
                        targetRects == null? "not found": "found", targetRects == null? 0: targetRects.length);
                    if (targetRects != null)
                    {
                        for (int i = 0; i < targetRects.length; i++)
                        {
                            dashboard.displayPrintf(8 + i, "x=%d, y=%d, width=%d, height=%d",
                                targetRects[i].x, targetRects[i].y, targetRects[i].width, targetRects[i].height);
                            tracer.traceInfo("TargetRect", "%02d: x=%d, y=%d, width=%d, height=%d",
                                i, targetRects[i].x, targetRects[i].y, targetRects[i].width, targetRects[i].height);
                        }
                    }
                }
            }

            if (DEBUG_FACE_DETECTION)
            {
                if (faceDetector != null && faceDetector.isEnabled())
                {
                    Rect[] faceRects = faceDetector.getFaceRects();
                    if (faceRects != null)
                    {
                        for (int i = 0; i < faceRects.length; i++)
                        {
                            dashboard.displayPrintf(8 + i, "x=%d, y=%d, width=%d, height=%d",
                                faceRects[i].x, faceRects[i].y, faceRects[i].width, faceRects[i].height);
                            tracer.traceInfo("FaceRect", "%02d: x=%d, y=%d, width=%d, height=%d",
                                i, faceRects[i].x, faceRects[i].y, faceRects[i].width, faceRects[i].height);
                        }
                    }
                }
            }

            if (DEBUG_WINCH)
            {
                dashboard.displayPrintf(8, "Winch: power=%.1f, position=%.1f, touch=%s/%s",
                    winch.getPower(), winch.getPosition(),
                    Boolean.toString(winch.isUpperLimitSwitchActive()),
                    Boolean.toString(winch.isLowerLimitSwitchActive()));
                double tilt = winch.getRobotTilt();
                double zGForce = winch.getZValue();
                dashboard.displayPrintf(9, "Winch: current=%.1f/%.1f (%.1f)",
                    winch.getMasterCurrent(), winch.getSlaveCurrent(), winch.getMaxCurrent());
                dashboard.displayPrintf(10, "Winch: OffGround=%s", Boolean.toString(winch.isOffGround()));
                dashboard.displayPrintf(11, "Winch: MotorSlowed=%s", Boolean.toString(winch.isMotorSlowed()));
                dashboard.displayPrintf(12, "Winch: TouchPlate=%s", Boolean.toString(winch.touchingPlate()));
                dashboard.displayPrintf(13, "Winch: tilt=%.1f, Z=%.1f", tilt, zGForce);
                HalDashboard.putNumber("Tilt", tilt);
            }

            if (DEBUG_PIXY)
            {
                if (frontPixy != null && frontPixy.isEnabled())
                {
                    PixyVision.TargetInfo targetInfo = frontPixy.getTargetInfo();
                    if (targetInfo == null)
                    {
                        dashboard.displayPrintf(14, "Pixy: Target not found!");
                    }
                    else
                    {
                        dashboard.displayPrintf(14, "Pixy: x=%d, y=%d, width=%d, height=%d",
                            targetInfo.rect.x, targetInfo.rect.y, targetInfo.rect.width, targetInfo.rect.height);
                        dashboard.displayPrintf(15, "xDistance=%.1f, yDistance=%.1f, angle=%.1f",
                            targetInfo.xDistance, targetInfo.yDistance, targetInfo.angle);
                    }
                }
            }
        }
    }   //updateDashboard

    public void startTraceLog(String prefix)
    {
        String filePrefix = prefix != null? prefix: matchType.toString();
        if (prefix == null) filePrefix += String.format("%03d", matchNumber);
        tracer.openTraceLog("/home/lvuser/tracelog", filePrefix);
    }   //startTraceLog

    public void stopTraceLog()
    {
        tracer.closeTraceLog();
    }   //stopTraceLog

    public void traceStateInfo(double elapsedTime, String stateName)
    {
        tracer.traceInfo(moduleName, "[%5.3f] %10s: xPos=%6.2f,yPos=%6.2f,heading=%6.1f/%6.1f,volts=%.1f(%.1f)",
            elapsedTime, stateName, driveBase.getXPosition(), driveBase.getYPosition(), driveBase.getHeading(),
            targetHeading, battery.getVoltage(), battery.getLowestVoltage());
    }   //traceStateInfo

    //
    // Implements TrcPidController.PidInput.
    //
    @Override
    public double getInput(TrcPidController pidCtrl)
    {
        double value = 0.0;

        if (pidCtrl == encoderXPidCtrl)
        {
            value = driveBase.getXPosition();
        }
        else if (pidCtrl == encoderYPidCtrl)
        {
            value = driveBase.getYPosition();
        }
        else if (pidCtrl == gyroTurnPidCtrl)
        {
            value = driveBase.getHeading();
        }
        else if (pidCtrl == sonarDrivePidCtrl)
        {
            value = getUltrasonicDistance();
            if (value == 0.0)
            {
                value = lastUltrasonicDistance;
            }
            else
            {
                lastUltrasonicDistance = value;
            }
        }
        else if (pidCtrl == visionTurnPidCtrl)
        {
            TargetInfo targetInfo = frontPixy.getTargetInfo();
            value = targetInfo != null? targetInfo.angle: 0.0;
        }

        return value;
    }   //getInput

}   //class Robot
