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

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frclib.FrcAHRSGyro;
import frclib.FrcCANSparkMax;
import frclib.FrcCANTalon;
import frclib.FrcJoystick;
import frclib.FrcPdp;
import frclib.FrcRobotBase;
import frclib.FrcRobotBattery;
import frclib.FrcTalonServo;
import hallib.HalDashboard;
import trclib.TrcEnhancedServo;
import trclib.TrcPidController;
import trclib.TrcPidController.PidCoefficients;
import trclib.TrcPidDrive;
import trclib.TrcPidMotor;
import trclib.TrcRobot.RunMode;
import trclib.TrcRobotBattery;
import trclib.TrcSwerveDriveBase;
import trclib.TrcSwerveModule;
import trclib.TrcUtil;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.PrintStream;
import java.util.Date;
import java.util.Scanner;
import java.util.stream.IntStream;

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
    public static final boolean USE_MAGIC_STEER = true;

    private static final boolean DEBUG_POWER_CONSUMPTION = false;
    private static final boolean DEBUG_DRIVE_BASE = false;
    private static final boolean DEBUG_PID_DRIVE = false;
    private static final boolean DEBUG_SUBSYSTEMS = false;

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
    public FrcJoystick buttonPanel = null;
    public FrcJoystick switchPanel = null;
    //
    // Sensors.
    //
    public FrcPdp pdp = null;
    public TrcRobotBattery battery = null;
    public FrcAHRSGyro gyro = null;
    //
    // DriveBase subsystem.
    //
    public FrcCANSparkMax lfDriveMotor, rfDriveMotor, lrDriveMotor, rrDriveMotor;
    public FrcCANTalon lfSteerMotor, rfSteerMotor, lrSteerMotor, rrSteerMotor;
    public TrcSwerveModule leftFrontWheel;
    public TrcSwerveModule leftRearWheel;
    public TrcSwerveModule rightFrontWheel;
    public TrcSwerveModule rightRearWheel;
    public TrcSwerveDriveBase driveBase;

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

    private FrcAuto autoMode;

    /**
     * Constructor.
     */
    public Robot()
    {
        super(programName);
    }   //Robot

    private FrcCANSparkMax createSparkMax(String name, int id)
    {
        FrcCANSparkMax spark = new FrcCANSparkMax(name, id, true);
        spark.setInverted(false);
        spark.setPositionSensorInverted(false);
        spark.motor.enableVoltageCompensation(RobotInfo.BATTERY_NOMINAL_VOLTAGE);
        return spark;
    }

    private FrcCANTalon createSteerTalon(String name, int id, boolean inverted)
    {
        FrcCANTalon talon = new FrcCANTalon(name, id);
        talon.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        talon.motor.configVoltageCompSaturation(RobotInfo.BATTERY_NOMINAL_VOLTAGE);
        talon.motor.enableVoltageCompensation(true);
        talon.motor.overrideLimitSwitchesEnable(false);
        talon.setBrakeModeEnabled(true);
        talon.setPositionSensorInverted(inverted);
        talon.setInverted(!inverted);
        // TODO: Fix sensor reading. currently, increasing target angle turns CW, which is correct.
        // however, this reads as more negative, which is weird honestly. (90 deg reads -90 deg) Something seems out of phase?
        // The sensors are confirmed to be in phase with motor, so figure out some polarity garbage?
        return talon;
    }

    private TrcSwerveModule createModule(String name, FrcCANSparkMax drive, FrcCANTalon steer, int steerZero)
    {
        steer.motor.getSensorCollection().setPulseWidthPosition(0, 10); // only resets the index
        int pwmPosTicks = steer.motor.getSensorCollection().getPulseWidthPosition();
        int absPosTicks = pwmPosTicks - steerZero;
        steer.motor.getSensorCollection().setQuadraturePosition(absPosTicks, 10);

        TrcSwerveModule module;
        if (USE_MAGIC_STEER)
        {
            FrcTalonServo servo = new FrcTalonServo(name + ".servo", steer, RobotInfo.magicSteerCoeff,
                RobotInfo.STEER_DEGREES_PER_TICK, RobotInfo.STEER_MAX_REQ_VEL, RobotInfo.STEER_MAX_ACCEL);
            module = new TrcSwerveModule(name, drive, new TrcEnhancedServo(name + ".enhancedServo", servo));
        }
        else
        {
            TrcPidController ctrl = new TrcPidController(name + ".ctrl", RobotInfo.pidSteerCoeff,
                RobotInfo.STEER_TOLERANCE, steer::getPosition);
            TrcPidMotor pidMotor = new TrcPidMotor(name + ".pid", steer, ctrl, 0.0);
            pidMotor.setPositionScale(RobotInfo.STEER_DEGREES_PER_TICK);
            module = new TrcSwerveModule(name, drive, pidMotor);
        }
        module.disableSteeringLimits();
        return module;
    }

    private int[] getSteerZeroPositions()
    {
        try (Scanner in = new Scanner(new FileReader("/home/lvuser/steerzeros.txt")))
        {
            return IntStream.range(0, 4).map(e -> in.nextInt()).toArray();
        }
        catch (Exception e)
        {
            return new int[4];
        }
    }

    public void saveSteerZeroPositions()
    {
        try (PrintStream out = new PrintStream(new FileOutputStream("/home/lvuser/steerzeros.txt")))
        {
            out.printf("%.0f\n", TrcUtil.modulo(lfSteerMotor.getPosition(), 4096));
            out.printf("%.0f\n", TrcUtil.modulo(rfSteerMotor.getPosition(), 4096));
            out.printf("%.0f\n", TrcUtil.modulo(lrSteerMotor.getPosition(), 4096));
            out.printf("%.0f\n", TrcUtil.modulo(rrSteerMotor.getPosition(), 4096));
        }
        catch (FileNotFoundException e)
        {
            e.printStackTrace();
        }
    }

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

        pdp.registerEnergyUsed(new FrcPdp.Channel(RobotInfo.PDP_CHANNEL_LEFT_FRONT_WHEEL, "LeftFrontWheel"),
            new FrcPdp.Channel(RobotInfo.PDP_CHANNEL_LEFT_REAR_WHEEL, "LeftRearWheel"),
            new FrcPdp.Channel(RobotInfo.PDP_CHANNEL_RIGHT_FRONT_WHEEL, "RightFrontWheel"),
            new FrcPdp.Channel(RobotInfo.PDP_CHANNEL_RIGHT_REAR_WHEEL, "RightRearWheel"));

        //
        // DriveBase subsystem.
        //
        lfDriveMotor = createSparkMax("LFDrive", RobotInfo.CANID_LEFTFRONT_DRIVE);
        rfDriveMotor = createSparkMax("RFDrive", RobotInfo.CANID_RIGHTFRONT_DRIVE);
        lrDriveMotor = createSparkMax("LRDrive", RobotInfo.CANID_LEFTREAR_DRIVE);
        rrDriveMotor = createSparkMax("RRDrive", RobotInfo.CANID_RIGHTREAR_DRIVE);

        lfSteerMotor = createSteerTalon("LFSteer", RobotInfo.CANID_LEFTFRONT_STEER, false);
        rfSteerMotor = createSteerTalon("RFSteer", RobotInfo.CANID_RIGHTFRONT_STEER, true);
        lrSteerMotor = createSteerTalon("LRSteer", RobotInfo.CANID_LEFTREAR_STEER, true);
        rrSteerMotor = createSteerTalon("RRSteer", RobotInfo.CANID_RIGHTREAR_STEER, false);

        int[] zeros = getSteerZeroPositions();
        leftFrontWheel = createModule("LeftFrontWheel", lfDriveMotor, lfSteerMotor, zeros[0]);
        rightFrontWheel = createModule("RightFrontWheel", rfDriveMotor, rfSteerMotor, zeros[1]);
        leftRearWheel = createModule("LeftRearWheel", lrDriveMotor, lrSteerMotor, zeros[2]);
        rightRearWheel = createModule("RightRearWheel", rrDriveMotor, rrSteerMotor, zeros[3]);

        //
        // Initialize DriveBase subsystem.
        //
        driveBase = new TrcSwerveDriveBase(leftFrontWheel, leftRearWheel, rightFrontWheel, rightRearWheel, gyro,
            RobotInfo.ROBOT_WIDTH, RobotInfo.ROBOT_LENGTH);
        driveBase.setPositionScales(RobotInfo.ENCODER_INCHES_PER_COUNT);
        //
        // Create PID controllers for DriveBase PID drive.
        //
        encoderXPidCtrl = new TrcPidController("encoderXPidCtrl",
            new PidCoefficients(RobotInfo.ENCODER_KP, RobotInfo.ENCODER_KI, RobotInfo.ENCODER_KD, RobotInfo.ENCODER_KF),
            RobotInfo.ENCODER_TOLERANCE, driveBase::getXPosition);
        encoderYPidCtrl = new TrcPidController("encoderYPidCtrl",
            new PidCoefficients(RobotInfo.ENCODER_KP, RobotInfo.ENCODER_KI, RobotInfo.ENCODER_KD, RobotInfo.ENCODER_KF),
            RobotInfo.ENCODER_TOLERANCE, driveBase::getYPosition);
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

        NetworkTableEntry zerosButton = SmartDashboard.getEntry("SaveZeros");
        zerosButton.setBoolean(false);
        zerosButton.addListener(this::zeroButtonPress, EntryListenerFlags.kUpdate);

        //
        // Create Robot Modes.
        //
        autoMode = new FrcAuto(this);
        setupRobotModes(new FrcTeleOp(this), autoMode, new FrcTest(this), new FrcDisabled(this));

        pdp.registerEnergyUsedForAllUnregisteredChannels();
    }   //robotInit

    private void zeroButtonPress(EntryNotification e)
    {
        saveSteerZeroPositions();
        e.getEntry().setBoolean(false);
    }

    public void robotStartMode(RunMode runMode, RunMode prevMode)
    {
        final String funcName = "robotStartMode";

        if (runMode != RunMode.DISABLED_MODE)
        {
            openTraceLog(
                runMode == RunMode.AUTO_MODE ? "FrcAuto" : runMode == RunMode.TELEOP_MODE ? "FrcTeleOp" : "FrcTest");
            setTraceLogEnabled(true);

            Date now = new Date();
            globalTracer
                .traceInfo(funcName, "[%.3f] %s: ***** %s *****", Robot.getModeElapsedTime(), now.toString(), runMode);

            pdp.setTaskEnabled(true);
            battery.setEnabled(true);
            driveBase.resetOdometry(true, false);
            driveBase.setOdometryEnabled(true);
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
            globalTracer.traceInfo(funcName, "mode=%s,heading=%.1f", runMode.name(), driveBase.getHeading());
            driveBase.setOdometryEnabled(false);
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

    /**
     * Stops the lower level subsystems. This does NOT stop any auto/auto assist commands.
     */
    public void stopSubsystems()
    {
        pidDrive.cancel();
        driveBase.stop();
    }

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
        return autoMode.isAutoActive();
    }

    public void cancelAllAuto()
    {
        if (autoMode.isAutoActive())
        {
            autoMode.cancel();
        }
    }

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
}   //class Robot
