package team492;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import frclib.FrcCANTalon;
import frclib.FrcTalonServo;
import trclib.TrcEnhancedServo;
import trclib.TrcPidController;
import trclib.TrcSwerveModule;

public class ModuleTest extends TimedRobot
{
    private TrcSwerveModule module;
    private Joystick joystick;
    private FrcCANTalon driveMotor, steerMotor;

    @Override
    public void robotInit()
    {
        joystick = new Joystick(0);

        driveMotor = new FrcCANTalon("Drive", 11);
        steerMotor = new FrcCANTalon("steer", 14);

        TrcPidController.PidCoefficients coeff = new TrcPidController.PidCoefficients(RobotInfo.STEER_KP,
            RobotInfo.STEER_KI, RobotInfo.STEER_KD, RobotInfo.STEER_KF);
        FrcTalonServo servo = new FrcTalonServo("servo", steerMotor, coeff, RobotInfo.STEER_DEGREES_PER_TICK, RobotInfo.STEER_MAX_VEL, RobotInfo.STEER_MAX_ACCEL, false);
        module = new TrcSwerveModule("module", driveMotor, new TrcEnhancedServo("enhancedServo", servo));
    }

    @Override
    public void teleopPeriodic()
    {
        double angle = Math.toDegrees(Math.atan2(deadband(joystick.getX()), deadband(-joystick.getY())));
        module.setSteerAngle(angle);
        double currAngle = module.getSteerAngle();
        System.out.printf("angle=%.2f,target=%.2f\n", currAngle, angle);
    }

    private double deadband(double d){
        return Math.abs(d) >= 0.05 ? d : 0.0;
    }

    @Override
    public void disabledInit()
    {
        driveMotor.set(0.0);
        steerMotor.set(0.0);
    }
}
