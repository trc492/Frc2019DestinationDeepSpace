package team492;

import com.revrobotics.CANSparkMax;
import frclib.FrcCANSparkMax;
import frclib.FrcCANTalon;

public class SparkMaxFollowTest
{
    private FrcCANTalon talon;
    private FrcCANSparkMax spark;

    public SparkMaxFollowTest()
    {
        talon = new FrcCANTalon("Talon", 3);
        spark = new FrcCANSparkMax("SparkMax", 4, true);
        spark.motor.follow(CANSparkMax.ExternalFollower.kFollowerPhoenix, talon.motor.getDeviceID());
    }

    public void start()
    {
        talon.set(0.5); // 50% power, the spark should be doing this too.
    }

    public void stop()
    {
        talon.set(0); // stop the motor
    }
}
