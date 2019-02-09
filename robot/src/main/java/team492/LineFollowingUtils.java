package team492;

// import org.opencv.

public class LineFollowingUtils
{
    public static double getTurnDegrees(double angle)
    {
        return angle - 90.0;
    }

    public static double getAngle(RealWorldPair origin, RealWorldPair p2)
    {
        double dx = p2.getXLength() - origin.getXLength();
        double dy = p2.getYLength() - origin.getYLength();

        double theta = Math.atan2(dy, dx);
        theta = Math.toDegrees(theta);
        theta = (theta + 360.0) % 360.0;
        return theta;
    }

    /**
	 * This function creates a rough estimation of an object's position in real-world units.
     * 
	 * @param x0 the x-coordinate of a pair on the camera plane. (x: right positive)
	 * @param y0 the y-coordinate of a pair on the camera plane: (y: down positive)
	 * @param widthCoefficient: the width coefficient, the length of the camera view width in inches.
	 * @param heightCoefficient the height coefficient, the height of the camera view width in inches.
	 * @return a RealWorldPair instance of the approximate scaled real world location of the objects at the coordinates (x0, y0)
	 */
    public static RealWorldPair getRWP(int x0, int y0, double widthCoefficient, double heightCoefficient)
    {
        double xLength = widthCoefficient * ((double) x0 / (double) RobotInfo.PIXY2_LINE_TRACKING_WIDTH);
        double yLength = heightCoefficient * ((double) (RobotInfo.PIXY2_LINE_TRACKING_HEIGHT - y0) / (double) RobotInfo.PIXY2_LINE_TRACKING_HEIGHT);
        return new RealWorldPair(xLength, yLength);
    }

    public static class RealWorldPair
    {
        private double xLength;
        private double yLength;

        public RealWorldPair(double xLength, double yLength)
        {
            this.xLength = xLength;
            this.yLength = yLength;
        }

        public double getXLength()
        {
            return xLength;
        }

        public double getYLength()
        {
            return yLength;
        }
    }
}