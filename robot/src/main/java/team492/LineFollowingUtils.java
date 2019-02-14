package team492;

import org.opencv.core.Point;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.calib3d.Calib3d;

import java.util.LinkedList;

public class LineFollowingUtils
{
    private CameraFieldOfView cfov;

    public LineFollowingUtils()
    {
        Point tL = new Point(RobotInfo.PIXY2_TOP_LEFT_X, RobotInfo.PIXY2_TOP_LEFT_Y);
        Point tR = new Point(RobotInfo.PIXY2_TOP_RIGHT_X, RobotInfo.PIXY2_TOP_RIGHT_Y);
        Point bL = new Point(RobotInfo.PIXY2_BOTTOM_LEFT_X, RobotInfo.PIXY2_BOTTOM_LEFT_Y);
        Point bR = new Point(RobotInfo.PIXY2_BOTTOM_RIGHT_X, RobotInfo.PIXY2_BOTTOM_RIGHT_Y);
        cfov = new CameraFieldOfView(tL, tR, bL, bR, RobotInfo.PIXY2_LINE_TRACKING_WIDTH,
            RobotInfo.PIXY2_LINE_TRACKING_HEIGHT);
    }

    public double getTurnDegrees(double angle)
    {
        return angle - 90.0;
    }

    public double getAngle(Point origin, Point p2)
    {
        double dx = p2.x - origin.x;
        double dy = p2.y - origin.y;

        double theta = Math.atan2(dy, dx);
        theta = Math.toDegrees(theta);
        theta = (theta + 360.0) % 360.0;
        return theta;
    }

    /**
     * This function creates an estimation of an object's position in real-world
     * units, using a homography matrix
     * 
     * @param x0
     *               the x-coordinate of a pair on the camera plane. (x: right
     *               positive)
     * @param y0
     *               the y-coordinate of a pair on the camera plane: (y: down
     *               positive)
     * @return a RealWorldPair instance of the approximate scaled real world
     *         location of the objects at the coordinates (x0, y0)
     */
    public Point getRWP(double x0, double y0)
    {
        return cfov.GetRealWorldCoords(new Point(x0, y0));
    }

    public static class CameraFieldOfView
    {
        // X,Y robot coordinates of the corners of the image. we're assuming no
        // lens distortion
        private Point topLeft;
        private Point topRight;
        private Point bottomLeft;
        private Point bottomRight;
        private double xResolution;
        private double yResolution;
        // homography matrix mapping image corners to real-world coordinates.
        private Mat H;

        public CameraFieldOfView(Point topLeft, Point topRight, Point bottomLeft, Point bottomRight, double xResolution,
            double yResolution)
        {
            this.topLeft = topLeft;
            this.topRight = topRight;
            this.bottomLeft = bottomLeft;
            this.bottomRight = bottomRight;
            this.xResolution = xResolution;
            this.yResolution = yResolution;
            // find the 3x3 homography matrix H
            LinkedList<Point> objList = new LinkedList<Point>();
            LinkedList<Point> sceneList = new LinkedList<Point>();

            objList.add(topLeft);
            sceneList.add(new Point(0.0, 0.0));
            objList.add(topRight);
            sceneList.add(new Point(this.xResolution, 0.0));
            objList.add(bottomLeft);
            sceneList.add(new Point(0.0, this.yResolution));
            objList.add(bottomRight);
            sceneList.add(new Point(this.xResolution, this.yResolution));

            MatOfPoint2f obj = new MatOfPoint2f();
            obj.fromList(objList);

            MatOfPoint2f scene = new MatOfPoint2f();
            scene.fromList(sceneList);

            this.H = Calib3d.findHomography(scene, obj);
        }

        // get the real world coordinates of a image (x,y) point.
        public Point GetRealWorldCoords(Point p)
        {
            double[][] pmat = new double[3][1];
            pmat[0][0] = p.x;
            pmat[1][0] = p.y;
            pmat[2][0] = 1.0;
        	
            double[][] h = new double[3][3];
            
            for (int i = 0; i < this.H.rows(); i++)
            {
                for (int j = 0; j < this.H.cols(); j++)
                {
                    h[i][j] = this.H.get(i, j)[0];
                }
            }
            
            
            double[][] result = multiply(h, pmat);
            
            for (int furry = 0; furry < result.length; furry++)
            {
            	result[furry][0] *= (1.0 / result[2][0]);
            }
            
            return new Point(result[0][0], result[1][0]);
        }
        
        public double[][] multiply(double[][] firstMatrix, double[][] secondMatrix)
        {
        	double[][] product = new double[firstMatrix.length][secondMatrix[0].length];
            for(int i = 0; i < firstMatrix.length; i++) {
                for (int j = 0; j < secondMatrix[0].length; j++) {
                    for (int k = 0; k < firstMatrix[0].length; k++) {
                        product[i][j] += firstMatrix[i][k] * secondMatrix[k][j];
                    }
                }
            }
            return product;
        }
    }
}