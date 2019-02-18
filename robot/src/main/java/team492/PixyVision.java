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

import java.util.ArrayList;

import org.opencv.core.Point;
import org.opencv.core.Rect;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import frclib.FrcPixyCam1;
import frclib.FrcPixyCam2;
import trclib.TrcHomographyMapper;
import trclib.TrcPixyCam1;
import trclib.TrcPixyCam2;
import trclib.TrcPixyCam2.*;
import trclib.TrcUtil;

public class PixyVision
{
    private static final boolean debugEnabled = false;

    // If last target rect is this old, its stale data.
    private static final double LAST_TARGET_RECT_FRESH_DURATION_SECONDS = 0.1;  // 100 msec

    public class TargetInfo
    {
        public Rect rect;
        public double xDistance;
        public double yDistance;
        public double angle;

        public TargetInfo(Rect rect, double xDistance, double yDistance, double angle)
        {
            this.rect = rect;
            this.xDistance = xDistance;
            this.yDistance = yDistance;
            this.angle = angle;
        }   //TargetInfo

        public String toString()
        {
            return String.format("Rect[%d,%d,%d,%d], xDistance=%.1f, yDistance=%.1f, angle=%.1f",
                rect.x, rect.y, rect.width, rect.height, xDistance, yDistance, angle);
        }
    }   //class TargetInfo

    public enum Orientation
    {
        NORMAL_LANDSCAPE,
        CLOCKWISE_PORTRAIT,
        ANTICLOCKWISE_PORTRAIT,
        UPSIDEDOWN_LANDSCAPE
    }   //enum Orientation

    private static final double PIXY_DISTANCE_SCALE = 2300.0;   //DistanceInInches*targetWidthdInPixels
    private static final double TARGET_WIDTH_INCHES = 13.0 * Math.sqrt(2.0);// 13x13 square, diagonal is 13*sqrt(2) inches

    private FrcPixyCam1 pixyCamera1 = null;
    private FrcPixyCam2 pixyCamera2 = null;
    private TrcHomographyMapper homographyMapper = null;
    private Robot robot;
    private int signature;
    private int brightness;
    private Orientation orientation;
    private Rect lastTargetRect = null;
    private double lastTargetRectExpireTime = TrcUtil.getCurrentTime();

    private void commonInit(Robot robot, int signature, int brightness, Orientation orientation)
    {
        this.robot = robot;
        this.signature = signature;
        this.brightness = brightness;
        this.orientation = orientation;
        homographyMapper = new TrcHomographyMapper(
            // Camera coordinates: top left, top right, bottom left and bottom right
            new Point(0.0, 0.0), new Point(RobotInfo.PIXY2_LINE_TRACKING_WIDTH, 0.0),
            new Point(0.0, RobotInfo.PIXY2_LINE_TRACKING_HEIGHT), new Point(RobotInfo.PIXY2_LINE_TRACKING_WIDTH, RobotInfo.PIXY2_LINE_TRACKING_HEIGHT),
            // World coordinates: top left, top right, bottom left and bottom right.
            new Point(RobotInfo.PIXYCAM_WORLD_TOPLEFT_X, RobotInfo.PIXYCAM_WORLD_TOPLEFT_Y),
            new Point(RobotInfo.PIXYCAM_WORLD_TOPRIGHT_X, RobotInfo.PIXYCAM_WORLD_TOPRIGHT_Y),
            new Point(RobotInfo.PIXYCAM_WORLD_BOTTOMLEFT_X, RobotInfo.PIXYCAM_WORLD_BOTTOMLEFT_Y),
            new Point(RobotInfo.PIXYCAM_WORLD_BOTTOMRIGHT_X, RobotInfo.PIXYCAM_WORLD_BOTTOMRIGHT_Y));
    }   //commonInit

    public PixyVision(
        final String instanceName, Robot robot, boolean pixyV2, int signature, int brightness, Orientation orientation,
        SPI.Port port)
    {
        if (pixyV2)
        {
            pixyCamera2 = new FrcPixyCam2(instanceName, port);
        }
        else
        {
            pixyCamera1 = new FrcPixyCam1(instanceName, port);
        }
        commonInit(robot, signature, brightness, orientation);
    }   //PixyVision

    public PixyVision(
        final String instanceName, Robot robot, boolean pixyV2, int signature, int brightness, Orientation orientation,
        I2C.Port port, int i2cAddress)
    {
        if (pixyV2)
        {
            pixyCamera2 = new FrcPixyCam2(instanceName, port, i2cAddress);
        }
        else
        {
            pixyCamera1 = new FrcPixyCam1(instanceName, port, i2cAddress);
        }
        commonInit(robot, signature, brightness, orientation);
    }   //PixyVision

    public void setEnabled(boolean enabled)
    {
        if (pixyCamera1 != null)
        {
            pixyCamera1.setEnabled(enabled);
            if (enabled)
            {
                pixyCamera1.setBrightness((byte)brightness);
            }
        }
        else if (pixyCamera2 != null)
        {
            pixyCamera2.setEnabled(enabled);
            if (enabled)
            {
                pixyCamera2.setCameraBrightness((byte)brightness);
            }
        }
    }   //setEnabled

    public boolean isEnabled()
    {
        return pixyCamera1 != null ? pixyCamera1.isEnabled() : pixyCamera2 != null ? pixyCamera2.isEnabled() : false;
    }   //isEnabled

    private Vector[] getFeatureVectors()
    {
        final String funcName = "getFeatureVectors";
        Vector[] vectors = null;

        if (pixyCamera2 != null)
        {
            double startTime = TrcUtil.getCurrentTime();
            Feature[] features = pixyCamera2.getMainFeatures(TrcPixyCam2.PIXY2_FEATURES_VECTOR);
            double elapsedTime = TrcUtil.getCurrentTime() - startTime;

            if (features != null)
            {
                for (int i = 0; i < features.length; i++)
                {
                    if (debugEnabled)
                    {
                        robot.globalTracer.traceInfo(funcName, "[%.3f] %d/%d: %s",
                            elapsedTime, i, features.length, features[i]);
                    }

                    if (features[i].type == TrcPixyCam2.PIXY2_FEATURES_VECTOR)
                    {
                        vectors = ((FeatureVectors) features[i]).vectors;
                        break;
                    }
                }
            }
        }

        return vectors;
    }   //getFeatureVectors

    public Vector getLineVector()
    {
        final String funcName = "getLineVector";
        Vector lineVector = null;
        Vector[] vectors = getFeatureVectors();

        if (vectors != null && vectors.length > 0)
        {

            /*
            // Disregard this code and the following two comments for now, we are going to find the centermost vector instead.
            // - Ideally, the best line should be the longest line.
            // - This is mostly to filter out any remaining interference that the Pixy might pick up.
            double maxLen = 0.0;

            for (int i = 0; i < vectors.length; i++)
            {
                double dx = vectors[i].x1 - vectors[i].x0;
                double dy = vectors[i].y1 - vectors[i].y0;
                double lineLen = Math.sqrt(dx * dx + dy * dy);

                if (lineLen > maxLen)
                {
                    maxLen = lineLen;
                    lineVector = vectors[i];
                }
            }
            */

            // - Ideally, the best line should be the center-most line.
            // - This is to filter out remaining interference that the Pixy might pick up
            //   and maximize the chances of returning the line that our robot wants to align with.
            double centerDistance = Double.MAX_VALUE;

            for (int i = 0; i < vectors.length; i++)
            {
                double lineCenterX = (vectors[i].x1 + vectors[i].x0) / 2.0;
                double lineCenterY = (vectors[i].y1 + vectors[i].y0) / 2.0;
                double dx = lineCenterX - ((RobotInfo.PIXY2_LINE_TRACKING_WIDTH / 2.0) + RobotInfo.PIXY2_LINE_TRACK_MID_WIDTH_OFFST);
                double dy = lineCenterY - ((RobotInfo.PIXY2_LINE_TRACKING_HEIGHT / 2.0) + RobotInfo.PIXY2_LINE_TRACK_MID_HEIGHT_OFFST);
                double centerDist = Math.sqrt(dx * dx + dy * dy);
                if (centerDist < centerDistance)
                {
                    centerDistance = centerDist;
                    lineVector = vectors[i];
                }
            }

            if (debugEnabled)
            {
                if (lineVector != null)
                {
                    // robot.globalTracer.traceInfo(funcName, "Line found (len=%.2f): %s", maxLen, lineVector);
                    robot.globalTracer.traceInfo(funcName, "Line found (distanceCenter=%.2f): %s", centerDistance, lineVector);
                }
                else
                {
                    robot.globalTracer.traceInfo(funcName, "Line not found!");
                }
            }
        }

        return lineVector;
    }   //getLineVector

    private double getLineAngle(Point p1, Point p2)
    {
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;

        double theta = Math.atan2(dy, dx);
        theta = Math.toDegrees(theta);
        theta = (theta + 360.0) % 360.0;
        return theta;
    }   //getLineAngle

    public double getVectorAngle(Vector vector)
    {
        Point p1 = homographyMapper.mapPoint(new Point(vector.x0, vector.y0));
        Point p2 = homographyMapper.mapPoint(new Point(vector.x1, vector.y1));
        return getLineAngle(p1, p2) - 90.0;
    }   //getVectorAngle

    public Point mapPoint(Point point)
    {
        return homographyMapper.mapPoint(point);
    }

    /**
     * This method gets the rectangle of the last detected target from the camera. If the camera does not have
     * any. It may mean the camera is still busy analyzing a frame or it can't find any valid target in a frame.
     * We can't tell the reason. If the camera is likely busying processing a frame, we will return the last
     * cached rectangle. Therefore, the last cached rectangle will have an expiration (i.e. cached data can be
     * stale). If the last cached data becomes stale, we will discard it and return nothing. Otherwise, we will
     * return the cached data. Of course we will return fresh data if the camera does return another rectangle,
     * in which case it will become the new cached data.
     *
     * @return rectangle of the detected target last received from the camera or last cached target if cached
     *         data has not expired. Null if no object was seen and last cached data has expired.
     */
    private Rect getV1TargetRect()
    {
        final String funcName = "getV1TargetRect";
        Rect targetRect = null;
        TrcPixyCam1.ObjectBlock[] detectedObjects = pixyCamera1.getDetectedObjects();
        double currTime = TrcUtil.getCurrentTime();

        if (debugEnabled)
        {
            robot.globalTracer.traceInfo(
                funcName, "%s object(s) found", detectedObjects != null? "" + detectedObjects.length: "null");
        }

        if (detectedObjects != null && detectedObjects.length >= 1)
        {
            //
            // Make sure the camera detected at least one objects.
            //
            ArrayList<Rect> objectList = new ArrayList<>();
            //
            // Filter out objects that don't have the correct signature.
            //
            for (int i = 0; i < detectedObjects.length; i++)
            {
                if (signature == detectedObjects[i].signature)
                {
                    int temp;
                    //
                    // If we have the camera mounted in other orientations, we need to adjust the object rectangles
                    // accordingly.
                    //
                    switch (orientation)
                    {
                        case CLOCKWISE_PORTRAIT:
                            temp = RobotInfo.PIXYCAM_WIDTH - detectedObjects[i].centerX;
                            detectedObjects[i].centerX = detectedObjects[i].centerY;
                            detectedObjects[i].centerY = temp;
                            temp = detectedObjects[i].width;
                            detectedObjects[i].width = detectedObjects[i].height;
                            detectedObjects[i].height = temp;
                            break;

                        case ANTICLOCKWISE_PORTRAIT:
                            temp = detectedObjects[i].centerX;
                            detectedObjects[i].centerX = RobotInfo.PIXYCAM_HEIGHT - detectedObjects[i].centerY;
                            detectedObjects[i].centerY = temp;
                            temp = detectedObjects[i].width;
                            detectedObjects[i].width = detectedObjects[i].height;
                            detectedObjects[i].height = temp;
                            break;

                        case UPSIDEDOWN_LANDSCAPE:
                            detectedObjects[i].centerX = RobotInfo.PIXYCAM_WIDTH - detectedObjects[i].centerX;
                            detectedObjects[i].centerY = RobotInfo.PIXYCAM_HEIGHT - detectedObjects[i].centerY;
                            break;

                        case NORMAL_LANDSCAPE:
                            break;
                    }

                    Rect rect = new Rect(detectedObjects[i].centerX - detectedObjects[i].width/2,
                                         detectedObjects[i].centerY - detectedObjects[i].height/2,
                                         detectedObjects[i].width, detectedObjects[i].height);
                    objectList.add(rect);

                    if (debugEnabled)
                    {
                        robot.globalTracer.traceInfo(funcName, "[%d] %s", i, detectedObjects[i].toString());
                    }
                }
            }

            if (objectList.size() >= 1)
            {
                //
                // Find the largest target rect in the list.
                //
                Rect maxRect = objectList.get(0);
                for(Rect rect: objectList)
                {
                    double area = rect.width * rect.height;
                    if (area > maxRect.width * maxRect.height)
                    {
                        maxRect = rect;
                    }
                }

                targetRect = maxRect;

                if (debugEnabled)
                {
                    robot.globalTracer.traceInfo(funcName, "===TargetRect===: x=%d, y=%d, w=%d, h=%d",
                        targetRect.x, targetRect.y, targetRect.width, targetRect.height);
                }
            }

            if (targetRect == null)
            {
                robot.globalTracer.traceInfo(funcName, "===TargetRect=== None, is now null");
            }

            lastTargetRect = targetRect;
            lastTargetRectExpireTime = currTime + LAST_TARGET_RECT_FRESH_DURATION_SECONDS;
        }
        else if (currTime < lastTargetRectExpireTime)
        {
            targetRect = lastTargetRect;
        }

        return targetRect;
    }   //getV1TargetRect

    private Rect getV2TargetRect()
    {
        final String funcName = "getV2TargetRect";
        Rect targetRect = null;
        Block[] detectedObjects = pixyCamera2.getBlocks((byte) 255, (byte) 255);
        double currTime = TrcUtil.getCurrentTime();

        if (debugEnabled)
        {
            robot.globalTracer.traceInfo(
                funcName, "%s object(s) found", detectedObjects != null? "" + detectedObjects.length: "null");
        }

        if (detectedObjects != null && detectedObjects.length >= 1)
        {
            //
            // Make sure the camera detected at least one objects.
            //
            ArrayList<Rect> objectList = new ArrayList<>();
            //
            // Filter out objects that don't have the correct signature.
            //
            for (int i = 0; i < detectedObjects.length; i++)
            {
                if (signature == detectedObjects[i].signature)
                {
                    int temp;
                    //
                    // If we have the camera mounted in other orientations, we need to adjust the object rectangles
                    // accordingly.
                    //
                    switch (orientation)
                    {
                        case CLOCKWISE_PORTRAIT:
                            temp = RobotInfo.PIXYCAM_WIDTH - detectedObjects[i].centerX;
                            detectedObjects[i].centerX = detectedObjects[i].centerY;
                            detectedObjects[i].centerY = temp;
                            temp = detectedObjects[i].width;
                            detectedObjects[i].width = detectedObjects[i].height;
                            detectedObjects[i].height = temp;
                            break;

                        case ANTICLOCKWISE_PORTRAIT:
                            temp = detectedObjects[i].centerX;
                            detectedObjects[i].centerX = RobotInfo.PIXYCAM_HEIGHT - detectedObjects[i].centerY;
                            detectedObjects[i].centerY = temp;
                            temp = detectedObjects[i].width;
                            detectedObjects[i].width = detectedObjects[i].height;
                            detectedObjects[i].height = temp;
                            break;

                        case UPSIDEDOWN_LANDSCAPE:
                            detectedObjects[i].centerX = RobotInfo.PIXYCAM_WIDTH - detectedObjects[i].centerX;
                            detectedObjects[i].centerY = RobotInfo.PIXYCAM_HEIGHT - detectedObjects[i].centerY;
                            break;

                        case NORMAL_LANDSCAPE:
                            break;
                    }

                    Rect rect = new Rect(detectedObjects[i].centerX - detectedObjects[i].width/2,
                                         detectedObjects[i].centerY - detectedObjects[i].height/2,
                                         detectedObjects[i].width, detectedObjects[i].height);
                    objectList.add(rect);

                    if (debugEnabled)
                    {
                        robot.globalTracer.traceInfo(funcName, "[%d] %s", i, detectedObjects[i].toString());
                    }
                }
            }

            if (objectList.size() >= 1)
            {
                //
                // Find the largest target rect in the list.
                //
                Rect maxRect = objectList.get(0);
                for(Rect rect: objectList)
                {
                    double area = rect.width * rect.height;
                    if (area > maxRect.width * maxRect.height)
                    {
                        maxRect = rect;
                    }
                }

                targetRect = maxRect;

                if (debugEnabled)
                {
                    robot.globalTracer.traceInfo(funcName, "===TargetRect===: x=%d, y=%d, w=%d, h=%d",
                        targetRect.x, targetRect.y, targetRect.width, targetRect.height);
                }
            }

            if (targetRect == null)
            {
                robot.globalTracer.traceInfo(funcName, "===TargetRect=== None, is now null");
            }

            lastTargetRect = targetRect;
            lastTargetRectExpireTime = currTime + LAST_TARGET_RECT_FRESH_DURATION_SECONDS;
        }
        else if (currTime < lastTargetRectExpireTime)
        {
            targetRect = lastTargetRect;
        }

        return targetRect;
    }   //getV2TargetRect

    public TargetInfo getTargetInfo()
    {
        final String funcName = "getTargetInfo";
        TargetInfo targetInfo = null;
        Rect targetRect = pixyCamera1 != null ? getV1TargetRect() : pixyCamera2 != null ? getV2TargetRect() : null;

        if (targetRect != null)
        {
            //
            // Physical target width:           W = 10 inches.
            // Physical target distance 1:      D1 = 20 inches.
            // Target pixel width at 20 inches: w1 = 115
            // Physical target distance 2:      D2 = 24 inches
            // Target pixel width at 24 inches: w2 = 96
            // Camera lens focal length:        f
            //    W/D1 = w1/f and W/D2 = w2/f
            // => f = w1*D1/W and f = w2*D2/W
            // => w1*D1/W = w2*D2/W
            // => w1*D1 = w2*D2 = PIXY_DISTANCE_SCALE = 2300
            //
            // Screen center X:                 Xs = 320/2 = 160
            // Target center X:                 Xt
            // Heading error:                   e = Xt - Xs
            // Turn angle:                      a
            //    tan(a) = e/f
            // => a = atan(e/f) and f = w1*D1/W
            // => a = atan((e*W)/(w1*D1))
            //
            double targetCenterX = targetRect.x + targetRect.width/2.0;
            double targetXDistance = (targetCenterX - RobotInfo.PIXYCAM_WIDTH/2.0)*TARGET_WIDTH_INCHES/targetRect.width;
            double targetYDistance = PIXY_DISTANCE_SCALE/targetRect.width;
            double targetAngle = Math.toDegrees(Math.atan(targetXDistance/targetYDistance));
            targetInfo = new TargetInfo(targetRect, targetXDistance, targetYDistance, targetAngle);

            if (debugEnabled)
            {
                robot.globalTracer.traceInfo(
                    funcName, "###TargetInfo###: xDist=%.1f, yDist=%.1f, angle=%.1f",
                    targetXDistance, targetYDistance, targetAngle);
            }
        }

        return targetInfo;
    }   //getTargetInfo

}   // class PixyVision