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

package raspivision;

import com.google.gson.Gson;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

public class RaspiVision
{
    private enum DebugDisplayType
    {
        NONE, REGULAR, BOUNDING_BOX, MASK, CORNERS, FULL_PNP
    }

    private static final int TEAM_NUMBER = 492;
    private static final boolean SERVER = false; // true for debugging only
    private static final boolean MEASURE_FPS = true;
    private static final double FPS_AVG_WINDOW = 5; // 5 seconds
    private static final DebugDisplayType DEBUG_DISPLAY = DebugDisplayType.BOUNDING_BOX;

    private static final boolean APPROXIMATE_CAMERA_MATRIX = true; // TODO: Tune camera matrix
    private static final boolean FLIP_Y_AXIS = true;

    // Default image resolution, in pixels
    private static final int DEFAULT_WIDTH = 320;
    private static final int DEFAULT_HEIGHT = 240;

    // These are for the Raspberry Pi Camera v2
    private static final double CAMERA_FOV_X = 62.2;
    private static final double CAMERA_FOV_Y = 48.8;

    // These were calculated using the game manual specs on vision target
    // Origin is center of bounding box
    // Order is leftbottomcorner, lefttopcorner, rightbottomcorner, righttopcorner
    private static final Point3[] TARGET_WORLD_COORDS = new Point3[] { new Point3(-5.375, -2.9375, 0),
        new Point3(-5.9375, 2.9375, 0), new Point3(5.375, -2.9375, 0), new Point3(5.9375, 2.9375, 0) };

    // Calculated by calibrating the camera
    private static double[] CAMERA_MATRIX = new double[] { 279.23728868, 0.0, 154.82842703, 0.0, 281.04311504,
        116.78960534, 0.0, 0.0, 1.0 };

    // Calculated by calibrating the camera
    private static double[] DISTORTION_MATRIX = new double[] { 0.07558564, -0.2477479, 0.00454396, -0.0029934,
        0.42422577 };

    public static void main(String[] args)
    {
        // Load the C++ native code
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        RaspiVision vision = new RaspiVision();
        vision.start();
    }

    private Gson gson;
    private Thread visionThread;
    private Thread calcThread;
    private Thread cameraThread;

    private NetworkTableEntry visionData;
    private NetworkTableEntry cameraData;

    private int numFrames = 0;
    private double startTime = 0;
    private CvSource dashboardDisplay;

    private int width, height; // in pixels

    private final Object dataLock = new Object();
    private TargetData targetData = null;

    private final Object imageLock = new Object();
    private VisionTargetPipeline pipeline;
    private UsbCamera camera;
    private Mat image = null;

    // Instantiating Mats are expensive, so do it all up here, and just use the put methods.
    private MatOfDouble dist;
    private MatOfPoint2f imagePoints = new MatOfPoint2f();
    private MatOfPoint3f worldPoints = new MatOfPoint3f(TARGET_WORLD_COORDS);
    private Mat cameraMat = Mat.zeros(3, 3, CvType.CV_64F);
    private Mat rotationVector = new Mat();
    private Mat translationVector = new Mat();
    private Mat rotationMatrix = new Mat();
    private MatOfPoint2f projectedPoints = new MatOfPoint2f();
    private MatOfPoint3f pointToProject = new MatOfPoint3f();
    private MatOfPoint contourPoints = new MatOfPoint();

    public RaspiVision()
    {
        gson = new Gson();

        NetworkTableInstance instance = NetworkTableInstance.getDefault();
        if (SERVER)
        {
            System.out.print("Initializing server...");
            instance.startServer();
            System.out.println("Done!");
        }
        else
        {
            System.out.print("Connecting to server...");
            instance.startClientTeam(TEAM_NUMBER);
            System.out.println("Done!");
        }

        System.out.print("Initializing vision...");

        NetworkTable table = instance.getTable("RaspiVision");
        NetworkTableEntry cameraConfig = table.getEntry("CameraConfig");
        visionData = table.getEntry("VisionData");
        cameraData = table.getEntry("CameraData");
        NetworkTableEntry hueLow = table.getEntry("HueLow");
        NetworkTableEntry hueHigh = table.getEntry("HueHigh");
        NetworkTableEntry satLow = table.getEntry("SatLow");
        NetworkTableEntry satHigh = table.getEntry("SatHigh");
        NetworkTableEntry luminanceLow = table.getEntry("LuminanceLow");
        NetworkTableEntry luminanceHigh = table.getEntry("LuminanceHigh");

        cameraData.setDoubleArray(new double[] { DEFAULT_WIDTH, DEFAULT_HEIGHT });

        if (DEBUG_DISPLAY != DebugDisplayType.NONE)
        {
            dashboardDisplay = CameraServer.getInstance().putVideo("RaspiVision", DEFAULT_WIDTH, DEFAULT_HEIGHT);
        }

        if (!APPROXIMATE_CAMERA_MATRIX)
        {
            cameraMat.put(0, 0, CAMERA_MATRIX);
            dist = new MatOfDouble(DISTORTION_MATRIX);
        }
        else
        {
            dist = new MatOfDouble(0, 0, 0, 0);
        }

        cameraThread = new Thread(this::cameraCaptureThread);
        calcThread = new Thread(this::calculationThread);

        camera = CameraServer.getInstance().startAutomaticCapture();
        camera.setResolution(DEFAULT_WIDTH, DEFAULT_HEIGHT); // Default to 320x240, unless overridden by json config
        camera.setBrightness(40);
        pipeline = new VisionTargetPipeline();
        visionThread = new Thread(this::visionProcessingThread);
        visionThread.setDaemon(false);

        int flag = EntryListenerFlags.kNew | EntryListenerFlags.kUpdate;

        hueHigh.setDouble(pipeline.hslThresholdHue[1]);
        hueHigh.addListener(event -> pipeline.hslThresholdHue[1] = event.value.getDouble(), flag);
        hueLow.setDouble(pipeline.hslThresholdHue[0]);
        hueLow.addListener(event -> pipeline.hslThresholdHue[0] = event.value.getDouble(), flag);

        satHigh.setDouble(pipeline.hslThresholdSaturation[1]);
        satHigh.addListener(event -> pipeline.hslThresholdSaturation[1] = event.value.getDouble(), flag);
        satLow.setDouble(pipeline.hslThresholdSaturation[0]);
        satLow.addListener(event -> pipeline.hslThresholdSaturation[0] = event.value.getDouble(), flag);

        luminanceHigh.setDouble(pipeline.hslThresholdLuminance[1]);
        luminanceHigh.addListener(event -> pipeline.hslThresholdLuminance[1] = event.value.getDouble(), flag);
        luminanceLow.setDouble(pipeline.hslThresholdLuminance[0]);
        luminanceLow.addListener(event -> pipeline.hslThresholdLuminance[0] = event.value.getDouble(), flag);

        cameraConfig.addListener(event -> configCamera(camera, event.value.getString()),
            EntryListenerFlags.kNew | EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate);

        System.out.println("Done!\nInitialization complete!");
    }

    private void configCamera(UsbCamera camera, String json)
    {
        System.out.print("Configuring camera...");
        if (!camera.setConfigJson(json))
        {
            System.out.println();
            System.err.println("Invalid json configuration file!");
        }
        else
        {
            System.out.println("Done!");
        }
    }

    public void start()
    {
        System.out.print("Starting vision thread...");
        cameraThread.start();
        visionThread.start();
        calcThread.start();
        startTime = getTime();
        System.out.println("Done!");
    }

    private void cameraCaptureThread()
    {
        CvSink sink = new CvSink("RaspiVision");
        sink.setSource(camera);
        Mat image = new Mat();
        while (!Thread.interrupted())
        {
            long response = sink.grabFrame(image);
            if (response != 0L)
            {
                Mat frame = image.clone();
                synchronized (imageLock)
                {
                    this.image = frame;
                    imageLock.notify();
                }
            }
            else
            {
                System.err.println("Camera Error: " + sink.getError());
            }
        }
        sink.close();
    }

    private void visionProcessingThread()
    {
        try
        {
            Mat image = null;
            while (!Thread.interrupted())
            {
                synchronized (imageLock)
                {
                    while (this.image == image)
                    {
                        imageLock.wait();
                    }
                    image = this.image;
                }
                pipeline.process(image);
                processImage(pipeline);
                // I don't need the image anymore, so release the memory
                image.release();
            }
        }
        catch (InterruptedException e)
        {
            e.printStackTrace();
        }
    }

    private void calculationThread()
    {
        TargetData data = null;
        try
        {
            while (!Thread.interrupted())
            {
                synchronized (dataLock)
                {
                    while (targetData == data)
                    {
                        dataLock.wait();
                    }
                    data = targetData;
                }
                String dataString = "";
                if (data != null)
                {
                    RelativePose pose = calculateRelativePose(data);
                    dataString = gson.toJson(pose);
                }
                visionData.setString(dataString);
                // If fps counter is enabled, calculate fps
                // TODO: Measure fps even if data is null, since null data isn't fresh, so the fps seems to drop.
                if (MEASURE_FPS)
                {
                    measureFps();
                }
            }
        }
        catch (InterruptedException e)
        {
            e.printStackTrace();
        }
    }

    private void processImage(VisionTargetPipeline pipeline)
    {
        // If the resolution changed, update the camera data network tables entry
        if (width != pipeline.getInput().width() || height != pipeline.getInput().height())
        {
            width = pipeline.getInput().width();
            height = pipeline.getInput().height();
            cameraData.setDoubleArray(new double[] { width, height });
            // TODO: Should this be separate x and y focal lengths, or the average? test.
            if (APPROXIMATE_CAMERA_MATRIX)
            {
                double focalLengthX = (width / 2.0) / (Math.tan(Math.toRadians(CAMERA_FOV_X / 2.0)));
                double focalLengthY = (height / 2.0) / (Math.tan(Math.toRadians(CAMERA_FOV_Y / 2.0)));
                cameraMat.put(0, 0, focalLengthX, 0, width / 2.0, 0, focalLengthY, height / 2.0, 0, 0, 1);
            }
        }
        // Get the selected target from the pipeline
        TargetData data = pipeline.getSelectedTarget();
        synchronized (dataLock)
        {
            this.targetData = data;
            dataLock.notify();
        }

        // If debug display is enabled, render it
        if (DEBUG_DISPLAY == DebugDisplayType.BOUNDING_BOX || DEBUG_DISPLAY == DebugDisplayType.MASK
            || DEBUG_DISPLAY == DebugDisplayType.REGULAR)
        {
            debugDisplay(pipeline);
        }
    }

    private void debugDisplay(VisionTargetPipeline pipeline)
    {
        Mat image;
        Scalar color = new Scalar(0, 255, 0);
        if (DEBUG_DISPLAY == DebugDisplayType.MASK)
        {
            image = pipeline.getHslThresholdOutput();
            color = new Scalar(255);
        }
        else
        {
            image = pipeline.getInput();
        }
        if (DEBUG_DISPLAY == DebugDisplayType.BOUNDING_BOX)
        {
            image = image.clone();
            for (TargetData data : pipeline.getDetectedTargets())
            {
                if (data != null)
                {
                    int minX = data.x - data.w / 2;
                    int maxX = data.x + data.w / 2;
                    int minY = data.y - data.h / 2;
                    int maxY = data.y + data.h / 2;
                    Imgproc.rectangle(image, new Point(minX, minY), new Point(maxX, maxY), color, 2);
                }
            }
        }
        dashboardDisplay.putFrame(image);
    }

    private void measureFps()
    {
        numFrames++;
        double currTime = getTime();
        double elapsedTime = currTime - startTime;
        if (elapsedTime >= FPS_AVG_WINDOW)
        {
            double fps = (double) numFrames / elapsedTime;
            System.out.printf("Avg fps over %.3fsec: %.3f\n", elapsedTime, fps);
            numFrames = 0;
            startTime = currTime;
        }
    }

    private double getTime()
    {
        return (double) System.currentTimeMillis() / 1000;
    }

    private Point[] getImagePoints(TargetData data, MatOfPoint2f imagePoints)
    {
        // Calculate the corners of the left vision target
        Point[] leftPoints = new Point[4];
        data.leftTarget.rotatedRect.points(leftPoints);
        Point leftBottomCorner = Arrays.stream(leftPoints).max(Comparator.comparing(point -> point.y))
            .orElseThrow(IllegalStateException::new);
        Point leftTopCorner = Arrays.stream(leftPoints).min(Comparator.comparing(point -> point.y))
            .orElseThrow(IllegalStateException::new);

        // Calculate the corners of the right vision target
        Point[] rightPoints = new Point[4];
        data.rightTarget.rotatedRect.points(rightPoints);
        Point rightBottomCorner = Arrays.stream(rightPoints).max(Comparator.comparing(point -> point.y))
            .orElseThrow(IllegalStateException::new);
        Point rightTopCorner = Arrays.stream(rightPoints).min(Comparator.comparing(point -> point.y))
            .orElseThrow(IllegalStateException::new);

        Point[] points = new Point[] { leftBottomCorner, leftTopCorner, rightBottomCorner, rightTopCorner };

        if (FLIP_Y_AXIS)
        {
            // Invert the y axis of the image points. This is an in-place operation, so the MatOfPoint doesn't need to be updated.
            for (Point point : points)
            {
                point.y = height - point.y;
            }
        }

        imagePoints.fromArray(points);

        return points;
    }

    private RelativePose calculateRelativePose(TargetData data)
    {
        // Get the image points
        Point[] points = getImagePoints(data, imagePoints);

        // Use the black magic of the Ancient Ones to get the rotation and translation vectors
        Calib3d.solvePnP(worldPoints, imagePoints, cameraMat, dist, rotationVector, translationVector);
        // Get the distances in the x and z axes. (or in robot space, x and y)
        double x = translationVector.get(0, 0)[0];
        double z = translationVector.get(2, 0)[0];
        // Convert x,y to r,theta
        double distance = Math.sqrt(x * x + z * z);
        double heading = Math.toDegrees(Math.atan2(x, z));

        // Convert the axis-angle rotation vector into a rotation matrix
        Calib3d.Rodrigues(rotationVector, rotationMatrix);

        // Convert the rotation matrix to euler angles
        double[] angles = convertRotMatrixToEulerAngles(rotationMatrix);
        // Convert the yaw to actual yaw with my fuckit (tm) method.
        double objectYaw = angles[1]; // yawMapper.applyAsDouble(angles[1]);

        // Write to the debug display, if necessary
        if (DEBUG_DISPLAY == DebugDisplayType.FULL_PNP || DEBUG_DISPLAY == DebugDisplayType.CORNERS)
        {
            Mat image = pipeline.getInput().clone();

            // Draw the contours first, so the corners get put on top
            if (DEBUG_DISPLAY == DebugDisplayType.FULL_PNP)
            {
                List<MatOfPoint> contours = new ArrayList<>();
                contourPoints.fromArray(data.leftTarget.contour.toArray());
                contours.add(contourPoints);
                Imgproc.drawContours(image, contours, 0, new Scalar(255, 0, 255), 2);
                contourPoints.fromArray(data.rightTarget.contour.toArray());
                Imgproc.drawContours(image, contours, 0, new Scalar(255, 0, 255), 2);
            }

            // Draw the left and right target corners. First you have to re-flip the y coordinate
            for (Point point : points)
            {
                if (FLIP_Y_AXIS)
                {
                    point.y = height - point.y;
                }
                Imgproc.circle(image, point, 1, new Scalar(0, 255, 255), 2);
            }

            if (DEBUG_DISPLAY == DebugDisplayType.FULL_PNP)
            {
                // Project the XYZ axes out and draw the lines to show orientation
                Point origin = new Point(data.x, data.y);
                projectAndDrawAxes(image, origin, new Point3(10, 0, 0), rotationVector, translationVector, cameraMat,
                    dist, new Scalar(0, 0, 255));
                projectAndDrawAxes(image, origin, new Point3(0, 10, 0), rotationVector, translationVector, cameraMat,
                    dist, new Scalar(0, 255, 0));
                projectAndDrawAxes(image, origin, new Point3(0, 0, 10), rotationVector, translationVector, cameraMat,
                    dist, new Scalar(255, 0, 0));
            }

            dashboardDisplay.putFrame(image);
            image.release();
        }

        return new RelativePose(heading, distance, objectYaw);
    }

    private double[] convertRotMatrixToEulerAngles(Mat rotationMatrix)
    {
        double r00 = rotationMatrix.get(0, 0)[0];
        double r10 = rotationMatrix.get(1, 0)[0];
        double sy = Math.sqrt(r00 * r00 + r10 * r10);
        double x, y, z;
        if (sy >= 1e-6)
        {
            x = Math.atan2(rotationMatrix.get(2, 1)[0], rotationMatrix.get(2, 2)[0]);
            y = Math.atan2(-rotationMatrix.get(2, 0)[0], sy);
            z = Math.atan2(r10, r00);
        }
        else
        {
            x = Math.atan2(-rotationMatrix.get(1, 2)[0], rotationMatrix.get(1, 1)[0]);
            y = Math.atan2(-rotationMatrix.get(2, 0)[0], sy);
            z = 0;
        }
        return new double[] { Math.toDegrees(x), Math.toDegrees(y), Math.toDegrees(z) };
    }

    private void projectAndDrawAxes(Mat image, Point origin, Point3 point, Mat rotationVector, Mat translationVector,
        Mat cameraMat, MatOfDouble dist, Scalar color)
    {
        pointToProject.fromArray(point);
        Calib3d.projectPoints(pointToProject, rotationVector, translationVector, cameraMat, dist, projectedPoints);
        Point toDraw = projectedPoints.toArray()[0];
        if (FLIP_Y_AXIS)
        {
            // Re-flip the y axis
            toDraw.y = height - toDraw.y;
        }
        Imgproc.line(image, origin, toDraw, color, 2);
    }

    private class RelativePose
    {
        public double r;
        public double theta;
        public double objectYaw;
        public double x;
        public double y;

        public RelativePose(double xDistance, double zDistance, double objectYaw)
        {
            this.x = xDistance;
            // In robot code, y is forward, not z
            this.y = zDistance;
            this.objectYaw = objectYaw;
            this.r = Math.sqrt(xDistance * xDistance + zDistance * zDistance);
            this.theta = Math.toDegrees(Math.atan2(xDistance, zDistance));
        }
    }
}
