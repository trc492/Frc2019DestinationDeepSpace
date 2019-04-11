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
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.opencv.calib3d.Calib3d;
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
    private static final DebugDisplayType DEBUG_DISPLAY = DebugDisplayType.MASK;

    private static final boolean APPROXIMATE_CAMERA_MATRIX = true;
    private static final boolean FLIP_Y_AXIS = true;

    private Gson gson;
    private Thread visionThread;
    private Thread calcThread;
    private Thread cameraThread;

    private NetworkTableInstance instance;
    private NetworkTableEntry visionData;

    private int numFrames = 0;
    private double startTime = 0;
    private CvSource dashboardDisplay;

    private volatile double cameraPitch = 0.0;

    private volatile boolean useDebugDisplay;

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
    private MatOfPoint3f worldPoints = new MatOfPoint3f(CameraConstants.TARGET_WORLD_COORDS);
    private Mat cameraMat = Mat.zeros(3, 3, CvType.CV_64F);
    private Mat rotationVector = new Mat();
    private Mat translationVector = new Mat();
    private Mat rotationMatrix = new Mat();
    private MatOfPoint2f projectedPoints = new MatOfPoint2f();
    private MatOfPoint3f pointToProject = new MatOfPoint3f();
    private MatOfPoint contourPoints = new MatOfPoint();

    public RaspiVision(int cameraIndex)
    {
        gson = new Gson();

        instance = NetworkTableInstance.getDefault();
        if (SERVER)
        {
            System.out.print("Initializing server...");
            instance.startServer();
            System.out.println("Done!");
        }
        else
        {
            System.out.print("Connecting to server...");
            boolean done = false;
            while (!done)
            {
                instance.startClient("10.4.92.2");
                try
                {
                    Thread.sleep(1000);
                }
                catch (InterruptedException e)
                {
                    e.printStackTrace();
                }
                done = instance.isConnected();
                if (!done)
                {
                    System.out.print("\nConnection failed! Retrying...");
                }
            }
            System.out.println("Done!");
        }

        System.out.print("Initializing vision...");

        NetworkTable table = instance.getTable("RaspiVision");
        NetworkTableEntry cameraConfig = table.getEntry("CameraConfig");
        visionData = table.getEntry("VisionData");
        NetworkTableEntry useDebugDisplay = table.getEntry("UseDebugDisplay");
        NetworkTableEntry hueLow = table.getEntry("HueLow");
        NetworkTableEntry hueHigh = table.getEntry("HueHigh");
        NetworkTableEntry satLow = table.getEntry("SatLow");
        NetworkTableEntry satHigh = table.getEntry("SatHigh");
        NetworkTableEntry valueLow = table.getEntry("ValueLow");
        NetworkTableEntry valueHigh = table.getEntry("ValueHigh");
        NetworkTableEntry ratioLow = table.getEntry("RatioLow");
        NetworkTableEntry ratioHigh = table.getEntry("RatioHigh");
        NetworkTableEntry cameraPitch = table.getEntry("CameraPitch");

        useDebugDisplay.addListener(event -> this.useDebugDisplay = event.value.isBoolean() && event.value.getBoolean(),
            EntryListenerFlags.kNew | EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate);
        useDebugDisplay.setBoolean(useDebugDisplay.getBoolean(false));

        cameraPitch.addListener(event -> this.cameraPitch = event.value.isDouble() ? event.value.getDouble() : 0.0,
            EntryListenerFlags.kNew | EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate);

        if (DEBUG_DISPLAY != DebugDisplayType.NONE)
        {
            dashboardDisplay = CameraServer.getInstance()
                .putVideo("RaspiVision", CameraConstants.DEFAULT_WIDTH, CameraConstants.DEFAULT_HEIGHT);
        }

        if (!APPROXIMATE_CAMERA_MATRIX)
        {
            cameraMat.put(0, 0, CameraConstants.CAMERA_MATRIX);
            dist = new MatOfDouble(CameraConstants.DISTORTION_MATRIX);
        }
        else
        {
            dist = new MatOfDouble(0, 0, 0, 0);
        }

        cameraThread = new Thread(this::cameraCaptureThread);
        calcThread = new Thread(this::calculationThread);

        camera = CameraServer.getInstance().startAutomaticCapture(cameraIndex);
        if (CameraConstants.correctlyInitialized && !"".equals(CameraConstants.defaultJsonConfig.strip()))
        {
            cameraConfig.setString(CameraConstants.defaultJsonConfig);
        }
        else
        {
            camera.setResolution(CameraConstants.DEFAULT_WIDTH,
                CameraConstants.DEFAULT_HEIGHT); // Default to 320x240, unless overridden by json config
            camera.setBrightness(CameraConstants.DEFAULT_BRIGHTNESS);
        }
        pipeline = new VisionTargetPipeline();
        visionThread = new Thread(this::visionProcessingThread);
        visionThread.setDaemon(false);

        int flag = EntryListenerFlags.kNew | EntryListenerFlags.kUpdate;

        hueHigh.setDouble(pipeline.hsvThresholdHue[1]);
        hueHigh.addListener(event -> pipeline.hsvThresholdHue[1] = getDouble(event), flag);
        hueLow.setDouble(pipeline.hsvThresholdHue[0]);
        hueLow.addListener(event -> pipeline.hsvThresholdHue[0] = getDouble(event), flag);

        satHigh.setDouble(pipeline.hsvThresholdSaturation[1]);
        satHigh.addListener(event -> pipeline.hsvThresholdSaturation[1] = getDouble(event), flag);
        satLow.setDouble(pipeline.hsvThresholdSaturation[0]);
        satLow.addListener(event -> pipeline.hsvThresholdSaturation[0] = getDouble(event), flag);

        valueHigh.setDouble(pipeline.hsvThresholdValue[1]);
        valueHigh.addListener(event -> pipeline.hsvThresholdValue[1] = getDouble(event), flag);
        valueLow.setDouble(pipeline.hsvThresholdValue[0]);
        valueLow.addListener(event -> pipeline.hsvThresholdValue[0] = getDouble(event), flag);

        ratioLow.setDouble(pipeline.rotatedRectRatioMin);
        ratioLow.addListener(event -> pipeline.rotatedRectRatioMin = getDouble(event), flag);

        ratioHigh.setDouble(pipeline.rotatedRectRatioMax);
        ratioHigh.addListener(event -> pipeline.rotatedRectRatioMax = getDouble(event), flag);

        cameraConfig.addListener(event -> configCamera(camera, event.value.getString()),
            EntryListenerFlags.kNew | EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate);

        System.out.println("Done!\nInitialization complete!");
    }

    private double getDouble(EntryNotification event)
    {
        return event.value.isDouble() ? event.value.getDouble() : 0.0;
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
                processImage(image, pipeline);
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
                instance.flush(); // Write all the network table data
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

    private void processImage(Mat frame, VisionTargetPipeline pipeline)
    {
        // If the resolution changed, update the camera data network tables entry
        if (width != frame.width() || height != frame.height())
        {
            width = pipeline.getInput().width();
            height = pipeline.getInput().height();
            if (APPROXIMATE_CAMERA_MATRIX)
            {
                double focalLengthX = (width / 2.0) / (Math.tan(Math.toRadians(CameraConstants.CAMERA_FOV_X / 2.0)));
                double focalLengthY = (height / 2.0) / (Math.tan(Math.toRadians(CameraConstants.CAMERA_FOV_Y / 2.0)));
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
        if (useDebugDisplay && (DEBUG_DISPLAY == DebugDisplayType.BOUNDING_BOX || DEBUG_DISPLAY == DebugDisplayType.MASK
            || DEBUG_DISPLAY == DebugDisplayType.REGULAR))
        {
            debugDisplay(pipeline);
        }
    }

    private void debugDisplay(VisionTargetPipeline pipeline)
    {
        Mat image;
        Scalar color = new Scalar(0, 255, 0);
        boolean release = false;
        if (DEBUG_DISPLAY == DebugDisplayType.MASK)
        {
            image = pipeline.getHsvThresholdOutput();
            color = new Scalar(255);
        }
        else
        {
            image = pipeline.getInput();
        }
        if (DEBUG_DISPLAY == DebugDisplayType.BOUNDING_BOX)
        {
            release = true; // Release the image later if we clone it.
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
        if (release)
        {
            image.release();
        }
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
        double zRot = translationVector.get(2, 0)[0]; // This is uncorrected for pitch

        double z = Math.cos(Math.toRadians(cameraPitch)) * zRot;

        // Convert the axis-angle rotation vector into a rotation matrix
        Calib3d.Rodrigues(rotationVector, rotationMatrix);

        // Convert the rotation matrix to euler angles
        double[] angles = convertRotMatrixToEulerAngles(rotationMatrix);
        double objectYaw = angles[1];

        // Write to the debug display, if necessary
        if (useDebugDisplay && (DEBUG_DISPLAY == DebugDisplayType.FULL_PNP || DEBUG_DISPLAY == DebugDisplayType.CORNERS))
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

        return new RelativePose(x, z, objectYaw);
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
