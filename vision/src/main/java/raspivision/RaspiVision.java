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
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.Comparator;
import java.util.List;

public class RaspiVision
{
    private static final int TEAM_NUMBER = 492;
    private static final boolean SERVER = true; // true for debugging only
    private static final boolean MEASURE_FPS = true;
    private static final double FPS_AVG_WINDOW = 5; // 5 seconds
    private static final boolean DEBUG_DISPLAY = true;

    // Default image resolution, in pixels
    private static final int DEFAULT_WIDTH = 320;
    private static final int DEFAULT_HEIGHT = 240;

    // From the raspberry pi camera spec sheet, in degrees:
    private static final double CAMERA_FOV_X = 62.2;
    private static final double CAMERA_FOV_Y = 48.8;

    // These were calculated using the game manual specs on vision target
    // Origin is center of bounding box
    // Order is leftminx, leftmaxx, leftminy, leftmaxy, rightminx, rightmaxx, rightminy, rightmaxy
    private static final Point3[] TARGET_WORLD_COORDS = new Point3[] { new Point3(-7.3125, -2.4375, 0),
        new Point3(-4.0, 2.4375, 0), new Point3(-5.375, -2.9375, 0), new Point3(-5.9375, 2.9375, 0),
        new Point3(4.0, 2.4375, 0), new Point3(7.3125, -2.4375, 0), new Point3(5.375, -2.9375, 0),
        new Point3(5.9375, 2.9375, 0) };

    public static void main(String[] args)
    {
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
    private double focalLength; // In pixels

    private final Object dataLock = new Object();
    private TargetData targetData = null;

    private final Object imageLock = new Object();
    private VisionTargetPipeline pipeline;
    private UsbCamera camera;
    private Mat image;

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

        if (DEBUG_DISPLAY)
        {
            dashboardDisplay = CameraServer.getInstance().putVideo("RaspiVision", DEFAULT_WIDTH, DEFAULT_HEIGHT);
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
        image = new Mat();
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
                System.err.println(sink.getError());
            }
        }
    }

    private void visionProcessingThread()
    {
        try
        {
            Mat image = this.image;
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
                    RelativePose pose = new RelativePose(data);
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
            double focalLengthX = (width / 2.0) / (Math.tan(Math.toRadians(CAMERA_FOV_X / 2.0)));
            double focalLengthY = (height / 2.0) / (Math.tan(Math.toRadians(CAMERA_FOV_Y / 2.0)));
            focalLength = (focalLengthX + focalLengthY) / 2.0;
        }
        // Get the selected target from the pipeline
        TargetData data = pipeline.getSelectedTarget();
        synchronized (dataLock)
        {
            this.targetData = data;
            dataLock.notify();
        }

        // If debug display is enabled, render it
        if (DEBUG_DISPLAY)
        {
            debugDisplay(pipeline);
        }
    }

    private void debugDisplay(VisionTargetPipeline pipeline)
    {
        Mat image = pipeline.getInput().clone();
        for (TargetData data : pipeline.getDetectedTargets())
        {
            if (data != null)
            {
                int minX = data.x - data.w / 2;
                int maxX = data.x + data.w / 2;
                int minY = data.y - data.h / 2;
                int maxY = data.y + data.h / 2;
                Imgproc.rectangle(image, new Point(minX, minY), new Point(maxX, maxY), new Scalar(0, 255, 0), 2);
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

    private class RelativePose
    {
        public double heading;
        public double distance;

        /**
         * This calculates the pose relative to the vision target in r, theta. It uses the solvePNP algorithm from
         * OpenCV.
         *
         * @param data The target data container to use for the calculations.
         */
        public RelativePose(TargetData data)
        {
            // Calculate the corners of the left vision target
            List<Point> leftContour = data.leftTarget.contour.toList();
            Point leftMinX = leftContour.stream().min(Comparator.comparingDouble(c -> c.x))
                .orElseThrow(IllegalStateException::new);
            Point leftMaxX = leftContour.stream().max(Comparator.comparingDouble(c -> c.x))
                .orElseThrow(IllegalStateException::new);
            Point leftMinY = leftContour.stream().min(Comparator.comparingDouble(c -> c.y))
                .orElseThrow(IllegalStateException::new);
            Point leftMaxY = leftContour.stream().max(Comparator.comparingDouble(c -> c.y))
                .orElseThrow(IllegalStateException::new);

            // Calculate the corners of the right vision target
            List<Point> rightContour = data.rightTarget.contour.toList();
            Point rightMinX = rightContour.stream().min(Comparator.comparingDouble(c -> c.x))
                .orElseThrow(IllegalStateException::new);
            Point rightMaxX = rightContour.stream().max(Comparator.comparingDouble(c -> c.x))
                .orElseThrow(IllegalStateException::new);
            Point rightMinY = rightContour.stream().min(Comparator.comparingDouble(c -> c.y))
                .orElseThrow(IllegalStateException::new);
            Point rightMaxY = rightContour.stream().max(Comparator.comparingDouble(c -> c.y))
                .orElseThrow(IllegalStateException::new);

            // Assemble the calculated points into a matrix
            MatOfPoint2f imagePoints = new MatOfPoint2f(leftMinX, leftMaxX, leftMinY, leftMaxY, rightMinX, rightMaxX,
                rightMinY, rightMaxY);
            // Get the target world coords in 3d space
            MatOfPoint3f worldPoints = new MatOfPoint3f(TARGET_WORLD_COORDS);
            // Create the camera matrix. This uses the calculated approximate focal length and approximates the optical
            // center as the image center.
            Mat cameraMat = new Mat(3, 3, CvType.CV_32FC1);
            cameraMat.put(0, 0, focalLength, 0, width / 2.0, 0, focalLength, height / 2.0, 0, 0, 1);
            // Assume no distortion
            MatOfDouble dist = new MatOfDouble(0, 0, 0, 0);

            // Empty matrices which will receive the vectors
            Mat rotationVector = new Mat();
            Mat translationVector = new Mat(); // This can later be used to get target rotation
            Calib3d.solvePnP(worldPoints, imagePoints, cameraMat, dist, rotationVector, translationVector);
            // Get the distances in the x and z axes. (or in robot space, x and y)
            double x = translationVector.get(0, 0)[0];
            double z = translationVector.get(2, 0)[0];
            // Convert x,y to r,theta
            distance = Math.sqrt(x * x + z * z);
            heading = Math.toDegrees(Math.atan2(x, z));
        }
    }
}
