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
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.vision.VisionThread;
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

    private static final int DEFAULT_WIDTH = 320;
    private static final int DEFAULT_HEIGHT = 240;

    // From the raspberry pi camera spec sheet:
    private static final double CAMERA_FOV_X = 62.2;
    private static final double CAMERA_FOV_Y = 48.8;

    private static final Point3[] TARGET_WORLD_COORDS = new Point3[]{
        new Point3(-7.3125, -2.4375, 0),
        new Point3(-4.0,2.4375,0),
        new Point3(-5.375,-2.9375,0),
        new Point3(-5.9375,2.9375,0),
        new Point3(4.0,2.4375,0),
        new Point3(7.3125, -2.4375, 0),
        new Point3(5.375,-2.9375,0),
        new Point3(5.9375,2.9375,0)
    };

    public static void main(String[] args)
    {
        RaspiVision vision = new RaspiVision();
        vision.start();
    }

    private Gson gson;
    private VisionThread visionThread;
    private NetworkTableEntry visionData;
    private NetworkTableEntry cameraData;
    private int numFrames = 0;
    private double startTime = 0;
    private CvSource dashboardDisplay;
    private int width, height;
    private double focalLength; // In pixels
    private VisionTargetPipeline pipeline;

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

        UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
        camera.setResolution(DEFAULT_WIDTH, DEFAULT_HEIGHT); // Default to 320x240, unless overridden by json config
        visionThread = new VisionThread(camera, pipeline = new VisionTargetPipeline(), this::processImage);
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
        visionThread.start();
        startTime = getTime();
        System.out.println("Done!");
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
        String dataString = "";
        if (data != null)
        {
            dataString = gson.toJson(new RelativePose(data));
        }
        visionData.setString(dataString);

        // If debug display is enabled, render it
        if (DEBUG_DISPLAY)
        {
            debugDisplay(pipeline);
        }
        // If fps counter is enabled, calculate fps
        if (MEASURE_FPS)
        {
            measureFps();
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

        public RelativePose(TargetData data)
        {
            List<Point> leftContour = data.leftTarget.contour.toList();
            Point leftMinX = leftContour.stream().min(Comparator.comparingDouble(c -> c.x))
                .orElseThrow(IllegalStateException::new);
            Point leftMaxX = leftContour.stream().max(Comparator.comparingDouble(c -> c.x))
                .orElseThrow(IllegalStateException::new);
            Point leftMinY = leftContour.stream().min(Comparator.comparingDouble(c -> c.y))
                .orElseThrow(IllegalStateException::new);
            Point leftMaxY = leftContour.stream().max(Comparator.comparingDouble(c -> c.y))
                .orElseThrow(IllegalStateException::new);

            List<Point> rightContour = data.rightTarget.contour.toList();
            Point rightMinX = rightContour.stream().min(Comparator.comparingDouble(c -> c.x))
                .orElseThrow(IllegalStateException::new);
            Point rightMaxX = rightContour.stream().max(Comparator.comparingDouble(c -> c.x))
                .orElseThrow(IllegalStateException::new);
            Point rightMinY = rightContour.stream().min(Comparator.comparingDouble(c -> c.y))
                .orElseThrow(IllegalStateException::new);
            Point rightMaxY = rightContour.stream().max(Comparator.comparingDouble(c -> c.y))
                .orElseThrow(IllegalStateException::new);

            MatOfPoint2f imagePoints = new MatOfPoint2f(leftMinX, leftMaxX, leftMinY, leftMaxY, rightMinX, rightMaxX,
                rightMinY, rightMaxY);
            MatOfPoint3f worldPoints = new MatOfPoint3f(TARGET_WORLD_COORDS);
            Mat cameraMat = new Mat(3,3,CvType.CV_32FC1);
            cameraMat.put(0,0, focalLength, 0, width/2.0, 0, focalLength, height/2.0, 0,0,1);
            MatOfDouble dist = new MatOfDouble(0,0,0,0);

            Mat rotationVector = new Mat();
            Mat translationVector = new Mat();
            Calib3d.solvePnP(worldPoints, imagePoints, cameraMat, dist, rotationVector, translationVector);
            double x = translationVector.get(0,0)[0];
            double z = translationVector.get(2,0)[0];
            distance = Math.sqrt(x*x + z*z);
            heading = Math.toDegrees(Math.atan2(x,z));
        }
    }
}
