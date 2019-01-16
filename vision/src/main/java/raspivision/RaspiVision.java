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
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

public class RaspiVision
{
    private static final int TEAM_NUMBER = 492;
    private static final boolean SERVER = true; // true for debugging only
    private static final boolean MEASURE_FPS = true;
    private static final double FPS_AVG_WINDOW = 5; // 5 seconds
    private static final boolean DEBUG_DISPLAY = false;

    private static final int DEFAULT_WIDTH = 320;
    private static final int DEFAULT_HEIGHT = 240;

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

        cameraData.setDoubleArray(new double[] { DEFAULT_WIDTH, DEFAULT_HEIGHT });

        if (DEBUG_DISPLAY)
        {
            dashboardDisplay = CameraServer.getInstance().putVideo("RaspiVision", DEFAULT_WIDTH, DEFAULT_HEIGHT);
        }

        UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
        camera.setResolution(DEFAULT_WIDTH, DEFAULT_HEIGHT); // Default to 320x240, unless overridden by json config
        visionThread = new VisionThread(camera, new VisionTargetPipeline(), this::processImage);
        visionThread.setDaemon(false);

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
        }
        // Get the selected target from the pipeline
        TargetData data = pipeline.getSelectedTarget();
        String dataString = "";
        if (data != null)
        {
            dataString = gson.toJson(data);
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
                Imgproc.rectangle(image, new Point(minX, minY), new Point(maxX, maxY), new Scalar(0, 255, 0));
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
}
