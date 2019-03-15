package raspivision;

import com.google.gson.Gson;
import org.opencv.core.Point3;

import java.io.BufferedReader;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStreamReader;

public class CameraConstants
{
    // Default image resolution, in pixels
    public static int DEFAULT_WIDTH;
    public static int DEFAULT_HEIGHT;
    // These are for the Raspberry Pi Camera v2
    public static final double CAMERA_FOV_X = 62.2;
    public static final double CAMERA_FOV_Y = 48.8;
    public static final double CROSSHAIR_X_ANGLE = -7.125; // Angle in degrees from camera to end effector
    // These were calculated using the game manual specs on vision target
    // Origin is center of bounding box
    // Order is leftbottomcorner, lefttopcorner, rightbottomcorner, righttopcorner
    public static final Point3[] TARGET_WORLD_COORDS = new Point3[] { new Point3(-5.375, -2.9375, 0),
        new Point3(-5.9375, 2.9375, 0), new Point3(5.375, -2.9375, 0), new Point3(5.9375, 2.9375, 0) };
    public static String defaultJsonConfig;
    // Calculated by calibrating the camera
    public static double[] CAMERA_MATRIX = new double[] { 223.01258757, 0.0, 153.16212363, 0.0, 222.23760939,
        126.9374426, 0.0, 0.0, 1.0 };
    // Calculated by calibrating the camera
    public static double[] DISTORTION_MATRIX = new double[] { 0.21826744, -0.60425638, 0.00453213, -0.00482952,
        0.40065646 };

    static
    {
        try (BufferedReader in = new BufferedReader(new InputStreamReader(new FileInputStream("config.json"))))
        {
            StringBuilder sb = new StringBuilder();
            String s;
            while((s = in.readLine()) != null)
            {
                sb.append(s);
            }
            defaultJsonConfig = sb.toString();
            Gson gson = new Gson();
            Resolution resolution = gson.fromJson(defaultJsonConfig, Resolution.class);
            DEFAULT_WIDTH = resolution.width;
            DEFAULT_HEIGHT = resolution.height;
        }
        catch (IOException e)
        {
            DEFAULT_WIDTH = 320;
            DEFAULT_HEIGHT = 240;
            defaultJsonConfig = "";
            e.printStackTrace();
        }
    }

    private static class Resolution
    {
        public int width, height;
    }
}
