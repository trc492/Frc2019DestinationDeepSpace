package raspivision;

import com.google.gson.Gson;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.vision.VisionThread;
import org.opencv.core.CvType;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.RotatedRect;
import org.opencv.imgproc.Imgproc;
import trclib.TrcUtil;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;

public class RaspiVision
{
    private static final int TEAM_NUMBER = 492;
    private static final boolean SERVER = true; // true for debugging only

    public static void main(String[] args)
    {
        RaspiVision vision = new RaspiVision();
        vision.start();
    }

    private Gson gson;
    private VisionThread visionThread;
    private NetworkTableEntry visionData;

    public RaspiVision()
    {
        gson = new Gson();

        System.out.println("Starting network tables!");
        NetworkTableInstance instance = NetworkTableInstance.getDefault();
        if(SERVER)
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
        NetworkTable table = instance.getTable("RaspiVision");
        NetworkTableEntry cameraConfig = table.getEntry("CameraConfig");
        visionData = table.getEntry("VisionData");

        UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
        visionThread = new VisionThread(camera, new VisionTargetPipeline(), this::processImage);
        visionThread.setDaemon(false);

        cameraConfig.addListener(event -> camera.setConfigJson(event.value.getString()),
            EntryListenerFlags.kNew | EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate);

        System.out.println("Initialization complete!");
    }

    public void start()
    {
        System.out.print("Starting vision thread...");
        visionThread.start();
        System.out.println("Done!");
    }

    private double getCorrectedAngle(RotatedRect calculatedRect)
    {
        if (calculatedRect.size.width < calculatedRect.size.height)
        {
            return calculatedRect.angle + 180;
        }
        else
        {
            return calculatedRect.angle + 90;
        }
    }

    private void processImage(VisionTargetPipeline pipeline)
    {
        try
        {
            List<MatOfPoint> contours = pipeline.convexHullsOutput();
            List<VisionTarget> targets = contours.stream().map(this::mapContourToVisionTarget)
                .sorted(this::compareVisionTargets).collect(Collectors.toCollection(ArrayList::new));
            while (!targets.get(0).isLeftTarget)
            {
                targets.remove(0);
            }
            while (targets.get(targets.size() - 1).isLeftTarget)
            {
                targets.remove(targets.size() - 1);
            }
            if (isValid(targets))
            {
                TargetData data = selectTarget(targets);
                visionData.setString(gson.toJson(data));
            }
            else
            {
                visionData.setString("");
            }
        }
        catch (Exception e)
        {
            e.printStackTrace();
            visionData.setString("");
        }
    }

    /**
     * Select a target by looking at the pair of vision targets closest to the center.
     *
     * @param targets The detected vision targets.
     * @return The target data selected.
     */
    private TargetData selectTarget(List<VisionTarget> targets)
    {
        List<TargetData> targetDatas = new ArrayList<>(); // Yes I know datas isn't a word.
        // Pair vision targets and get the enclosing bounding box.
        for (int i = 0; i < targets.size(); i += 2)
        {
            VisionTarget left = targets.get(i);
            VisionTarget right = targets.get(i + 1);
            int leftBound = left.x - left.w / 2;
            int rightBound = right.x + right.w / 2;
            int topBound = Math.max(left.y + left.h / 2, right.y + right.h / 2);
            int bottomBound = Math.min(left.y - left.h / 2, right.y - right.h / 2);
            TargetData data = new TargetData((leftBound + rightBound) / 2, (topBound + bottomBound) / 2,
                rightBound - leftBound, Math.abs(bottomBound - topBound));
            targetDatas.add(data);
        }
        return targetDatas.stream().min(Comparator.comparingInt(e -> Math.abs(e.x)))
            .orElseThrow(IllegalStateException::new);
    }

    /**
     * Checks if the list of targets is valid.
     *
     * @param targets The targets to validate.
     * @return True if valid, false otherwise.
     */
    private boolean isValid(List<VisionTarget> targets)
    {
        // Invalid if no targets or odd number of targets.
        if (targets.size() == 0 || targets.size() % 2 == 1)
        {
            return false;
        }
        // Verify that there are alternating pairs of targets. (Left right left right)
        for (int i = 0; i < targets.size(); i += 2)
        {
            VisionTarget left = targets.get(i);
            VisionTarget right = targets.get(i + 1);
            if (!left.isLeftTarget || right.isLeftTarget)
            {
                return false;
            }
        }
        return true;
    }

    private int compareVisionTargets(VisionTarget target1, VisionTarget target2)
    {
        return Integer.compare(target1.x, target2.x);
    }

    private VisionTarget mapContourToVisionTarget(MatOfPoint m)
    {
        MatOfPoint2f contour = new MatOfPoint2f();
        m.convertTo(contour, CvType.CV_32F);
        RotatedRect rect = Imgproc.minAreaRect(contour);

        VisionTarget target = new VisionTarget();
        target.isLeftTarget = getCorrectedAngle(rect) <= 90;
        target.x = TrcUtil.round(rect.center.x);
        target.y = TrcUtil.round(rect.center.y);
        target.w = TrcUtil.round(rect.size.width);
        target.h = TrcUtil.round(rect.size.height);
        return target;
    }

    /**
     * Represents a single retroflective tape.
     */
    private class VisionTarget
    {
        public boolean isLeftTarget;
        public int x, y, w, h;

        public boolean equals(Object o)
        {
            if (!(o instanceof VisionTarget))
                return false;
            VisionTarget target = (VisionTarget) o;
            return target.isLeftTarget == isLeftTarget && target.x == x && target.y == y && target.w == w
                && target.h == h;
        }
    }
}
