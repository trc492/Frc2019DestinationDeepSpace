package team492;

import com.google.gson.Gson;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

public class RaspiVision
{
    private volatile RelativePose relativePose = null;
    private Gson gson;
    private int consecutiveTargetFrames = 0;
    private int maxAverageWindow = 10; // the last 10 frames
    private List<RelativePose> frames = new LinkedList<>();

    public RaspiVision()
    {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("RaspiVision");
        NetworkTableEntry entry = table.getEntry("VisionData");
        gson = new Gson();
        entry.addListener(this::updateTargetInfo,
            EntryListenerFlags.kNew | EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate);
    }

    private void updateTargetInfo(EntryNotification event)
    {
        String info = event.value.getString();
        if ("".equals(info))
        {
            this.relativePose = null;
            if (!frames.isEmpty())
            {
                frames.clear();
            }
            consecutiveTargetFrames = 0;
        }
        else
        {
            this.relativePose = gson.fromJson(info, RelativePose.class);
            this.consecutiveTargetFrames++;
            while (frames.size() > maxAverageWindow)
            {
                frames.remove(0);
            }
        }
    }

    public RelativePose getAveragePose()
    {
        return getAveragePose(maxAverageWindow);
    }

    public RelativePose getAveragePose(int numFrames)
    {
        int fromIndex = Math.max(0, frames.size() - numFrames);
        RelativePose average = new RelativePose();
        double actualFrames = frames.size() - fromIndex;
        for (int i = fromIndex; i < frames.size(); i++)
        {
            RelativePose pose = frames.get(i);
            average.objectYaw += pose.objectYaw;
            average.r += pose.r;
            average.theta += pose.theta;
            average.x += pose.x;
            average.y += pose.y;
        }
        average.objectYaw /= actualFrames;
        average.r /= actualFrames;
        average.theta /= actualFrames;
        average.x /= actualFrames;
        average.y /= actualFrames;
        return average;
    }

    public void setMaxAverageWindow(int numFrames)
    {
        this.maxAverageWindow = numFrames;
    }

    public int getConsecutiveTargetFrames()
    {
        return consecutiveTargetFrames;
    }

    public RelativePose getLastPose()
    {
        return relativePose;
    }

    public static class RelativePose
    {
        public double r, theta, objectYaw, x, y;
    }
}
