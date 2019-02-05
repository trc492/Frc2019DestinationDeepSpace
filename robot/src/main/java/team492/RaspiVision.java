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
import java.util.concurrent.atomic.AtomicInteger;

public class RaspiVision
{
    private volatile RelativePose relativePose = null;
    private Gson gson;
    private AtomicInteger consecutiveTargetFrames = new AtomicInteger(0);
    private int maxAverageWindow = 10; // the last 10 frames
    private List<RelativePose> frames = new LinkedList<>();
    private final Object framesLock = new Object();

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
            // We have not found a pose, so set to null
            this.relativePose = null;
            synchronized (framesLock)
            {
                // Clear the frames list if it is not empty
                if (!frames.isEmpty())
                {
                    frames.clear();
                }
            }
            // Reset the number of consecutive frames
            consecutiveTargetFrames.set(0);
        }
        else
        {
            // Deserialize the latest calculated pose
            this.relativePose = gson.fromJson(info, RelativePose.class);
            // Increment the number of consecutive frames
            this.consecutiveTargetFrames.incrementAndGet();
            synchronized (framesLock)
            {
                // Add the latest pose
                frames.add(relativePose);
                // Trim the list so only the last few are kept
                while (frames.size() > maxAverageWindow)
                {
                    frames.remove(0);
                }
            }
        }
    }

    /**
     * Get the average pose of the last n frames where n=maxAverageWindow.
     *
     * @return The average pose. (all attributes averaged)
     */
    public RelativePose getAveragePose()
    {
        return getAveragePose(maxAverageWindow);
    }

    /**
     * Calcluateds the avergae pose of the lsat numFrames frames.
     *
     * @param numFrames How many frames to average.
     * @return The average pose.
     */
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

    /**
     * Set the max number of frames to keep. You will not be able to average more frames than this.
     *
     * @param numFrames How many frames to keep for averaging at maximum.
     */
    public void setMaxAverageWindow(int numFrames)
    {
        if (numFrames < 0)
        {
            throw new IllegalArgumentException("numFrames must be >= 0!");
        }
        this.maxAverageWindow = numFrames;
    }

    /**
     * Gets how many consecutive frames the target has been detected.
     *
     * @return The number of consecutive frames the target has been detected.
     */
    public int getConsecutiveTargetFrames()
    {
        return consecutiveTargetFrames.get();
    }

    /**
     * Gets the last calculated pose.
     *
     * @return The pose calculated by the vision system. If no object detected, returns null.
     */
    public RelativePose getLastPose()
    {
        return relativePose;
    }

    public static class RelativePose
    {
        public double r, theta, objectYaw, x, y;
    }
}
