package team492;

import com.google.gson.Gson;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Direction;

import java.util.LinkedList;
import java.util.List;

public class RaspiVision
{
    private volatile RelativePose relativePose = null;
    private Gson gson;
    private int maxAverageWindow = 10; // the last 10 frames
    private List<RelativePose> frames = new LinkedList<>();
    private final Object framesLock = new Object();
    private Relay ringLight;

    public RaspiVision()
    {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("RaspiVision");
        NetworkTableEntry entry = table.getEntry("VisionData");
        gson = new Gson();
        entry.addListener(this::updateTargetInfo,
            EntryListenerFlags.kNew | EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate);
        ringLight = new Relay(RobotInfo.RELAY_RINGLIGHT_POWER);
        ringLight.setDirection(Direction.kForward);
    }

    public void setRingLightEnabled(boolean enabled)
    {
        ringLight.set(enabled ? Relay.Value.kOn : Relay.Value.kOff);
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
        }
        else
        {
            // Deserialize the latest calculated pose
            this.relativePose = gson.fromJson(info, RelativePose.class);
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
     * Calculates the average pose of the last numFrames frames.
     *
     * @param numFrames How many frames to average.
     * @return The average pose.
     */
    public RelativePose getAveragePose(int numFrames)
    {
        return getAveragePose(numFrames, false);
    }

    /**
     * Calculates the average pose of the last numFrames frames, optionally requiring numFrames frames.
     *
     * @param numFrames  How many frames to average.
     * @param requireAll If true, require at least numFrames frames to average.
     * @return Average of last numFrames frames, or null if not enough frames and requireAll is true.
     */
    public RelativePose getAveragePose(int numFrames, boolean requireAll)
    {
        RelativePose average = new RelativePose();
        synchronized (framesLock)
        {
            if ((requireAll && frames.size() < numFrames) || frames.isEmpty())
            {
                return null;
            }
            int fromIndex = Math.max(0, frames.size() - numFrames);
            List<RelativePose> poses = frames.subList(fromIndex, frames.size());
            for (RelativePose pose : poses)
            {
                average.objectYaw += pose.objectYaw;
                average.r += pose.r;
                average.theta += pose.theta;
                average.x += pose.x;
                average.y += pose.y;
            }
            average.objectYaw /= poses.size();
            average.r /= poses.size();
            average.theta /= poses.size();
            average.x /= poses.size();
            average.y /= poses.size();
        }
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
