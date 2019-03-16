package team492;

import com.google.gson.Gson;
import edu.wpi.first.networktables.ConnectionNotification;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Direction;
import trclib.TrcUtil;

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
    private Robot robot;

    public RaspiVision(Robot robot)
    {
        this.robot = robot;

        NetworkTableInstance instance = NetworkTableInstance.getDefault();
        NetworkTable table = instance.getTable("RaspiVision");
        NetworkTableEntry entry = table.getEntry("VisionData");
        NetworkTableEntry pitchEntry = table.getEntry("CameraPitch");
        instance.addConnectionListener(this::connectionListener, false);
        gson = new Gson();
        entry.addListener(this::updateTargetInfo,
            EntryListenerFlags.kNew | EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate);
        ringLight = new Relay(RobotInfo.RELAY_RINGLIGHT_POWER);
        ringLight.setDirection(Direction.kForward);

        pitchEntry.setDouble(RobotInfo.CAMERA_PITCH);
    }

    public void setRingLightEnabled(boolean enabled)
    {
        ringLight.set(enabled ? Relay.Value.kOn : Relay.Value.kOff);
    }

    private void connectionListener(ConnectionNotification notification)
    {
        if (!notification.connected)
        {
            robot.globalTracer.traceInfo("RaspiVision.connectionListener", "Client %s disconnected!",
                notification.conn.remote_ip);
        }
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
            RelativePose relativePose = gson.fromJson(info, RelativePose.class);
            relativePose.time = TrcUtil.getCurrentTime();
            this.relativePose = relativePose;
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

    private boolean isFresh(RelativePose pose)
    {
        return pose != null && TrcUtil.getCurrentTime() - pose.time >= RobotInfo.CAMERA_DATA_TIMEOUT;
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
            int numFreshFrames = 0;
            for (RelativePose pose : poses)
            {
                // Only use data if it's fresh
                if (isFresh(pose))
                {
                    average.objectYaw += pose.objectYaw;
                    average.r += pose.r;
                    average.theta += pose.theta;
                    average.x += pose.x;
                    average.y += pose.y;
                    numFreshFrames++;
                }
            }
            // If no fresh data, clear all and return null
            if (numFreshFrames == 0)
            {
                frames.clear();
                return null;
            }
            average.objectYaw /= (double) numFreshFrames;
            average.r /= (double) numFreshFrames;
            average.theta /= (double) numFreshFrames;
            average.x /= (double) numFreshFrames;
            average.y /= (double) numFreshFrames;
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
        RelativePose pose = relativePose;
        return isFresh(pose) ? pose : null;
    }

    public static class RelativePose
    {
        public double r, theta, objectYaw, x, y;
        public double time;
    }
}
