package team492;

import com.google.gson.Gson;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision
{
    public volatile RelativePose relativePose = null;

    public void start()
    {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("RaspiVision");
        NetworkTableEntry entry = table.getEntry("VisionData");
        Gson gson = new Gson();
        entry.addListener(event -> this.relativePose = gson.fromJson(event.value.getString(), RelativePose.class),
            EntryListenerFlags.kNew | EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate);
    }

    public static class RelativePose
    {
        public double r, theta, objectYaw, x, y;
    }
}
