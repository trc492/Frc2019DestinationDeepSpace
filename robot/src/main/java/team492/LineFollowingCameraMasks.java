package team492;

import org.opencv.core.Rect;
import org.opencv.core.Point;
import trclib.TrcPixyCam2.Vector;

public class LineFollowingCameraMasks
{
    public Rect[] rects;

    public LineFollowingCameraMasks(Rect[] rects)
    {
        this.rects = rects;
    }

    public boolean isOutsideMask(Vector feature) 
    {
        boolean outside = true;
        for (int i = 0; i < rects.length; i++)
         {
            outside = outside && !rects[i].contains(new Point(feature.x0, feature.y0));
            outside = outside && !rects[i].contains(new Point(feature.x1, feature.y1));
            if (!outside) break;
        }
        return outside;
    }
}