package raspivision;

import com.google.gson.Gson;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.vision.VisionThread;
import org.opencv.core.CvType;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.RotatedRect;
import org.opencv.imgproc.Imgproc;
import trclib.TrcUtil;

import java.io.IOException;
import java.io.InputStream;
import java.io.PrintStream;
import java.net.Socket;
import java.util.ArrayList;
import java.util.List;

public class RaspiVision
{
    private static final int TCP_PORT = 5800;

    private Socket socket;
    private Gson gson;
    private PrintStream output;
    private InputStream input;
    private Thread communicationThread;
    private TargetData targetData;
    private final Object lock = new Object();

    public RaspiVision() throws IOException
    {
        socket = new Socket("10.4.92.2", TCP_PORT);
        gson = new Gson();
        input = socket.getInputStream();
        output = new PrintStream(socket.getOutputStream());

        UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
        VisionThread thread = new VisionThread(camera, new VisionTargetPipeline(), this::processImage);
        thread.setDaemon(false);
        thread.start();

        communicationThread = new Thread(this::communicationTask);
    }

    private void communicationTask()
    {
        while(!Thread.interrupted())
        {
            try
            {
                // Read a single byte from the rio and respond with target data.
                input.read();
                TargetData target;
                synchronized (lock)
                {
                    target = targetData;
                }
                output.println(gson.toJson(target));
                output.flush();
            }
            catch (IOException e)
            {
                e.printStackTrace();
            }
        }
    }

    private double getCorrectedAngle(RotatedRect calculatedRect){
        if(calculatedRect.size.width < calculatedRect.size.height){
            return calculatedRect.angle+180;
        }else{
            return calculatedRect.angle+90;
        }
    }

    private void processImage(VisionTargetPipeline pipeline)
    {
        List<MatOfPoint> contours = pipeline.convexHullsOutput();
        List<VisionTarget> targets = new ArrayList<>();
        for(MatOfPoint m : contours)
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
            targets.add(target);
            // TODO: group vision targets by left and right
        }
    }

    private static class VisionTarget
    {
        public boolean isLeftTarget;
        public int x, y, w, h;
    }

    private static class TargetData
    {
        private static long lastId = 0;
        public long id;
        public int x, y, w, h;
        public TargetData(int x, int y, int w, int h)
        {
            this.id = lastId++;
            this.x = x;
            this.y = y;
            this.w = w;
            this.h = h;
        }

    }
}
