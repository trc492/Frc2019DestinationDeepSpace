package raspivision;

import com.google.gson.Gson;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.vision.VisionThread;

import java.io.BufferedReader;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.PrintStream;
import java.net.Socket;

public class Main
{
    private static final int TCP_PORT = 5800;

    private static Socket socket;
    private static Gson gson;
    private static PrintStream output;
    private static InputStream input;
    private static Thread communicationThread;

    public static void main(String[] args) throws IOException
    {
        socket = new Socket("10.4.92.2", TCP_PORT);
        gson = new Gson();
        input = socket.getInputStream();
        output = new PrintStream(socket.getOutputStream());

        UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
        VisionThread thread = new VisionThread(camera, new VisionTargetPipeline(), Main::processImage);
        thread.setDaemon(false);
        thread.start();
    }

    private static void communicationTask()
    {
        while(!Thread.interrupted())
        {
            try
            {
                input.read();

            }
            catch (IOException e)
            {
                e.printStackTrace();
            }
        }
    }

    private static void processImage(VisionPipeline pipeline)
    {

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

    private void sendData(int x, int y, int w, int h)
    {
        TargetData data = new TargetData(x, y, w, h);
    }
}
