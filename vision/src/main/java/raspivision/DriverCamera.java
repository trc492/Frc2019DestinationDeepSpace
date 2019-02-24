/*
 * Copyright (c) 2019 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package raspivision;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import org.opencv.core.Mat;

public class DriverCamera
{
    private static final int DEFAULT_WIDTH = 640;
    private static final int DEFAULT_HEIGHT = 480;

    private Thread captureThread;
    private UsbCamera camera;
    private CvSink sink;
    private CvSource display;
    private Mat image;
    private boolean running;

    public DriverCamera(int cameraIndex)
    {
        image = new Mat();
        camera = CameraServer.getInstance().startAutomaticCapture(cameraIndex);
        camera.setResolution(DEFAULT_WIDTH, DEFAULT_HEIGHT);
        display = CameraServer.getInstance().putVideo("RaspiVision", DEFAULT_WIDTH, DEFAULT_HEIGHT);
        sink = new CvSink("DriverCamera");
        sink.setSource(camera);
    }

    public void start()
    {
        if (running)
        {
            return;
        }
        running = true;
        captureThread = new Thread(this::captureTask);
        captureThread.start();
    }

    public void stop()
    {
        if (!running)
        {
            return;
        }
        running = false;
        captureThread.interrupt();
        try
        {
            captureThread.join();
        }
        catch (InterruptedException e)
        {
            e.printStackTrace();
        }
    }

    private void captureTask()
    {
        while (!Thread.interrupted())
        {
            long response = sink.grabFrame(image);
            if (response != 0L)
            {
                display.putFrame(image);
            }
            else
            {
                System.err.println("Camera Error: " + sink.getError());
            }
        }
        sink.close();
    }
}
