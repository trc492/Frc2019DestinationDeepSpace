/*
 * Copyright (c) 2017 Titan Robotics Club (http://www.titanrobotics.com)
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

package frclib;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import trclib.TrcDbgTrace;

/**
 * This class implements a Vision Targeting system that uses OpenCV. It uses a separate vision thread and will take
 * care of thread synchronization so making the use of this class extremely simple. This class is intended to be
 * inherited by another class that provides specific methods to process the image and to retrieve the results.
 */
public abstract class FrcVisionTarget extends FrcOpenCVDetector<Rect[]>
{
    /**
     * This method is called to process an image for detecting objects.
     *
     * @param image specifies the image to be processed.
     */
    public abstract void processImage(Mat image);

    /**
     * This method returns an array of detected object rectangles.
     *
     * @return array of detected object rectangles.
     */
    public abstract Rect[] getDetectedObjectRects();

    private static final int NUM_IMAGE_BUFFERS = 2;

    private volatile Rect[] objectRects = null;
    private volatile Mat currImage = null;
    private boolean videoOutEnabled = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param videoIn specifies the video input stream.
     * @param videoOut specifies the video output stream.
     */
    public FrcVisionTarget(final String instanceName, CvSink videoIn, CvSource videoOut)
    {
        super(instanceName, videoIn, videoOut, NUM_IMAGE_BUFFERS, null);
    }   //FrcVisionTarget

    /**
     * This method returns an array of rectangles of last detected objects.
     *
     * @return array of rectangle of last detected objects.
     */
    public Rect[] getObjectRects()
    {
        final String funcName = "getObjectRects";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        return objectRects;
    }   //getObjectRects

    /**
     * This method update the video stream with the detected targets overlay on the image as rectangles.
     *
     * @param color specifies the color of the rectangle outline overlay onto the detected targets.
     * @param thickness specifies the thickness of the rectangle outline.
     */
    public void putFrame(Scalar color, int thickness)
    {
        if (currImage != null)
        {
            super.putFrame(currImage, objectRects, color, thickness);
        }
    }   //putFrame

    /**
     * This method update the video stream with the detected targets overlay on the image as rectangles.
     */
    public void putFrame()
    {
        if (currImage != null)
        {
            super.putFrame(currImage, objectRects, new Scalar(0, 255, 0), 0);
        }
    }   //putFrame

    /**
     * This method enables/disables the video out stream.
     *
     * @param enabled specifies true to enable video out stream, false to disable.
     */
    public void setVideoOutEnabled(boolean enabled)
    {
        final String funcName = "setVideoOutEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enabled=%s", Boolean.toString(enabled));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        videoOutEnabled = enabled;
    }   //setVideoOutEnabled

    //
    // Implements the TrcVisionTask.VisionProcessor.detectObjects method.
    //

    /**
     * This method is called to detect objects in the acquired image frame.
     *
     * @param image specifies the image to be processed.
     * @return detected objects, null if none detected.
     */
    @Override
    public Rect[] detectObjects(Mat image, Rect[] buffers)
    {
        //
        // Process the image to detect the objects we are looking for and put them into detectedObjects.
        //
        processImage(image);
        objectRects = getDetectedObjectRects();

        if (videoOutEnabled)
        {
            putFrame();
        }

        return objectRects;
    }   //detectObjects

}   //class FrcVisionTarget
