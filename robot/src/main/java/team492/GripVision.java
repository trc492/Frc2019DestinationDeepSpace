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

package team492;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import frclib.FrcVisionTarget;
import trclib.TrcDbgTrace;

public class GripVision extends FrcVisionTarget
{
    private static final String moduleName = "GripVision";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private TrcDbgTrace tracer = TrcDbgTrace.getGlobalTracer();
    private GripPipeline pipeline;

    public GripVision(final String instanceName, CvSink videoIn, CvSource videoOut)
    {
        super(instanceName, videoIn, videoOut);

        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName, tracingEnabled, traceLevel, msgLevel);
        }

        pipeline = new GripPipeline();
    }   //GripVision

    public Rect getTargetRect()
    {
        Rect targetRect = null;
        Rect[] objectRects = getObjectRects();

        if (debugEnabled)
        {
            dbgTrace.traceInfo(moduleName, "%s object(s) found",
                objectRects != null? "" + objectRects.length: "null");
        }

        if (objectRects != null && objectRects.length >= 2)
        {
            if (debugEnabled)
            {
                for (int i = 0; i < objectRects.length; i++)
                {
                    tracer.traceInfo(moduleName, "%02d: x=%d, y=%d, width=%d, height=%d",
                        i, objectRects[i].x, objectRects[i].y, objectRects[i].width, objectRects[i].height);
                }
            }
            //
            // Sort the detected objects by area from largest to smallest.
            //
            Arrays.sort(
                objectRects,
                new Comparator<Rect>()
                {
                    public int compare(Rect rect1, Rect rect2)
                    {
                        return rect2.width*rect2.height - rect1.width*rect1.height;
                    }
                });

            if (debugEnabled)
            {
                for (int i = 0; i < objectRects.length; i++)
                {
                    tracer.traceInfo(moduleName + ".sorted", "%02d: x=%d, y=%d, width=%d, height=%d",
                        i, objectRects[i].x, objectRects[i].y, objectRects[i].width, objectRects[i].height);
                }
            }

            int targetRectX1 = Math.min(objectRects[0].x, objectRects[1].x);
            int targetRectY1 = Math.min(objectRects[0].y, objectRects[1].y);
            int targetRectX2 = Math.max(objectRects[0].x + objectRects[0].width,
                                        objectRects[1].x + objectRects[1].width);
            int targetRectY2 = Math.max(objectRects[0].y + objectRects[0].height,
                                        objectRects[1].y + objectRects[1].height);
            int targetRectWidth = targetRectX2 - targetRectX1;
            int targetRectHeight = targetRectY2 - targetRectY1;

            targetRect = new Rect(targetRectX1, targetRectY1, targetRectWidth, targetRectHeight);

            if (debugEnabled)
            {
                tracer.traceInfo(moduleName, "TargetRect: x=%d, y=%d, w=%d, h=%d",
                    targetRect.x, targetRect.y, targetRect.width, targetRect.height);
            }
        }

        return targetRect;
    }   //getTargetRect

    //
    // Implements FrcVisionTarget abstract methods.
    //

    @Override
    public void processImage(Mat image)
    {
        pipeline.process(image);
    }   //processImage

    @Override
    public Rect[] getDetectedObjectRects()
    {
        Rect[] objectRects = null;
        ArrayList<MatOfPoint> detectedObjects = pipeline.findContoursOutput();
        //
        // If we detected any objects, convert them into an array of rectangles.
        //
        if (detectedObjects != null && !detectedObjects.isEmpty())
        {
            objectRects = new Rect[detectedObjects.size()];
            for (int i = 0; i < objectRects.length; i++)
            {
                MatOfPoint object = detectedObjects.get(i);
                objectRects[i] = Imgproc.boundingRect(object);
                object.release();
            }
        }

        return objectRects;
    }   //getDetectedObjects

}   //class GripVision
