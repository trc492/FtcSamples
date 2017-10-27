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

package samples;

import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

import ftclib.FtcOpMode;
import hallib.HalDashboard;
import hallib.HalVideoSource;
import trclib.TrcDbgTrace;
import trclib.TrcOpenCvDetector;

public class GripVision extends TrcOpenCvDetector<Rect[]>
{
    private static final String moduleName = "GripVision";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;
    private TrcDbgTrace tracer = FtcOpMode.getGlobalTracer();

    private static final int NUM_IMAGE_BUFFERS = 2;

    private static final double[] RED_TARGET_HUE = {0.0, 100.0};
    private static final double[] BLUE_TARGET_HUE = {40.0, 160.0};

    private GripPipeline gripRedTarget = null;
    private GripPipeline gripBlueTarget = null;
    private volatile Rect[] targetRects = null;
    private boolean videoOutEnabled = false;

    public GripVision(final String instanceName, HalVideoSource videoSource)
    {
        super(instanceName, videoSource, NUM_IMAGE_BUFFERS, null);

        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName, tracingEnabled, traceLevel, msgLevel);
        }

        gripRedTarget = new GripPipeline("RedTarget", RED_TARGET_HUE);
        gripBlueTarget = new GripPipeline("BlueTarget", BLUE_TARGET_HUE);
        targetRects = new Rect[2];
    }   //GripVision

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

    public void retrieveTargetRects(Rect[] rects)
    {
        synchronized (targetRects)
        {
            rects[0] = targetRects[0];
            targetRects[0] = null;
            rects[1] = targetRects[1];
            targetRects[1] = null;
        }
    }   //retrieveTargetRects

    /**
     * This method update the video stream with the detected targets overlay on the image as rectangles.
     *
     * @param color specifies the color of the rectangle outline overlay onto the detected targets.
     * @param thickness specifies the thickness of the rectangle outline.
     */
    private void putFrame(Mat image, Scalar color, int thickness)
    {
        drawRectangles(image, targetRects, color, thickness);
    }   //putFrame

    /**
     * This method update the video stream with the detected targets overlay on the image as rectangles.
     */
    private void putFrame(Mat image)
    {
        drawRectangles(image, targetRects, new Scalar(0, 255, 0), 0);
    }   //putFrame

    private Rect getTargetRect(GripPipeline pipeline, Mat image)
    {
        Rect targetRect = null;
        MatOfKeyPoint detectedTargets;

        pipeline.process(image);
        detectedTargets = pipeline.findBlobsOutput();
        if (detectedTargets != null)
        {
            KeyPoint[] targets = detectedTargets.toArray();
            if (targets.length > 1)
            {
                HalDashboard.getInstance().displayPrintf(15, "%s: %s", pipeline, targets[0]);
                double radius = targets[0].size/2;
                targetRect = new Rect(
                        (int)(targets[0].pt.x - radius), (int)(targets[0].pt.y - radius),
                        (int)targets[0].size, (int)targets[0].size);
            }
            detectedTargets.release();
        }

        return targetRect;
    }   //getTargetRect

    /**
     * This method is called to detect objects in the acquired image frame.
     *
     * @param image specifies the image to be processed.
     * @param buffers specifies the preallocated buffer to hold the detected targets (not used since no
     *        preallocated buffer required).
     * @return detected objects, null if none detected.
     */
    @Override
    public Rect[] detectObjects(Mat image, Rect[] buffers)
    {
        //
        // Process the image to detect the targets we are looking for and put them into targetRects.
        //
        synchronized (targetRects)
        {
            targetRects[0] = getTargetRect(gripRedTarget, image);
            targetRects[1] = getTargetRect(gripBlueTarget, image);
        }

        if (videoOutEnabled)
        {
            putFrame(image);
        }

        return targetRects;
    }   //detectObjects

}   //class GripVision
