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

package trclib;

/**
 * This class implements a platform independent vision task. When enabled, it grabs a frame from the video source,
 * calls the provided object detector to process the frame and overlays rectangles on the detected objects in the
 * image. This class is to be extended by a platform dependent vision processor who will provide the video input
 * and output. 
 */
public class TrcVisionTask<I, O> implements Runnable
{
    private static final String moduleName = "TrcVisionTask";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private static final boolean visionPerfEnabled = true;

    /**
     * This interface provides methods to grab image from the video input, render image to video output and detect
     * objects from the acquired image.
     */
    public interface VisionProcessor<I, O>
    {
        /**
         * This method is called to grab an image frame from the video input.
         *
         * @param image specifies the frame buffer to hold the captured image.
         * @return true if frame is successfully captured, false otherwise.
         */
        boolean grabFrame(I image);

        /**
         * This method is called to render an image to the video output and overlay detected objects on top of it.
         * 
         * @param image specifies the frame to be rendered to the video output.
         * @param detectedObjects specifies the detected objects.
         */
        void putFrame(I image, O detectedObjects);

        /**
         * This method is called to detect objects in the acquired image frame.
         *
         * @param image specifies the image to be processed.
         * @param detectedObjects specifies the object rectangle array to hold the detected objects.
         * @return true if detected objects, false otherwise.
         */
        boolean detectObjects(I image, O detectedObjects);

    }   //interface VisionProcessor

    private VisionProcessor<I, O> visionProcessor;
    private I image;
    private O detectedObjects;
    private O targets;
    private long totalTime = 0;
    private long totalFrames = 0;

    private Object monitor;
    private Thread visionThread = null;

    private long processingInterval = 50;   // in msec
    private boolean taskEnabled = false;
    private boolean oneShotEnabled = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param visionProcessor specifies the vision processor object.
     * @param imageBuffer specifies the buffer to hold video image.
     * @param detectedObjectsBuffer specifies the buffer to hold the detected objects.
     */
    public TrcVisionTask(VisionProcessor<I, O> visionProcessor, I imageBuffer, O detectedObjectsBuffer)
    {
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName, tracingEnabled, traceLevel, msgLevel);
        }

        this.visionProcessor = visionProcessor;
        this.image = imageBuffer;
        this.detectedObjects = detectedObjectsBuffer;
        targets = null;
        monitor = new Object();
        visionThread = new Thread(this, "VisionTask");
        visionThread.setDaemon(true);
        visionThread.start();
    }   //TrcVisionTask

    /**
     * This method enables/disables the vision processing task.
     *
     * @param enabled specifies true to enable vision task, false to disable.
     */
    public void setTaskEnabled(boolean enabled)
    {
        final String funcName = "setTaskEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enabled=%s", Boolean.toString(enabled));
        }

        if (!taskEnabled && enabled)
        {
            //
            // Enable task.
            //
            synchronized(monitor)
            {
                taskEnabled = true;
                monitor.notify();
            }
        }
        else if (taskEnabled && !enabled)
        {
            //
            // Disable task.
            //
            taskEnabled = false;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setTaskEnabled

    /**
     * This method returns the state of the vision task.
     *
     * @return true if the vision task is enabled, false otherwise.
     */
    public boolean isTaskEnabled()
    {
        final String funcName = "isTaskEnabled";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Boolean.toString(taskEnabled));
        }

        return taskEnabled;
    }   //isTaskEnabled

    /**
     * This method sets the vision task processing interval.
     *
     * @param interval specifies the processing interval in msec.
     */
    public void setProcessingInterval(long interval)
    {
        final String funcName = "setProcessingInterval";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "interval=%dms", interval);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        processingInterval = interval;
    }   //setProcessInterval

    /**
     * This method returns the vision task processing interval.
     *
     * @return vision task processing interval in msec.
     */
    public long getProcessingInterval()
    {
        final String funcName = "getProcessingInterval";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%d", processingInterval);
        }

        return processingInterval;
    }   //getProcessingInterval

    /**
     * This method returns the detected objects. If nothing found, it returns null.
     *
     * @return detected objects, null if nothing found.
     */
    public O getTargets()
    {
        final String funcName = "getObjects";
        O newTargets = null;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        synchronized(monitor)
        {
            if (!taskEnabled && targets == null)
            {
                oneShotEnabled = true;
                monitor.notify();
            }
            newTargets = targets;
            targets = null;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        return newTargets;
    }   //getTargets

    //
    // Implements Runnable interface.
    //

    /**
     * This method runs the vision processing task.
     */
    public void run()
    {
        while (true)
        {
            synchronized(monitor)
            {
                //
                // Wait until we are enabled.
                //
                while (!taskEnabled && !oneShotEnabled)
                {
                    try
                    {
                        monitor.wait();
                    }
                    catch (InterruptedException e)
                    {
                    }
                }
            }

            long startTime = TrcUtil.getCurrentTimeMillis();
            processImage();
            long sleepTime = processingInterval - (TrcUtil.getCurrentTimeMillis() - startTime);

            TrcUtil.sleep(sleepTime);
        }
    }   //run

    /**
     * This method grabs a new video frame and processes it.
     */
    private void processImage()
    {
        final String funcName = "processImage";
        double startTime;
        double elapsedTime;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK);
        }

        if (visionProcessor.grabFrame(image))
        {
            //
            // Capture an image and subject it for object detection. The object detector produces an array of
            // rectangles representing objects detected.
            //
            startTime = TrcUtil.getCurrentTimeMillis();
            visionProcessor.detectObjects(image, detectedObjects);
            elapsedTime = TrcUtil.getCurrentTimeMillis() - startTime;
            totalTime += elapsedTime;
            totalFrames++;
            if (visionPerfEnabled && dbgTrace != null)
            {
                dbgTrace.traceInfo(funcName, "Average processing time = %.3f msec", (double)totalTime/totalFrames);
            }

            visionProcessor.putFrame(image, detectedObjects);

            synchronized(monitor)
            {
                oneShotEnabled = false;
                targets = detectedObjects;
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //processImage

}   //class TrcVisionTask
