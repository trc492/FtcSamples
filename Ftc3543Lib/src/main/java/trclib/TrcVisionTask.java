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
 */
/**
 * This class implements a platform independent vision task. When enabled, it grabs a frame from the video source,
 * calls the provided object detector to process the frame and overlays rectangles on the detected objects in the
 * image. This class is to be extended by a platform dependent vision processor who will provide the video input
 * and output. 
 *
 * @param <I> specifies the type of the image frame buffer.
 * @param <O>specifies the type of the detected objects.
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
     *
     * @param <I> specifies the type of the image frame buffer.
     * @param <O> specifies the type of the detected objects.
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

    /**
     * This class keeps track of the state of the vision task. It also provides thread synchronization control to
     * make sure the integrity of the task state.
     */
    private class TaskState
    {
        private boolean taskTerminated;
        private boolean taskEnabled;
        private boolean oneShotEnabled;
        private O targets;

        /**
         * Constructor: Create an instance of the object.
         */
        public TaskState()
        {
            taskTerminated = false;
            taskEnabled = false;
            oneShotEnabled = false;
            targets = null;
        }   //TaskState

        /**
         * This method checks if the task has been terminated.
         *
         * @return true if task has been terminated, false otherwise.
         */
        public synchronized boolean isTaskTerminated()
        {
            return taskTerminated;
        }   //isTaskTerminated

        /**
         * This method is called to terminate the vision task.
         */
        public synchronized void terminateTask()
        {
            taskTerminated = true;
        }   //terminateTask

        /**
         * This method checks if the vision task is enabled.
         *
         * @return true if task is enabled, false otherwise.
         */
        public synchronized boolean isTaskEnabled()
        {
            return !taskTerminated && (taskEnabled || oneShotEnabled);
        }   //isTaskEnabled

        /**
         * This method enables/disables the vision task. If this is called to disable the task, the task will be
         * set to a paused state. The operation will be resumed when this is called to enable it again.
         *
         * @param enabled specifies true to enable vision task, false to disable.
         */
        public synchronized void setTaskEnabled(boolean enabled)
        {
            if (!taskTerminated)
            {
                taskEnabled = enabled;
            }
        }   //setTaskEnabled

        /**
         * This method returns the last detected targets. If there is no new detected targets since the last call,
         * it will return null.
         *
         * @return newly detected targets if any, null if none.
         */
        public synchronized O getTargets()
        {
            O newTargets = null;

            if (!taskTerminated)
            {
                //
                // If task was not enabled, it must be a one-shot deal. Since we don't already have targets detected,
                // we must unblock the task so it can process the next image frame.
                //
                if (!taskEnabled && targets == null)
                {
                    oneShotEnabled = true;
                }
                newTargets = targets;
                targets = null;
            }

            return newTargets;
        }   //getTargets

        /**
         * This method is called to set new targets after new targets have been detected.
         *
         * @param targets specifies newly detected targets. 
         */
        public synchronized void setTargets(O targets)
        {
            if (!taskTerminated)
            {
                this.targets = targets;
                oneShotEnabled = false;
            }
        }   //setTargets

    }   //class TaskState

    private VisionProcessor<I, O> visionProcessor;
    private I image;
    private O[] detectedObjectsBuffers;
    private int bufferIndex = 0;
    private long totalTime = 0;
    private long totalFrames = 0;
    private long processingInterval = 50;   // in msec
    private TaskState taskState = new TaskState();
    private Thread visionThread = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param visionProcessor specifies the vision processor object.
     * @param imageBuffer specifies the buffer to hold video image.
     * @param detectedObjectsBuffers specifies an array of buffers to hold the detected objects.
     */
    public TrcVisionTask(VisionProcessor<I, O> visionProcessor, I imageBuffer, O[] detectedObjectsBuffers)
    {
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName, tracingEnabled, traceLevel, msgLevel);
        }

        this.visionProcessor = visionProcessor;
        this.image = imageBuffer;
        this.detectedObjectsBuffers = detectedObjectsBuffers;

        visionThread = new Thread(this, "VisionTask");
//        visionThread.setDaemon(true);
        visionThread.start();
    }   //TrcVisionTask

    /**
     * This method is called to terminate the vision task. Once this is called, no other method in this class
     * should be called except for isTaskTerminated().
     */
    public void terminateTask()
    {
        taskState.terminateTask();
    }   //terminateTask

    /**
     * This method checks if the vision task has been terminated.
     *
     * @return true if vision task is terminated, false otherwise.
     */
    public boolean isTaskTerminated()
    {
        return taskState.isTaskTerminated();
    }   //isTaskTerminated

    /**
     * This method enables/disables the vision processing task. As long as the task is enabled, it will continue to
     * process image frames.
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

        taskState.setTaskEnabled(enabled);

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
        boolean enabled = taskState.isTaskEnabled();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Boolean.toString(enabled));
        }

        return enabled;
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
        final String funcName = "getTargets";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        return taskState.getTargets();
    }   //getTargets

    //
    // Implements Runnable interface.
    //

    /**
     * This method runs the vision processing task.
     */
    @Override
    public void run()
    {
        while (!taskState.isTaskTerminated())
        {
            long startTime = TrcUtil.getCurrentTimeMillis();

            if (taskState.isTaskEnabled())
            {
                processImage();
            }

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
            visionProcessor.detectObjects(image, detectedObjectsBuffers[bufferIndex]);
            elapsedTime = TrcUtil.getCurrentTimeMillis() - startTime;
            totalTime += elapsedTime;
            totalFrames++;
            if (visionPerfEnabled && dbgTrace != null)
            {
                dbgTrace.traceInfo(funcName, "Average processing time = %.3f msec", (double)totalTime/totalFrames);
            }

            visionProcessor.putFrame(image, detectedObjectsBuffers[bufferIndex]);

            taskState.setTargets(detectedObjectsBuffers[bufferIndex]);
            //
            // Switch to the next buffer so that we won't clobber the info while the client is accessing it.
            //
            bufferIndex = (bufferIndex + 1)%detectedObjectsBuffers.length;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //processImage

}   //class TrcVisionTask
