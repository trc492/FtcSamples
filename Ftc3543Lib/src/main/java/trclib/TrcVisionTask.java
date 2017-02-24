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
 *
 * @param <I> specifies the type of the image frame buffer.
 * @param <O>specifies the type of the detected objects.
 */
public class TrcVisionTask<I, O> extends TrcThread<O> implements TrcThread.PeriodicTask
{
    private static final String moduleName = "TrcVisionTask";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private static final boolean visionPerfEnabled = true;
    private static final long DEF_PROCESSING_INTERVAL = 50;     //in msec

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

    private VisionProcessor<I, O> visionProcessor;
    private I image;
    private O[] detectedObjectsBuffers;
    private int bufferIndex = 0;
    private long totalTime = 0;
    private long totalFrames = 0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param visionProcessor specifies the vision processor object.
     * @param imageBuffer specifies the buffer to hold video image.
     * @param detectedObjectsBuffers specifies an array of buffers to hold the detected objects.
     */
    public TrcVisionTask(VisionProcessor<I, O> visionProcessor, I imageBuffer, O[] detectedObjectsBuffers)
    {
        super("VisionTask", null);

        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName, tracingEnabled, traceLevel, msgLevel);
        }

        this.visionProcessor = visionProcessor;
        this.image = imageBuffer;
        this.detectedObjectsBuffers = detectedObjectsBuffers;
        setPeriodicTask(this);
        setProcessingInterval(DEF_PROCESSING_INTERVAL);
    }   //TrcVisionTask

    //
    // Implements TrcThread.PeriodicTask interface.
    //

    /**
     * This method runs the vision processing task.
     */
    @Override
    public void runPeriodic()
    {
        final String funcName = "runPeriodic";
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
            setData(detectedObjectsBuffers[bufferIndex]);
            //
            // Switch to the next buffer so that we won't clobber the info while the client is accessing it.
            //
            bufferIndex = (bufferIndex + 1)%detectedObjectsBuffers.length;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //runPeriodic

}   //class TrcVisionTask
