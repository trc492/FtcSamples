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

import java.util.Arrays;
import java.util.concurrent.ConcurrentLinkedQueue;

import trclib.TrcDbgTrace;
import trclib.TrcEvent;
import trclib.TrcUtil;

/**
 * This class implements a platform independent serial bus device. This class is intended to be inherited by a
 * platform dependent serial bus device such as I2C device or Serial Port device that provides synchronous methods
 * to access the device. It creates a request queue to allow both synchronous and asynchronous requests to be queued
 * for processing. The request queue is processed by a separate thread for asynchronous access.
 *
 * @param <T> specifies the request tag type.
 */
public abstract class TrcSerialBusDevice implements Runnable
{
    protected static final String moduleName = "TrcSerialBusDevice";
    protected static final boolean debugEnabled = false;
    protected static final boolean tracingEnabled = false;
    protected static final boolean useGlobalTracer = false;
    protected static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    protected static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    protected TrcDbgTrace dbgTrace = null;

    /**
     * This method is called to read data from the device with the specified length.
     *
     * @param address specifies the data address if any, can be -1 if no address is required.
     * @param length specifies the number of bytes to read.
     * @return a byte array containing the data read.
     */
    public abstract byte[] readData(int address, int length);

    /**
     * This method is called to write data to the device with the specified data buffer and length.
     *
     * @param address specifies the data address if any, can be -1 if no address is required.
     * @param buffer specifies the buffer containing the data to be written to the device.
     * @param length specifies the number of bytes to write.
     * @return number of bytes written.
     */
    public abstract int writeData(int address, byte[] buffer, int length);

    /**
     * This interface provides callback notification on asynchronous read/write completion.
     */
    public interface CompletionHandler
    {
        /**
         * This method is called when the read operation has been completed.
         *
         * @param requestTag specifies the tag to identify the request. Can be null if none was provided.
         * @param address specifies the data address read from if any, can be -1 if none specified.
         * @param data specifies the byte array containing data read.
         * @param error specifies true if the request failed, false otherwise. When true, data is invalid.
         * @return true if retry the read request, false otherwise.
         */
        boolean readCompletion(Object requestTag, int address, byte[] data, boolean error);

        /**
         * This method is called when the write operation has been completed.
         *
         * @param requestTag specifies the tag to identify the request. Can be null if none was provided.
         * @param address specifies the data address wrote to if any, can be -1 if none specified.
         * @param length specifies the number of bytes written.
         * @param error specifies true if the request failed to write the specified length, false otherwise.
         *              When true, length is invalid.
         */
        void writeCompletion(Object requestTag, int address, int length, boolean error);

    }   //interface CompletionHandler

    /**
     * This class implements a request. Typically, a request will be put in the request queue so that each request
     * will be processed in the order they came in.
     */
    private class Request
    {
        public Object requestTag;
        public boolean readRequest;
        public int address;
        public byte[] buffer;
        public int length;
        public boolean repeat;
        public TrcEvent event;
        public CompletionHandler handler;
        public boolean error;

        /**
         * Constructor: Create an instance of the object.
         *
         * @param requestTag specifies the tag to identify the request. Can be null if none was provided.
         * @param readRequest specifies true for a read request, false for a write request.
         * @param address specifies the data address if any, can be -1 if no address is required.
         * @param buffer specifies the buffer that contains data for a write request, ignored for read request.
         * @param length specifies the number of bytes to read or write.
         * @param repeat specifies true to re-queue the request when completed.
         * @param event specifies the event to signal when the request is completed, can be null if none specified.
         * @param handler specifies the completion handler to call when the request is completed, can be null if none
         *                specified.
         */
        public Request(
            Object requestTag, boolean readRequest, int address, byte[] buffer, int length, boolean repeat,
            TrcEvent event, CompletionHandler handler)
        {
            this.requestTag = requestTag;
            this.readRequest = readRequest;
            this.address = address;
            this.buffer = buffer;
            this.length = length;
            this.repeat = repeat;
            this.event = event;
            this.handler = handler;
            this.error = false;
        }   //Request

    }   //class Request

    private TrcDbgTrace perfTracer = null;
    private double totalTime = 0.0;
    private int totalRequests = 0;

    private final String instanceName;
    private ConcurrentLinkedQueue<Request> requestQueue;
    private Thread deviceTask;
    private volatile long processingInterval = 0;    // in msec
    private volatile boolean taskEnabled = false;
    private volatile boolean taskTerminatedAbnormally = false;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public TrcSerialBusDevice(final String instanceName)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
        requestQueue = new ConcurrentLinkedQueue<>();
        deviceTask = new Thread(this, instanceName);
        deviceTask.setUncaughtExceptionHandler((thread, throwable) ->
        {
            if (!(throwable.getClass().equals(InterruptedException.class)))
            {
                taskTerminatedAbnormally = true;
                if (debugEnabled)
                {
                    dbgTrace.traceWarn(moduleName, "Thread %s for %s had uncaught exception: %s",
                        thread, instanceName, throwable);
                }
            }
        });
        deviceTask.start();
    }   //TrcSerialBusDevice

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method checks if the device task has been terminated.
     *
     * @return true if task has been terminated, false otherwise.
     */
    public synchronized boolean isTaskTerminated()
    {
        return !deviceTask.isAlive();
    }   //isTaskTerminated

    /**
     * This method checks if the device task has been terminated.
     *
     * @return true if task has been terminated, false otherwise.
     */
    public synchronized boolean isTaskTerminatedAbnormally()
    {
        return taskTerminatedAbnormally;
    }   //isTaskTerminatedAbnormally

    /**
     * This method is called to terminate the device task.
     */
    public synchronized void terminateTask()
    {
        if (deviceTask.isAlive())
        {
            deviceTask.interrupt();
        }
    }   //terminateTask

    /**
     * This method checks if the device task is enabled.
     *
     * @return true if task is enabled, false otherwise.
     */
    public synchronized boolean isTaskEnabled()
    {
        return deviceTask.isAlive() && taskEnabled;
    }   //isTaskEnabled

    /**
     * This method enables/disables the device task. If enabling task, the task will be started. If disabling task,
     * the task will be terminated.
     *
     * @param enabled specifies true to enable device task, false to disable.
     */
    public synchronized void setTaskEnabled(boolean enabled)
    {
        if (deviceTask.isAlive())
        {
            if (!taskEnabled && enabled)
            {
                totalTime = 0.0;
                totalRequests = 0;
            }
            taskEnabled = enabled;
        }
    }   //setTaskEnabled

    /**
     * This method sets the device task processing interval.
     *
     * @param interval specifies the processing interval in msec. If 0, process as fast as the CPU can run.
     */
    public synchronized void setProcessingInterval(long interval)
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
     * This method returns the device task processing interval.
     *
     * @return device task processing interval in msec.
     */
    public synchronized long getProcessingInterval()
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
     * This method enables/disables performance report.
     *
     * @param enabled specifies true to enable performance report, false to disable.
     */
    public void setPerformanceTracer(TrcDbgTrace tracer)
    {
        perfTracer = tracer;
    }   //setPerformanceTracer

    /**
     * This method is doing a synchronous read from the device with the specified length to read.
     *
     * @param address specifies the data address if any, can be -1 if no address is required.
     * @param length specifies the number of bytes to read.
     * @return data read as an array of bytes.
     */
    public byte[] syncRead(int address, int length)
    {
        final String funcName = "syncRead";
        byte[] data = null;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "addr=%d,len=%d", address, length);
        }

        if (!deviceTask.isAlive())
        {
            throw new RuntimeException("Must call setTaskEnabled first.");
        }

        TrcEvent event = new TrcEvent(instanceName + "." + funcName + "." + length);
        Request request = new Request(null, true, address, null, length, false, event, null);

        requestQueue.add(request);

        while (!event.isSignaled())
        {
            Thread.yield();
        }

        data = request.buffer;
        request.buffer = null;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s",
                data == null? "null": Arrays.toString(data));
        }

        return data;
    }   //syncRead

    /**
     * This method is doing a synchronous read from the device with the specified length to read.
     *
     * @param length specifies the number of bytes to read.
     * @return data read as an array of bytes.
     */
    public byte[] syncRead(int length)
    {
        return syncRead(-1, length);
    }   //syncRead

    /**
     * This method is doing a synchronous write to the device with the specified data and length.
     *
     * @param address specifies the data address if any, can be -1 if no address is required.
     * @param data specifies the data to write to the device.
     * @param length specifies the number of bytes to write.
     * @return number of bytes written.
     */
    public int syncWrite(int address, byte[] data, int length)
    {
        final String funcName = "syncWrite";
        int bytesWritten;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "addr=%d,data=%s,length=%d",
                address, Arrays.toString(data), length);
        }

        if (!deviceTask.isAlive())
        {
            throw new RuntimeException("Must call setTaskEnabled first.");
        }

        TrcEvent event = new TrcEvent(instanceName + "." + funcName + "." + length);
        Request request = new Request(null, false, address, data, length, false, event, null);

        requestQueue.add(request);

        while (!event.isSignaled())
        {
            Thread.yield();
        }
        bytesWritten = request.length;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%d", bytesWritten);
        }

        return bytesWritten;
    }   //syncWrite

    /**
     * This method is doing a synchronous write to the device with the specified data and length.
     *
     * @param data specifies the data to write to the device.
     * @param length specifies the number of bytes to write.
     * @return number of bytes written.
     */
    public int syncWrite(byte[] data, int length)
    {
        return syncWrite(-1, data, length);
    }   //syncWrite

    /**
     * This method is doing an asynchronous read from the device with the specified length to read.
     *
     * @param requestTag specifies the tag to identify the request. Can be null if none was provided.
     * @param address specifies the data address if any, can be -1 if no address is required.
     * @param length specifies the number of bytes to read.
     * @param repeat specifies true to re-queue the request when completed.
     * @param event specifies the event to signal when the request is completed, can be null if none specified.
     * @param handler specifies the completion handler to call when the request is completed, can be null if none
     *                specified.
     */
    public void asyncRead(
        Object requestTag, int address, int length, boolean repeat, TrcEvent event, CompletionHandler handler)
    {
        final String funcName = "asyncRead";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "tag=%s,addr=%d,len=%d,repeat=%s,event=%s",
                requestTag != null? requestTag: "null", address, length, Boolean.toString(repeat),
                event == null? "null": event.toString());
        }

        requestQueue.add(new Request(requestTag, true, address, null, length, repeat, event, handler));

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //asyncRead

    /**
     * This method is doing an asynchronous read from the device with the specified length to read.
     *
     * @param requestTag specifies the tag to identify the request. Can be null if none was provided.
     * @param address specifies the data address if any, can be -1 if no address is required.
     * @param length specifies the number of bytes to read.
     * @param event specifies the event to signal when the request is completed, can be null if none specified.
     * @param handler specifies the completion handler to call when the request is completed, can be null if none
     *                specified.
     */
    public void asyncRead(Object requestTag, int address, int length, TrcEvent event, CompletionHandler handler)
    {
        asyncRead(requestTag, address, length, false, event, handler);
    }

    /**
     * This method is doing an asynchronous read from the device with the specified length to read.
     *
     * @param requestTag specifies the tag to identify the request. Can be null if none was provided.
     * @param length specifies the number of bytes to read.
     * @param repeat specifies true to re-queue the request when completed.
     * @param event specifies the event to signal when the request is completed, can be null if none specified.
     * @param handler specifies the completion handler to call when the request is completed, can be null if none
     *                specified.
     */
    public void asyncRead(Object requestTag, int length, boolean repeat, TrcEvent event, CompletionHandler handler)
    {
        asyncRead(requestTag, -1, length, repeat, event, handler);
    }   //asyncRead

    /**
     * This method is doing an asynchronous read from the device with the specified length to read.
     *
     * @param requestTag specifies the tag to identify the request. Can be null if none was provided.
     * @param length specifies the number of bytes to read.
     * @param event specifies the event to signal when the request is completed, can be null if none specified.
     * @param handler specifies the completion handler to call when the request is completed, can be null if none
     *                specified.
     */
    public void asyncRead(Object requestTag, int length, TrcEvent event, CompletionHandler handler)
    {
        asyncRead(requestTag, -1, length, false, event, handler);
    }   //asyncRead

    /**
     * This method is doing an asynchronous write to the device with the specified data and length
     *
     * @param requestTag specifies the tag to identify the request. Can be null if none was provided.
     * @param address specifies the data address if any, can be -1 if no address is required.
     * @param data specifies the buffer containing the data to write to the device.
     * @param length specifies the number of bytes to write.
     * @param event specifies the event to signal when the request is completed, can be null if none specified.
     * @param handler specifies the completion handler to call when the request is completed, can be null if none
     *                specified.
     */
    public void asyncWrite(
        Object requestTag, int address, byte[] data, int length, TrcEvent event, CompletionHandler handler)
    {
        final String funcName = "asyncWrite";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "tag=%s,addr=%d,data=%s,length=%d,event=%s",
                requestTag != null? requestTag: "null", address, Arrays.toString(data), length,
                event == null? "null": event.toString());
        }

        requestQueue.add(new Request(requestTag, false, address, data, length, false, event, handler));

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //asyncWrite

    public void preemptiveWrite(int address, byte[] data, int length)
    {
        writeData(address, data, data.length);
    }   //preemptiveWrite

    /**
     * This method is doing an asynchronous write to the device with the specified data and length
     *
     * @param requestTag specifies the tag to identify the request. Can be null if none was provided.
     * @param data specifies the buffer containing the data to write to the device.
     * @param length specifies the number of bytes to write.
     * @param event specifies the event to signal when the request is completed, can be null if none specified.
     * @param handler specifies the completion handler to call when the request is completed, can be null if none
     *                specified.
     */
    public void asyncWrite(Object requestTag, byte[] data, int length, TrcEvent event, CompletionHandler handler)
    {
        asyncWrite(requestTag, -1, data, length, event, handler);
    }   //asyncWrite

    /**
     * This method sends a byte command to the device.
     *
     * @param address specifies the data address if any, can be -1 if no address is required.
     * @param command specifies the command byte.
     * @param waitForCompletion specifies true to wait for write completion.
     */
    public void sendByteCommand(int address, byte command, boolean waitForCompletion)
    {
        final String funcName = "sendByteCommand";
        byte[] data = new byte[1];

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "addr=%d,cmd=0x%x,sync=%s",
                address, command, Boolean.toString(waitForCompletion));
        }

        data[0] = command;
        if (waitForCompletion)
        {
            syncWrite(address, data, data.length);
        }
        else
        {
            //
            // Fire and forget.
            //
            asyncWrite(null, address, data, data.length, null, null);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //sendByteCommand

    /**
     * This method sends a 16-bit command to the device.
     *
     * @param address specifies the data address if any, can be -1 if no address is required.
     * @param command specifies the 16-bit command.
     * @param waitForCompletion specifies true to wait for write completion.
     */
    public void sendWordCommand(int address, short command, boolean waitForCompletion)
    {
        final String funcName = "sendWordCommand";
        byte[] data = new byte[2];

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "addr=%d,cmd=0x%x,sync=%s",
                address, command, Boolean.toString(waitForCompletion));
        }

        data[0] = (byte)(command & 0xff);
        data[1] = (byte)(command >> 8);
        if (waitForCompletion)
        {
            syncWrite(address, data, data.length);
        }
        else
        {
            //
            // Fire and forget.
            //
            asyncWrite(null, address, data, data.length, null, null);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //sendWordCommand

    //
    // Implements Runnable interface.
    //

    /**
     * This method runs the periodic processing task.
     */
    @Override
    public void run()
    {
        final String funcName = "run";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.CALLBK);
        }

        while (!Thread.interrupted())
        {
            long requestStartTime = TrcUtil.getCurrentTimeMillis();

            if (isTaskEnabled())
            {
                //
                // Don't remove the request yet. If it is a read request and the handler is rejecting the data, let
                // the request stays at the head of the queue so it can retry the read request.
                //
                Request request = requestQueue.peek();

                if (request != null)
                {
                    double startTime;
                    double elapsedTime;

                    startTime = TrcUtil.getCurrentTime();
                    if (request.readRequest)
                    {
                        request.buffer = readData(request.address, request.length);
                        request.error = request.buffer == null;
                    }
                    else
                    {
                        int length = writeData(request.address, request.buffer, request.length);
                        request.error = length != request.length;
                        request.length = length;
                    }
                    elapsedTime = TrcUtil.getCurrentTime() - startTime;
                    totalTime += elapsedTime;
                    totalRequests++;
                    if (perfTracer != null)
                    {
                        perfTracer.traceInfo(funcName, "Average request time = %.3f msec", totalTime/totalRequests);
                    }

                    if (request.event != null)
                    {
                        request.event.set(true);
                    }

                    if (request.handler != null)
                    {
                        if (request.readRequest)
                        {
                            if (!request.handler.readCompletion(
                                    request.requestTag, request.address, request.buffer, request.error))
                            {
                                //
                                // The handler accepted the data, so remove the request from the head of the queue.
                                //
                                request = requestQueue.poll();
                            }
                        }
                        else
                        {
                            request.handler.writeCompletion(
                                request.requestTag, request.address, request.length, request.error);
                            //
                            // Write request completed, remove it from head of the queue.
                            //
                            request = requestQueue.poll();
                        }
                    }
                    else
                    {
                        //
                        // There is no handler, we are done. Remove the request from the head of the queue.
                        //
                        request = requestQueue.poll();
                    }

                    if (request.readRequest && request.repeat)
                    {
                        //
                        // This is a repeat request, add it back to the tail of the queue.
                        //
                        requestQueue.add(request);
                    }
                }
            }

            if (processingInterval > 0)
            {
                long sleepTime = processingInterval - (TrcUtil.getCurrentTimeMillis() - requestStartTime);
                TrcUtil.sleep(sleepTime);
            }
            else
            {
                Thread.yield();
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.CALLBK);
        }
    }   //run

}   //class TrcSerialBusDevice
