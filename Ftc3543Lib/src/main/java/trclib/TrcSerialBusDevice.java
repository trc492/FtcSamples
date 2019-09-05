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
import java.util.Locale;

/**
 * This class implements a platform independent serial bus device. This class is intended to be inherited by a
 * platform dependent serial bus device such as I2C device or Serial Port device that provides synchronous methods
 * to access the device. Optionally, it creates a request queue to allow both synchronous and asynchronous requests
 * to be queued for processing. The request queue is processed by a separate thread for asynchronous access. If
 * no request queue is specified, only synchronous requests are allowed.
 */
public abstract class TrcSerialBusDevice
{
    protected static final String moduleName = "TrcSerialBusDevice";
    protected static final boolean debugEnabled = false;
    protected static final boolean tracingEnabled = false;
    protected static final boolean useGlobalTracer = false;
    protected static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    protected static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    protected TrcDbgTrace dbgTrace = null;

    /**
     * This method is called to read data from the device synchronously with the specified length.
     *
     * @param address specifies the data address if any, can be -1 if no address is required.
     * @param length specifies the number of bytes to read.
     * @return a byte array containing the data read.
     */
    public abstract byte[] readData(int address, int length);

    /**
     * This method is called to write data to the device synchronously with the specified data buffer and length.
     *
     * @param address specifies the data address if any, can be -1 if no address is required.
     * @param buffer specifies the buffer containing the data to be written to the device.
     * @param length specifies the number of bytes to write.
     * @return number of bytes written.
     */
    public abstract int writeData(int address, byte[] buffer, int length);

    /**
     * This class implements a request. Typically, a request will be put into a FIFO request queue so that each
     * request will be processed in the order they came in.
     */
    public class Request
    {
        public Object requestId;
        public boolean readRequest;
        public int address;
        public byte[] buffer;
        public int length;
        public TrcEvent completionEvent;
        public TrcNotifier.Receiver completionHandler;
        public boolean canceled;

        /**
         * Constructor: Create an instance of the object.
         *
         * @param requestId specifies the request ID and is not interpreted by the TrcSerialBusDevice class.
         *                    it is just passed back to the requester's notification handler.
         * @param readRequest specifies true for a read request, false for a write request.
         * @param address specifies the data address if any, can be -1 if no address is required.
         * @param buffer specifies the buffer that contains data for a write request, ignored for read request.
         * @param length specifies the number of bytes to read or write.
         * @param completionEvent specifies the event to signal when the request is completed.
         * @param completionHandler specifies the notification handler to call when the request is completed,
         *                          can be null if none specified.
         */
        public Request(
            Object requestId, boolean readRequest, int address, byte[] buffer, int length, TrcEvent completionEvent,
            TrcNotifier.Receiver completionHandler)
        {
            this.requestId = requestId;
            this.readRequest = readRequest;
            this.address = address;
            this.buffer = buffer;
            this.length = length;
            this.completionEvent = completionEvent;
            this.completionHandler = completionHandler;
            this.canceled = false;
        }   //Request

        /**
         * This method returns the request info as a string.
         *
         * @return request info string.
         */
        @Override
        public String toString()
        {
            return String.format(Locale.US, "%s: %s, addr=%d, buff=%s, len=%d, event=%s, canceled=%s",
                requestId != null? requestId: "null", readRequest? "Read": "Write", address,
                buffer == null? "null": Arrays.toString(buffer), length, completionEvent, canceled);
        }   //toString

    }   //class Request

    private final String instanceName;
    private final TrcRequestQueue<Request> requestQueue;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public TrcSerialBusDevice(String instanceName, boolean useRequestQueue)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
        requestQueue = useRequestQueue ? new TrcRequestQueue<>(instanceName) : null;
    }   //TrcSerialBusDevice

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @Override
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method enables/disables the serial bus device thread.
     *
     * @param enabled specifies true to enable device thread, false to disable.
     */
    public synchronized void setEnabled(boolean enabled)
    {
        if (requestQueue != null)
        {
            requestQueue.setEnabled(enabled);
        }
    }   //setEnabled

    /**
     * This method checks if the serial bus device is enabled.
     *
     * @return true if serial bus device is enabled, false if disabled.
     */
    public synchronized boolean isEnabled()
    {
        return requestQueue == null || requestQueue.isEnabled();
    }   //isEnabled

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

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "addr=%d,len=%d", address, length);
        }

        if (!isEnabled())
        {
            throw new RuntimeException("Device is not enabled, must call setEnabled first.");
        }

        byte[] data = null;
        if (requestQueue != null)
        {
            TrcEvent completionEvent = new TrcEvent(instanceName + "." + funcName);
            Request request = new Request(null, true, address, null, length, completionEvent, null);
            TrcRequestQueue<Request>.RequestEntry entry = requestQueue.add(request, this::requestHandler, false);

            while (!completionEvent.isSignaled())
            {
                Thread.yield();
            }

            if (!entry.isCanceled())
            {
                data = request.buffer;
                request.buffer = null;
            }
        }
        else
        {
            data = readData(address, length);
        }

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

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "addr=%d,data=%s,length=%d",
                address, Arrays.toString(data), length);
        }

        if (!isEnabled())
        {
            throw new RuntimeException("Must call setEnabled first.");
        }

        int bytesWritten = 0;
        if (requestQueue != null)
        {
            TrcEvent completionEvent = new TrcEvent(instanceName + "." + funcName);
            Request request = new Request(null, false, address, data, length, completionEvent, null);
            TrcRequestQueue<Request>.RequestEntry  entry = requestQueue.add(request, this::requestHandler, false);

            while (!completionEvent.isSignaled())
            {
                Thread.yield();
            }

            if (!entry.isCanceled())
            {
                bytesWritten = request.length;
            }
        }
        else
        {
            bytesWritten = writeData(address, data, length);
        }

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
     * @param requestId specifies the request ID and is not interpreted by the TrcSerialBusDevice class.
     *                    it is just passed back to the requester's notification handler.
     * @param address specifies the data address if any, can be -1 if no address is required.
     * @param length specifies the number of bytes to read.
     * @param repeat specifies true to re-queue the request when completed.
     * @param completionEvent specifies the event to signal when the request is completed,
     *                        can be null if none specified.
     * @param completionHandler specifies the notification handler to call when the request is completed,
     *                          can be null if none specified.
     */
    public void asyncRead(
        Object requestId, int address, int length, boolean repeat, TrcEvent completionEvent,
        TrcNotifier.Receiver completionHandler)
    {
        final String funcName = "asyncRead";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "Id=%s,addr=%d,len=%d,repeat=%s,event=%s",
                requestId == null? "null": requestId, address, length, repeat, completionEvent);
        }

        if (requestQueue != null)
        {
            Request request = new Request(requestId, true, address, null, length, completionEvent, completionHandler);
            requestQueue.add(request, this::requestHandler, repeat);
        }
        else
        {
            throw new UnsupportedOperationException("asyncRead is not support without a request queue.");
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //asyncRead

    /**
     * This method is doing an asynchronous read from the device with the specified length to read.
     *
     * @param requestCtxt specifies the request context and is not interpreted by the TrcSerialBusDevice class.
     *                    it is just passed back to the requester's notification handler.
     * @param address specifies the data address if any, can be -1 if no address is required.
     * @param length specifies the number of bytes to read.
     * @param event specifies the event to signal when the request is completed, can be null if none specified.
     * @param handler specifies the notification handler to call when the request is completed, can be null if none
     *                specified.
     */
    public void asyncRead(Object requestCtxt, int address, int length, TrcEvent event, TrcNotifier.Receiver handler)
    {
        asyncRead(requestCtxt, address, length, false, event, handler);
    }   //asyncRead

    /**
     * This method is doing an asynchronous read from the device with the specified length to read.
     *
     * @param requestCtxt specifies the request context and is not interpreted by the TrcSerialBusDevice class.
     *                    it is just passed back to the requester's notification handler.
     * @param length specifies the number of bytes to read.
     * @param repeat specifies true to re-queue the request when completed.
     * @param event specifies the event to signal when the request is completed, can be null if none specified.
     * @param handler specifies the notification handler to call when the request is completed, can be null if none
     *                specified.
     */
    public void asyncRead(Object requestCtxt, int length, boolean repeat, TrcEvent event, TrcNotifier.Receiver handler)
    {
        asyncRead(requestCtxt, -1, length, repeat, event, handler);
    }   //asyncRead

    /**
     * This method is doing an asynchronous read from the device with the specified length to read.
     *
     * @param requestCtxt specifies the request context and is not interpreted by the TrcSerialBusDevice class.
     *                    it is just passed back to the requester's notification handler.
     * @param length specifies the number of bytes to read.
     * @param event specifies the event to signal when the request is completed, can be null if none specified.
     * @param handler specifies the notification handler to call when the request is completed, can be null if none
     *                specified.
     */
    public void asyncRead(Object requestCtxt, int length, TrcEvent event, TrcNotifier.Receiver handler)
    {
        asyncRead(requestCtxt, -1, length, false, event, handler);
    }   //asyncRead

    /**
     * This method is doing an asynchronous write to the device with the specified data and length
     *
     * @param requestId specifies the request ID and is not interpreted by the TrcSerialBusDevice class.
     *                    it is just passed back to the requester's notification handler.
     * @param address specifies the data address if any, can be -1 if no address is required.
     * @param data specifies the buffer containing the data to write to the device.
     * @param length specifies the number of bytes to write.
     * @param completionEvent specifies the event to signal when the request is completed,
     *                        can be null if none specified.
     * @param completionHandler specifies the notification handler to call when the request is completed,
     *                          can be null if none specified.
     */
    public void asyncWrite(
        Object requestId, int address, byte[] data, int length, TrcEvent completionEvent,
        TrcNotifier.Receiver completionHandler)
    {
        final String funcName = "asyncWrite";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "ctxt=%s,addr=%d,data=%s,length=%d,event=%s",
                requestId == null? "null": requestId, address, Arrays.toString(data), length, completionEvent);
        }

        if (requestQueue != null)
        {
            Request request = new Request(requestId, false, address, data, length, completionEvent, completionHandler);
            requestQueue.add(request, this::requestHandler, false);
        }
        else
        {
            throw new UnsupportedOperationException("asyncWrite is not support without a request queue.");
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //asyncWrite

    /**
     * This method is doing an asynchronous write to the device with the specified data and length
     *
     * @param requestCtxt specifies the request context and is not interpreted by the TrcSerialBusDevice class.
     *                    it is just passed back to the requester's notification handler.
     * @param data specifies the buffer containing the data to write to the device.
     * @param length specifies the number of bytes to write.
     * @param event specifies the event to signal when the request is completed, can be null if none specified.
     * @param handler specifies the notification handler to call when the request is completed, can be null if none
     *                specified.
     */
    public void asyncWrite(Object requestCtxt, byte[] data, int length, TrcEvent event, TrcNotifier.Receiver handler)
    {
        asyncWrite(requestCtxt, -1, data, length, event, handler);
    }   //asyncWrite

    /**
     * This method writes the data to the device preemptively bypassing the queue.
     *
     * @param address specifies the data address if any, can be -1 if no address is required.
     * @param data specifies the buffer containing the data to write to the device.
     * @param length specifies the number of bytes to write.
     */
    public synchronized void preemptiveWrite(int address, byte[] data, int length)
    {
        if (requestQueue != null)
        {
            requestQueue.addPriorityRequest(
                new Request(null, false, address, data, length, null, null), this::requestHandler);
        }
        else
        {
            throw new UnsupportedOperationException("preemptiveWrite is not support without a request queue.");
        }
    }   //preemptiveWrite

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

    /**
     * This method processes a request.
     *
     * @param context specifies the request entry to be processed.
     */
    private void requestHandler(Object context)
    {
        final String funcName = "requestHandler";
        @SuppressWarnings("unchecked")
        TrcRequestQueue<Request>.RequestEntry entry = (TrcRequestQueue<Request>.RequestEntry) context;
        Request request = entry.getRequest();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "request=%s", request);
        }

        request.canceled = entry.isCanceled();
        if (!request.canceled)
        {
            if (request.readRequest)
            {
                request.buffer = readData(request.address, request.length);
                if (debugEnabled)
                {
                    if (request.buffer != null)
                    {
                        dbgTrace.traceInfo(funcName, "readData(addr=0x%x,len=%d)=%s",
                        request.address, request.length, Arrays.toString(request.buffer));
    
                    }
                }
            }
            else
            {
                request.length = writeData(request.address, request.buffer, request.length);
            }
        }

        if (request.completionEvent != null)
        {
            request.completionEvent.set(true);
        }

        if (request.completionHandler != null)
        {
            request.completionHandler.notify(request);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //requestHandler

}   //class TrcSerialBusDevice
