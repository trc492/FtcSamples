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

package trclib;

import java.util.Locale;
import java.util.concurrent.LinkedBlockingQueue;

/**
 * This class implements a generic request queue that runs on its own thread. It allows the caller to add requests
 * to the end of the queue. The request thread will call the client to process the request asynchronously from the
 * head of the queue. When the request is completed, an optional event will be signaled as well as an optional
 * callback if provided.
 *
 * @param <R> specifies the type of the request.
 */
public class TrcRequestQueue<R>
{
    protected static final String moduleName = "TrcRequestQueue";
    protected static final boolean debugEnabled = false;
    protected static final boolean tracingEnabled = false;
    protected static final boolean useGlobalTracer = false;
    protected static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    protected static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    protected TrcDbgTrace dbgTrace = null;

    /**
     * This class implements a request entry. Typically, an entry will be put into a FIFO request queue so that each
     * entry will be processed in the order they came in.
     */
    public class RequestEntry
    {
        private R request;
        private TrcNotifier.Receiver requestHandler;
        private boolean repeat;
        private boolean canceled;

        /**
         * Constructor: Create an instance of the object.
         *
         * @param request specifies the request.
         * @param requestHandler specifies the request handler to call when the request is up for processing.
         * @param repeat specifies true to re-queue the request when completed.
         */
        public RequestEntry(R request, TrcNotifier.Receiver requestHandler, boolean repeat)
        {
            this.request = request;
            this.requestHandler = requestHandler;
            this.repeat = repeat;
            this.canceled = false;
        }   //RequestEntry

        /**
         * This method retrieves the request object.
         *
         * @return request object.
         */
        public R getRequest()
        {
            return request;
        }   //getRequest

        /**
         * This method checks if the request entry is canceled.
         *
         * @return true if the request entry is canceled, false otherwise.
         */
        public boolean isCanceled()
        {
            return canceled;
        }   //isCanceled

        /**
         * This method returns the request info as a string.
         *
         * @return request info string.
         */
        @Override
        public String toString()
        {
            return String.format(Locale.US, "request=%s, repeat=%s, canceled=%s", request, repeat, canceled);
        }   //toString

    }   //class RequestEntry

    private final String instanceName;
    private final LinkedBlockingQueue<RequestEntry> requestQueue;
    private volatile Thread requestThread = null;
    private boolean enabled = false;
    private RequestEntry priorityRequest = null;
    private TrcDbgTrace perfTracer = null;
    private double totalNanoTime = 0.0;
    private int totalRequests = 0;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public TrcRequestQueue(String instanceName)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
        requestQueue = new LinkedBlockingQueue<>();
    }   //TrcRequestQueue

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
     * This method enables/disables the request queue. On enable, it creates the request thread to start processing
     * request entries in the queue. On disable, it shuts down the request thread and cancels all pending requests
     * still in the queue.
     *
     * @param enabled specifies true to enable request queue, false to disable.
     */
    public synchronized void setEnabled(boolean enabled)
    {
        if (enabled)
        {
            //
            // Enabling request queue, make sure the request queue is not already enabled.
            //
            if (requestThread == null)
            {
                requestThread = new Thread(this::requestTask, instanceName);
                requestThread.start();
                this.enabled = true;
            }
        }
        else if (requestThread != null)
        {
            //
            // Disabling request queue, make sure the request queue is indeed active.
            // The request queue may not be empty. So we need to signal termination but allow the request queue to
            // orderly shut down. If request queue is already disabled but the request thread is still active, it
            // means the thread is busy emptying its queue. So we don't need to double signal termination.
            //
            if (this.enabled)
            {
                this.enabled = false;
                requestThread.interrupt();
            }
        }
    }   //setEnabled

    /**
     * This method checks if the request queue is enabled.
     *
     * @return true if request queue is enabled, false if disabled.
     */
    public synchronized boolean isEnabled()
    {
        return enabled;
    }   //isEnabled

    /**
     * This method enables/disables performance report.
     *
     * @param tracer specifies the tracer to be used for performance tracing, can be null to disable performance
     *               tracing.
     */
    public synchronized void setPerformanceTracer(TrcDbgTrace tracer)
    {
        perfTracer = tracer;
    }   //setPerformanceTracer

    /**
     * This method queues a request at the end of the request queue to be processed asynchronously on a thread.
     *
     * @param request specifies the request to be queued.
     * @param requestHandler specifies the handler to call when the request is up for processing.
     * @param repeat specifies true to re-queue the request when completed.
     * @return request entry added to the end of the queue. It can be used to cancel the request if it is still in
     *         queue.
     */
    public RequestEntry add(R request, TrcNotifier.Receiver requestHandler, boolean repeat)
    {
        final String funcName = "add";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "request=%s,repeat=%s", request, repeat);
        }

        RequestEntry entry = null;
        if (isEnabled())
        {
            entry = new RequestEntry(request, requestHandler, repeat);
            requestQueue.add(entry);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", entry);
        }

        return entry;
    }   //add

    /**
     * This method adds the priority request to the head of the queue. It will be processed once the current active
     * request is done processing. If there is already an existing priority request pending, this request will not
     * be added to the queue and null is returned.
     *
     * @param request specifies the priority request.
     * @param requestHandler specifies the handler to call when the request is up for processing.
     * @return request entry added to the head of the queue. It can be used to cancel the request if it is still in
     *         queue. It may return null if the priority request failed to be added to the queue.
     */
    public synchronized RequestEntry addPriorityRequest(R request, TrcNotifier.Receiver requestHandler)
    {
        final String funcName = "addPriorityRequest";
        RequestEntry entry = null;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "request=%s", request);
        }

        if (priorityRequest == null)
        {
            entry = new RequestEntry(request, requestHandler, false);
            priorityRequest = entry;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", entry);
        }

        return entry;
    }   //addPriorityRequest

    /**
     * This method cancels a request.
     *
     * @param entry specifies the request entry from add or addPriorityRequest to be canceled.
     * @return true if the request entry is found in the queue and canceled, false otherwise.
     */
    public synchronized boolean cancelRequest(RequestEntry entry)
    {
        final String funcName = "cancelRequest";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "entry=%s", entry);
        }

        boolean foundEntry;
        if (entry == priorityRequest)
        {
            priorityRequest = null;
            foundEntry = true;
        }
        else
        {
            foundEntry = requestQueue.contains(entry);
            if (foundEntry) requestQueue.remove(entry);
        }

        if (foundEntry)
        {
            entry.canceled = true;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", foundEntry);
        }

        return foundEntry;
    }   //cancelRequest

    /**
     * This method is called when the request queue thread is started. It processes all entries in the request queue
     * when they arrive. If the request queue is empty, the thread is blocked until a new request arrives. Therefore,
     * this thread only runs when there are requests in the queue. If this thread is interrupted, it will clean up
     * the request queue before exiting.
     */
    private void requestTask()
    {
        final String funcName = "requestTask";
        RequestEntry entry;

        if (debugEnabled)
        {
            dbgTrace.traceInfo(funcName, "RequestQueue %s starting...", instanceName);
        }

        while (!Thread.currentThread().isInterrupted())
        {
            synchronized (this)
            {
                if (priorityRequest != null)
                {
                    entry = priorityRequest;
                    priorityRequest = null;
                }
                else
                {
                    entry = null;
                }
            }

            try
            {
                if (entry == null)
                {
                    entry = requestQueue.take();
                }

                if (debugEnabled)
                {
                    dbgTrace.traceInfo(funcName, "[%.3f] processing request %s", TrcUtil.getCurrentTime(), entry);
                }

                long startNanoTime = TrcUtil.getCurrentTimeNanos();
                entry.requestHandler.notify(entry);
                long elapsedTime = TrcUtil.getCurrentTimeNanos() - startNanoTime;
        
                totalNanoTime += elapsedTime;
                totalRequests++;
        
                if (perfTracer != null)
                {
                    perfTracer.traceInfo(funcName, "Average request process time = %.6f sec", totalNanoTime/totalRequests/1000000000.0);
                }
        
                if (entry.repeat)
                {
                    //
                    // This is a repeat request, add it back to the tail of the queue.
                    //
                    requestQueue.add(entry);
                }
            }
            catch (InterruptedException e)
            {
                if (debugEnabled)
                {
                    dbgTrace.traceInfo(funcName, "Terminating RequestQueue %s", instanceName);
                }
                break;
            }
        }
        //
        // The thread is terminating, empty the queue before exiting and make sure nobody is trying to start another
        // thread until this one has exited.
        //
        synchronized (this)
        {
            while ((entry = requestQueue.poll()) != null)
            {
                if (debugEnabled)
                {
                    dbgTrace.traceInfo(funcName, "[%.3f] Canceling request %s", TrcUtil.getCurrentTime(), entry);
                }
                cancelRequest(entry);
            }

            requestThread = null;
        }

        if (debugEnabled)
        {
            dbgTrace.traceInfo(funcName, "RequestQueue %s is terminated", instanceName);
        }
    }   //requestTask

}   //class TrcRequestQueue
