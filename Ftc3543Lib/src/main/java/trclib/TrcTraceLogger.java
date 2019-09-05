/*
 * Copyright (c) 2018 Titan Robotics Club (http://www.titanrobotics.com)
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

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.concurrent.LinkedBlockingQueue;

public class TrcTraceLogger
{
    private static final String moduleName = "TrcTraceLogger";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private final String traceLogName;
    private final LinkedBlockingQueue<String> msgQueue;

    private PrintWriter traceLog = null;
    private volatile Thread loggerThread = null;
    private volatile boolean enabled = false;
    private volatile TrcDbgTrace perfTracer = null;
    private double totalNanoTime = 0.0;
    private int totalMessages = 0;

    /**
     * Constructor: Create an instance of the trace logger.
     *
     * @param traceLogName specifies the log file name.
     */
    public TrcTraceLogger(String traceLogName)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                    TrcDbgTrace.getGlobalTracer():
                    new TrcDbgTrace(moduleName + "." + traceLogName, tracingEnabled, traceLevel, msgLevel);
        }

        this.traceLogName = traceLogName;
        msgQueue = new LinkedBlockingQueue<>();
    }   //TrcTraceLogger

    /**
     * This method returns the trace log name.
     *
     * @return trace log name, "None" if no log file opened.
     */
    @Override
    public String toString()
    {
        return traceLogName;
    }   //toString

    /**
     * This method enables/disables the trace logger thread.
     *
     * @param enabled specifies true to enable logger thread, false to disable.
     */
    public synchronized void setEnabled(boolean enabled)
    {
        if (loggerThread == null && enabled)
        {
            //
            // Trace logger was not enabled, somebody wants to enable it.
            // Open the log file for append and create the logger thread.
            //
            try
            {
                traceLog = new PrintWriter(new BufferedWriter(new FileWriter(traceLogName, true)));
            }
            catch (IOException e)
            {
                throw new RuntimeException("Failed to open trace log file " + traceLogName);
            }
            loggerThread = new Thread(this::loggerTask, traceLogName);
            loggerThread.start();
            this.enabled = true;
        }
        else if (loggerThread != null && !enabled)
        {
            //
            // Trace logger was enabled, somebody wants to disable it.
            //
            if (this.enabled)
            {
                //
                // Make sure the trace logger is indeed enabled. The message queue may not be empty. So we need to
                // signal termination but allow the logger thread to empty the queue before exiting.
                // If trace logger is already disabled and the loggerThread is still active, it means the thread is
                // busy emptying its queue. So we don't need to double signal termination.
                //
                this.enabled = false;
                loggerThread.interrupt();
            }
        }
    }   //setEnabled

    /**
     * This method checks if the trace log is enabled.
     *
     * @return true if trace log is enabled, false if disabled.
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
     * This method is called to log a message to the log file.
     *
     * @param msg specifies the message to be logged.
     */
    public synchronized boolean logMessage(String msg)
    {
        boolean success = false;

        if (isEnabled())
        {
            success = msgQueue.add(msg);
        }

        return success;
    }   //logMessage

    /**
     * This method writes the message to the trace log and also keeps track of logging performance.
     *
     * @param msg specifies the message to be logged.
     */
    private void writeMessage(String msg)
    {
        long startNanoTime = TrcUtil.getCurrentTimeNanos();
        traceLog.print(msg + "\r\n");
        traceLog.flush();
        double elapsedNanoTime = TrcUtil.getCurrentTimeNanos() - startNanoTime;
        totalNanoTime += elapsedNanoTime;
        totalMessages++;
        //
        // Make sure we don't recursively log the performance message itself.
        //
        if (perfTracer != null && !msg.startsWith(moduleName + "." + traceLogName)) //TODO: test this!
        {
            perfTracer.traceInfo(moduleName + "." + traceLogName, "Avg message log time = %.3f msec",
                    totalNanoTime/totalMessages/1000000000.0);
        }
    }   //writeMessage

    /**
     * This method closes the trace log file.
     */
    private void closeTraceLog()
    {
        if (traceLog != null)
        {
            traceLog.close();
            traceLog = null;
        }
    }   //closeTraceLog

    /**
     * This method is called when the logger thread is started. It processes all messages in the message queue when
     * they arrive. If the message queue is empty, the thread is blocked until a new message arrives. Therefore,
     * this thread only runs when there are messages in the queue. If this thread is interrupted, it will exit
     * only after all the remaining messages in the queue are written to the log.
     */
    private void loggerTask()
    {
        final String funcName = "loggerTask";
        String msg;

        if (debugEnabled)
        {
            dbgTrace.traceInfo("Trace Logger %s starting...", traceLogName);
        }

        while (!Thread.currentThread().isInterrupted())
        {
            try
            {
                msg = msgQueue.take();
                writeMessage(msg);
                if (debugEnabled)
                {
                    dbgTrace.traceInfo(funcName, "[%.3f] Logging message <%s>", TrcUtil.getCurrentTime(), msg);
                }
            }
            catch (InterruptedException e)
            {
                if (debugEnabled)
                {
                    dbgTrace.traceInfo(funcName, "Terminating Trace Logger %s", traceLogName);
                }
                break;
            }
        }
        //
        // The thread is terminating, empty the queue before exiting.
        //
        while ((msg = msgQueue.poll()) != null)
        {
            writeMessage(msg);
            if (debugEnabled)
            {
                dbgTrace.traceInfo(funcName, "[%.3f] Emptying message <%s>", TrcUtil.getCurrentTime(), msg);
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceInfo(funcName, "Closing Trace Log %s", traceLogName);
        }

        closeTraceLog();
        loggerThread = null;
    }   //loggerTask

}   //class TrcTraceLogger
