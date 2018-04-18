/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
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
 * This class implements a timer that will generate an event when the time has expired. This is useful for doing
 * delays in autonomous.
 */
public class TrcTimer
{
    private static final String moduleName = "TrcTimer";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private final String instanceName;
    private final TrcTaskMgr.TaskObject preContinuousTaskObj;
    private double expiredTime;
    private boolean enabled;
    private boolean expired;
    private boolean canceled;
    private TrcEvent notifyEvent;

    /**
     * Constructor: Creates an instance of the timer with the given name.
     *
     * @param instanceName specifies the name to identify this instance of the timer.
     */
    public TrcTimer(final String instanceName)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
        preContinuousTaskObj = TrcTaskMgr.getInstance().createTask(
            instanceName + ".preContinuous", this::preContinuousTask);
        expiredTime = 0.0;
        enabled = false;
        expired = false;
        canceled = false;
        notifyEvent = null;
    }   //TrcTimer

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
     * This methods sets the expire time relative to the current time. When the time expires, it will signal the
     * given event.
     *
     * @param time specifies the expire time in seconds relative to the current time.
     * @param event specifies the event to signal when time has expired.
     */
    public void set(double time, TrcEvent event)
    {
        final String funcName = "set";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                                "time=%f,event=%s", time, event != null? event.toString(): "null");
        }

        expired = false;
        canceled = false;
        expiredTime = TrcUtil.getCurrentTime() + time;
        if (event != null)
        {
            event.clear();
        }
        notifyEvent = event;
        setTaskEnabled(true);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //set

    /**
     * This method checks if the timer has expired.
     *
     * @return true if the timer has expired, false otherwise.
     */
    public boolean isExpired()
    {
        final String funcName = "isExpired";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Boolean.toString(expired));
        }

        return expired;
    }   //isExpired

    /**
     * This method checks if the timer was canceled.
     *
     * @return true if the timer was canceled, false otherwise.
     */
    public boolean isCanceled()
    {
        final String funcName = "isCanceled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Boolean.toString(canceled));
        }

        return canceled;
    }   //isCanceled

    /**
     * This method cancels the timer if it's set but has not expired. If the timer is canceled, the event is signaled.
     */
    public void cancel()
    {
        final String funcName = "cancel";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (enabled)
        {
            setTaskEnabled(false);
            expiredTime = 0.0;
            expired = false;
            notifyEvent.cancel();
            notifyEvent = null;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //cancel

    /**
     * This private method enables/disables the task that checks for timer expiration.
     *
     * @param enabled specifies if the timer task is enabled.
     */
    private void setTaskEnabled(boolean enabled)
    {
        final String funcName = "setTaskEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC, "enabled=%b", enabled);
        }

        if (enabled)
        {
            preContinuousTaskObj.registerTask(TrcTaskMgr.TaskType.PRECONTINUOUS_TASK);
        }
        else
        {
            preContinuousTaskObj.unregisterTask(TrcTaskMgr.TaskType.PRECONTINUOUS_TASK);
        }
        this.enabled = enabled;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC);
        }
    }   //setTaskEnabled

    //
    // Implements TrcTaskMgr.Task
    //

    /**
     * This method runs periodically at the fastest rate and checks if the timer has expired. After the timer expired,
     * the task is disabled and if there is an event object, it will be signaled.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the current robot run mode.
     */
    public void preContinuousTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = "preContinuousTask";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "taskType=%s,runMode=%s", taskType, runMode);
        }

        if (enabled && !expired && TrcUtil.getCurrentTime() >= expiredTime)
        {
            setTaskEnabled(false);

            if (debugEnabled)
            {
                dbgTrace.traceInfo(funcName, "Time expired, notifying %s.",
                                   notifyEvent != null? notifyEvent.toString(): "null");
            }

            if (notifyEvent != null)
            {
                notifyEvent.set(true);
                notifyEvent = null;
            }
            expiredTime = 0.0;
            expired = true;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //preContinuousTask

}   //class TrcTimer
