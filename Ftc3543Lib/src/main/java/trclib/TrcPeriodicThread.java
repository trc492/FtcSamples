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
 * This class implements a platform independent periodic task by using a separate thread. When enabled, the thread
 * periodically calls the runPeriodic method. Generally, this class is used by TrcTaskMgr to create a standalone
 * thread that runs the periodic task when STANDALONE_TASK is registered. By doing so, the standalone task doesn't
 * slow down the main robot thread even if it is performing something that may block for long period of time.
 * In rare occasions, this class can also be used by a sensor that requires long period of time to acquire its
 * data but arguably, one could just call TrcTaskMgr to create a STANDALONE_TASK instead. In other words, this
 * class is mainly used by TrcTaskMgr, there is really no reason for others to use this class. One should always
 * use TrcTaskMgr to create a STANDALONE_TASK.
 *
 * @param <T> specifies the data type that the periodic task will be acquiring/processing.
 */
public class TrcPeriodicThread<T>
{
    private static final String moduleName = "TrcPeriodicThread";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    public interface PeriodicTask
    {
        void runPeriodic(Object context);
    }   //interface PeriodicTask

    /**
     * This class keeps track of the state of the periodic task. It also provides thread synchronization control to
     * make sure the integrity of the task state.
     */
    private class TaskState
    {
        private final Thread periodicThread;
        private boolean taskEnabled;
        private T data;

        /**
         * Constructor: Create an instance of the object.
         */
        public TaskState(String instanceName, Runnable runnable, int taskPriority)
        {
            periodicThread = new Thread(runnable, instanceName);
            periodicThread.setPriority(taskPriority);
            taskEnabled = false;
            data = null;
            periodicThread.start();
        }   //TaskState

        /**
         * This method checks if the periodic task has been terminated.
         *
         * @return true if task has been terminated, false otherwise.
         */
        public boolean isTaskTerminated()
        {
            return !periodicThread.isAlive();
        }   //isTaskTerminated

        /**
         * This method is called to terminate the periodic task.
         */
        public void terminateTask()
        {
            periodicThread.interrupt();
        }   //terminateTask

        /**
         * This method checks if the periodic task is enabled.
         *
         * @return true if task is enabled, false otherwise.
         */
        public synchronized boolean isTaskEnabled()
        {
            return taskEnabled && periodicThread.isAlive();
        }   //isTaskEnabled

        /**
         * This method enables/disables the periodic task. If this is called to disable the task, the task will be
         * set to a paused state. The operation will be resumed when this is called to enable it again.
         *
         * @param enabled specifies true to enable periodic task, false to disable.
         */
        public synchronized void setTaskEnabled(boolean enabled)
        {
            if (periodicThread.isAlive())
            {
                taskEnabled = enabled;
            }
            else
            {
                //
                // Thread is not alive, make sure the task state is disabled.
                //
                taskEnabled = false;
            }
        }   //setTaskEnabled

        /**
         * This method returns the last data object associated with the task. If there is no new data since the last
         * call, it will return null. This method, along with setData(), provides a thread-safe way to access the data
         * produced or processed by the thread.
         *
         * @return newly acquired data if any, null if none.
         */
        public synchronized T getData()
        {
            T newData = null;

            if (periodicThread.isAlive())
            {
                //
                // Consume the data by transferring it out.
                //
                newData = data;
                data = null;
            }

            return newData;
        }   //getData

        /**
         * This method is called to set new data after new data have been acquired/processed. This method, along with
         * getData(), provides a thread-safe way to access the data produced or processed by the thread.
         *
         * @param data specifies newly acquired/processed data. 
         */
        public synchronized void setData(T data)
        {
            if (periodicThread.isAlive())
            {
                this.data = data;
            }
        }   //setData

    }   //class TaskState

    private static volatile int numActiveThreads = 0;
    private final String instanceName;
    private final PeriodicTask task;
    private final Object context;
    private final TaskState taskState;
    private volatile long processingInterval = 0;   // in msec

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param task specifies the periodic task the thread is to execute.
     * @param context specifies the task context to be passed to the periodic thread.
     * @param taskPriority specifies the periodic thread priority.
     */
    public TrcPeriodicThread(final String instanceName, PeriodicTask task, Object context, int taskPriority)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
        this.task = task;
        this.context = context;
        taskState = new TaskState(instanceName, this::run, taskPriority);
    }   //TrcPeriodicThread

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param task specifies the periodic task the thread is to execute.
     * @param context specifies the task context to be passed to the periodic thread.
     */
    public TrcPeriodicThread(final String instanceName, PeriodicTask task, Object context)
    {
        this(instanceName, task, context, Thread.NORM_PRIORITY);
    }   //TrcPeriodicThread

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
     * This method returns the number of active threads.
     *
     * @return number of active threads.
     */
    public static int getNumActiveThreads()
    {
        return numActiveThreads;
    }   //getNumActiveThreads

    /**
     * This method is called to terminate the periodic task. Once this is called, no other method in this class
     * should be called except for isTaskTerminated().
     */
    public void terminateTask()
    {
        taskState.terminateTask();
    }   //terminateTask

    /**
     * This method checks if the periodic task has been terminated.
     *
     * @return true if periodic task is terminated, false otherwise.
     */
    public boolean isTaskTerminated()
    {
        return taskState.isTaskTerminated();
    }   //isTaskTerminated

    /**
     * This method enables/disables the periodic task. As long as the task is enabled, it will continue to
     * acquire/process data.
     *
     * @param enabled specifies true to enable periodic task, false to disable.
     */
    public void setTaskEnabled(boolean enabled)
    {
        final String funcName = "setTaskEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enabled=%s", enabled);
        }

        taskState.setTaskEnabled(enabled);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setTaskEnabled

    /**
     * This method returns the state of the periodic task.
     *
     * @return true if the periodic task is enabled, false otherwise.
     */
    public boolean isTaskEnabled()
    {
        final String funcName = "isTaskEnabled";
        boolean enabled = taskState.isTaskEnabled();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", enabled);
        }

        return enabled;
    }   //isTaskEnabled

    /**
     * This method sets the periodic task processing interval.
     *
     * @param interval specifies the processing interval in msec. If 0, process as fast as the CPU can run.
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
     * This method returns the periodic task processing interval.
     *
     * @return periodic task processing interval in msec.
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
     * This method is called to set new data after new data have been acquired/processed.
     *
     * @param data specifies newly acquired/processed data. 
     */
    public void setData(T data)
    {
        taskState.setData(data);
    }   //setData

    /**
     * This method returns the acquired/processed data. If nothing found, it returns null.
     *
     * @return acquired/processed data, null if nothing found.
     */
    public T getData()
    {
        final String funcName = "getData";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        return taskState.getData();
    }   //getData

    //
    // Implements Runnable interface.
    //

    /**
     * This method runs the periodic processing task.
     */
    public void run()
    {
        final String funcName = "run";
        @SuppressWarnings("unused") long totalThreadNanoTime = 0;
        @SuppressWarnings("unused") int loopCount = 0;

        numActiveThreads++;
        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.CALLBK);
            Thread thread = Thread.currentThread();
            dbgTrace.traceInfo(moduleName, "Thread %d: name=%s, tid=%d, priority=%d, group=%s",
                    numActiveThreads, instanceName, thread.getId(), thread.getPriority(), thread.getThreadGroup());
        }

        while (!Thread.interrupted())
        {
            long startNanoTime = TrcUtil.getCurrentTimeNanos();
            long elapsedNanoTime;

            if (taskState.isTaskEnabled())
            {
                task.runPeriodic(context);
                elapsedNanoTime = TrcUtil.getCurrentTimeNanos() - startNanoTime;
                totalThreadNanoTime += elapsedNanoTime;
                loopCount++;

                if (debugEnabled)
                {
                    dbgTrace.traceVerbose(funcName, "%s: start=%.6f, elapsed=%.6f",
                            instanceName, startNanoTime/1000000000.0, elapsedNanoTime/1000000000.0);
                }
            }

            if (processingInterval > 0)
            {
                long sleepTime = processingInterval - (TrcUtil.getCurrentTimeNanos() - startNanoTime)/1000000;
                //
                // If the processing time does not use up the processingInterval time, make the thread sleep the
                // remaining time left.
                //
                if (sleepTime > 0)
                {
                    try
                    {
                        Thread.sleep(sleepTime);
                    }
                    catch (InterruptedException e)
                    {
                        break;
                    }
                }
            }
            else
            {
                //
                // If the thread does not have a processing interval, we should at least yield to prevent going into
                // a tight loop possibly doing nothing.
                //
                Thread.yield();
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.CALLBK);
            dbgTrace.traceInfo(moduleName, "Exiting thread %d: %s (AvgLoopTime=%.6f)",
                    numActiveThreads, instanceName, totalThreadNanoTime/1000000000.0/loopCount);
        }

        numActiveThreads--;
    }   //run

}   //class TrcPeriodicThread
