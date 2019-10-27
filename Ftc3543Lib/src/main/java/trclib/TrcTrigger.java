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

/**
 * This class implements a generic trigger mechanism. A generic trigger consists of an input method that will be
 * called periodically to check if the trigger condition is met. If so, the trigger event notifier is called.
 */
public class TrcTrigger
{
    private static final String moduleName = "TrcTrigger";
    protected static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    protected TrcDbgTrace dbgTrace = null;

    /**
     * This interface contains a method for checking if the trigger condition is met.
     */
    public interface Input
    {
        /**
         * This method is called periodically to check if the trigger condition is met.
         *
         * @return true if the trigger condition is met, false otherwise.
         */
        boolean isTriggered();

    }   //interface Input

    /**
     * This interface contains the method for the notifying a trigger event.
     */
    public interface Notifier
    {
        /**
         * This method is called when input has met the trigger condition.
         */
        void notifyEvent();

    }   //interface Notifier

    protected final String instanceName;
    private Input input;
    private Notifier notifier;
    private final TrcTaskMgr.TaskObject triggerTaskObj;
    private boolean enabled = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param input specifies the object to call for checking the trigger condition.
     * @param notifier specifies the object to call to notify a trigger event.
     */
    public TrcTrigger(String instanceName, Input input, Notifier notifier)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
        this.input = input;
        this.notifier = notifier;
        triggerTaskObj = TrcTaskMgr.getInstance().createTask(instanceName + ".triggerTask", this::triggerTask);
    }   //TrcDigitalInputTrigger

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
     * This method is called by the subclasses to set the input and notifier because they cannot set it in their
     * constructor.
     *
     * @param input specifies the input object.
     * @param notifier specifies the notifier object.
     */
    protected void setInputAndNotifier(Input input, Notifier notifier)
    {
        this.input = input;
        this.notifier = notifier;
    }   //setInputAndNotifier

    /**
     * This method enables/disables the task that monitors the device state.
     *
     * @param enabled specifies true to enable the task, false to disable.
     */
    public synchronized void setEnabled(boolean enabled)
    {
        final String funcName = "setEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enabled=%b", enabled);
        }

        if (enabled)
        {
            if (input == null || notifier == null)
            {
                throw new NullPointerException("Input/Notifier must be set before enabling trigger.");
            }

            triggerTaskObj.registerTask(TrcTaskMgr.TaskType.PRECONTINUOUS_TASK);    //TODO: should use INPUT_TASK
        }
        else
        {
            triggerTaskObj.unregisterTask(TrcTaskMgr.TaskType.PRECONTINUOUS_TASK);
        }
        this.enabled = enabled;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setEnabled

    /**
     * This method checks if the digital trigger is enabled.
     *
     * @return true if enabled, false otherwise.
     */
    public synchronized boolean isEnabled()
    {
        final String funcName = "isEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%b", enabled);
        }

        return enabled;
    }   //isEnabled

    /**
     * This method is called periodically to check if the digital input device has changed state.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is running. (e.g. Autonomous, TeleOp, Test).
     */
    private synchronized void triggerTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        if (input.isTriggered())
        {
            notifier.notifyEvent();
        }
    }   //triggerTask

}   //class TrcTrigger
