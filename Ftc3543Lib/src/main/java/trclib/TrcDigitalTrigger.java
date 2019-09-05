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
 * This class implements a digital trigger. A digital trigger consists of a digital input device. It monitors the
 * device state and calls the notification handler if the state changes.
 */
public class TrcDigitalTrigger
{
    private static final String moduleName = "TrcDigitalTrigger";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    /**
     * This interface contains the method for the trigger event handler.
     */
    public interface TriggerHandler
    {
        /**
         * This method is called when the digital input device has changed state.
         *
         * @param active specifies true if the digital device state is active, false otherwise.
         */
        void triggerEvent(boolean active);

    }   //interface TriggerHandler

    private final String instanceName;
    private final TrcDigitalInput digitalInput;
    private final TriggerHandler eventHandler;
    private final TrcTaskMgr.TaskObject triggerTaskObj;
    private Boolean prevState = null;
    private boolean enabled = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param digitalInput specifies the digital input device.
     * @param eventHandler specifies the object that will be called to handle the digital input device state change.
     */
    public TrcDigitalTrigger(
        final String instanceName, final TrcDigitalInput digitalInput, final TriggerHandler eventHandler)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        if (digitalInput == null || eventHandler == null)
        {
            throw new NullPointerException("DigitalInput/EventHandler must be provided");
        }

        this.instanceName = instanceName;
        this.digitalInput = digitalInput;
        this.eventHandler = eventHandler;
        triggerTaskObj = TrcTaskMgr.getInstance().createTask(instanceName + ".triggerTask", this::triggerTask);
    }   //TrcDigitalTrigger

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
            prevState = null;
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
        final String funcName = "triggerTask";
        boolean currState = digitalInput.isActive();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "taskType=%s,runMode=%s", taskType, runMode);
        }

        if (prevState == null || currState != prevState)
        {
            if (eventHandler != null)
            {
                eventHandler.triggerEvent(currState);
            }
            prevState = currState;

            if (debugEnabled)
            {
                dbgTrace.traceInfo(funcName, "%s triggered (state=%s)", instanceName, Boolean.toString(currState));
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //triggerTask

}   //class TrcDigitalTrigger
