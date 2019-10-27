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
 * This class implements a trigger for a digital input device. A digital input trigger consists of a digital input
 * device. It monitors the device state and calls the notification handler if the state changes.
 */
public class TrcDigitalInputTrigger extends TrcTrigger
{
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

    private final TrcDigitalInput digitalInput;
    private final TriggerHandler eventHandler;
    private Boolean prevState = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param digitalInput specifies the digital input device.
     * @param eventHandler specifies the object that will be called to handle the digital input device state change.
     */
    public TrcDigitalInputTrigger(
        final String instanceName, final TrcDigitalInput digitalInput, final TriggerHandler eventHandler)
    {
        super(instanceName, null, null);

        if (digitalInput == null || eventHandler == null)
        {
            throw new NullPointerException("DigitalInput/EventHandler must be provided");
        }

        this.digitalInput = digitalInput;
        this.eventHandler = eventHandler;
    }   //TrcDigitalInputTrigger

    /**
     * This method enables/disables the task that monitors the device state.
     *
     * @param enabled specifies true to enable the task, false to disable.
     */
    @Override
    public synchronized void setEnabled(boolean enabled)
    {
        if (enabled)
        {
            super.setInputAndNotifier(this::isTriggered, this::notifyEvent);
            prevState = null;
        }
        super.setEnabled(enabled);
    }   //setEnabled

    /**
     * This method is called periodically to check if the digital input device has changed state.
     */
    private synchronized boolean isTriggered()
    {
        final String funcName = "isTriggered";
        boolean triggered = false;
        boolean currState = digitalInput.isActive();

        if (prevState == null || currState != prevState)
        {
            triggered = true;
            prevState = currState;

            if (debugEnabled)
            {
                dbgTrace.traceInfo(funcName, "%s triggered (state=%s)", instanceName, currState);
            }
        }

        return triggered;
    }   //isTriggered

    /**
     * This method is called when the trigger condition is met and it will in turn call the trigger event handler.
     */
    private synchronized void notifyEvent()
    {
        eventHandler.triggerEvent(prevState);
    }   //notifyEvent;

}   //class TrcDigitalInputTrigger
