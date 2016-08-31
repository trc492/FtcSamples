/*
 * Titan Robotics Framework Library
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.net)
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

import java.util.ArrayList;

import hallib.HalUtil;

/**
 * This class implements an event driven state machine. The caller can
 * add multiple events for the state machine to monitor. If one or more
 * events are signaled, the state machine will automatically advance to
 * the specified next state.
 */
public class TrcStateMachine
{
    private static final String moduleName = "TrcStateMachine";
    private static final boolean debugEnabled = false;
    private TrcDbgTrace dbgTrace = null;

    private final String instanceName;
    private ArrayList<TrcEvent> eventList = new ArrayList<TrcEvent>();
    private Object currState = null;
    private Object nextState = null;
    private boolean enabled = false;
    private boolean ready = false;
    private boolean expired = false;
    private double expiredTime = 0.0;
    private boolean waitForAllEvents = false;

    /**
     * Constructor: Creates an instance of the state machine with the given name.
     *
     * @param instanceName specifies the instance name of the state machine.
     */
    public TrcStateMachine(final String instanceName)
    {
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(
                    moduleName + "." + instanceName,
                    false,
                    TrcDbgTrace.TraceLevel.API,
                    TrcDbgTrace.MsgLevel.INFO);
        }

        this.instanceName = instanceName;
    }   //TrcStateMachine

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
     * This method starts the state machine with the given starting state
     * and puts it in ready mode.
     *
     * @param state specifies the starting state.
     */
    public void start(Object state)
    {
        final String funcName = "start";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "state=%d", state);
        }

        eventList.clear();
        currState = state;
        nextState = state;
        enabled = true;
        ready = true;
        expired = false;
        expiredTime = 0.0;
        waitForAllEvents = false;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //start

    /**
     * This method stops the state machine by disabling it.
     */
    public void stop()
    {
        final String funcName = "stop";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        eventList.clear();
        currState = null;
        nextState = null;
        enabled = false;
        ready = false;
        expired = false;
        expiredTime = 0.0;
        waitForAllEvents = false;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //stop

    /**
     * This method returns the current state of the state machine.
     *
     * @return current state of the state machine.
     */
    public Object getState()
    {
        final String funcName = "getState";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "=%d", currState);
        }

        return currState;
    }   //getState

    /**
     * This method sets the current state of the state machine.
     *
     * @param state specifies the state to set the state machine to.
     */
    public void setState(Object state)
    {
        final String funcName = "setState";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "state=%d", state);
        }

        currState = state;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setState

    /**
     * This method checks if the state machine is enabled.
     *
     * @return true if state machine is enabled, false otherwise.
     */
    public boolean isEnabled()
    {
        final String funcName = "isEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "=%s", Boolean.toString(enabled));
        }

        return enabled;
    }   //isEnabled

    /**
     * This method checks if the state machine is in ready mode. If not,
     * it will enumerate all the events it is monitoring and make sure
     * if any or all of them are signaled as the condition for putting
     * the state machine in ready mode.
     *
     * @return true if the state machine is in ready mode, false otherwise.
     */
    public boolean isReady()
    {
        final String funcName = "isReady";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        //
        // If the state machine is enabled but not ready, check all events if the
        // state machine should be put back in ready mode.
        //
        if (enabled && !ready)
        {
            //
            // If a timeout was specifies and we have past the timeout time,
            // we will put the state machine back to ready mode but indicate
            // the timeout had expired.
            //
            if (expiredTime > 0.0 && HalUtil.getCurrentTime() >= expiredTime)
            {
                expiredTime = 0.0;
                ready = true;
                expired = true;
            }
            else
            {
                //
                // Count the number of signaled events.
                //
                int count = 0;
                for (int i = 0; i < eventList.size(); i++)
                {
                    TrcEvent event = eventList.get(i);
                    if (event.isSignaled() || event.isCanceled())
                    {
                        count++;
                    }
                }

                //
                // If waitForAllEvents is true, the number of signaled events must
                // equal to the size of the event list (i.e. all events have signaled).
                // If waitForAllEvents is false, then we just need a non-zero count
                // in order to put the state machine back to ready mode.
                //
                if (!waitForAllEvents && count > 0 ||
                    waitForAllEvents && count == eventList.size())
                {
                    ready = true;
                }
            }

            //
            // If we put the state machine back to ready mode, we need to clear
            // all events and the event list to monitor. Then we move the state
            // from the current state to the next state.
            //
            if (ready)
            {
                clearAllEvents();
                eventList.clear();
                currState = nextState;
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "=%s",
                    Boolean.toString(enabled && ready));
        }

        return enabled && ready;
    }   //isReady

    /**
     * This method checks if timeout has happened on waiting for event(s).
     *
     * @return true if a timeout was set and expired, false otherwise.
     */
    public boolean isTimedout()
    {
        final String funcName = "isTimedout";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "=%s", Boolean.toString(expired));
        }

        return expired;
    }   //isTimedout

    /**
     * This method adds an event to the event list to be monitored.
     *
     * @param event specifies the vent to be added to the list.
     */
    public void addEvent(TrcEvent event)
    {
        final String funcName = "addEvent";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "event=%s", event.toString());
        }

        //
        // Only add to the list if the given event is not already in the list.
        //
        if (!eventList.contains(event))
        {
            eventList.add(event);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //addEvent

    /**
     * This method puts the state machine into not ready mode and starts
     * monitoring the events in the list. If waitForAllEvents is false and
     * any event on the list is signaled or waitForAllEvents is true and
     * all events are signaled, the state machine will be put back to ready
     * mode and it will automatically advance to the given next state.
     * If timeout is non-zero, the state machine will be put back to ready
     * mode after timeout has expired even though the required event(s)
     * have not been signaled.
     *
     * @param nextState specifies the next state when the state machine
     *                  becomes ready.
     * @param timeout specifies a timeout value. A zero value means
     *                there is no timeout.
     * @param waitForAllEvents specifies true if all events must be
     *                         signaled for the state machine to go
     *                         ready. If false, any signaled event
     *                         will cause the state machien to go
     *                         ready.
     */
    public void waitForEvents(Object nextState, double timeout, boolean waitForAllEvents)
    {
        final String funcName = "waitForEvents";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "nextState=%d,timeout=%f,waitForAll=%s",
                    nextState, timeout, Boolean.toString(waitForAllEvents));
        }

        this.nextState = nextState;
        this.expiredTime = timeout;
        if (timeout > 0.0)
        {
            this.expiredTime += HalUtil.getCurrentTime();
        }
        this.waitForAllEvents = waitForAllEvents;
        ready = false;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //waitForEvents

    /**
     * This method puts the state machine into not ready mode and starts
     * monitoring the events in the list. If any event on the list is signaled,
     * the state machine will be put back to ready mode and it will automatically
     * advance to the given next state.
     * If timeout is non-zero, the state machine will be put back to ready
     * mode after timeout has expired even though the required event(s)
     * have not been signaled.
     *
     * @param nextState specifies the next state when the state machine
     *                  becomes ready.
     * @param timeout specifies a timeout value. A zero value means
     *                there is no timeout.
     */
    public void waitForEvents(Object nextState, double timeout)
    {
        waitForEvents(nextState, timeout, false);
    }   //waitForEvents

    /**
     * This method puts the state machine into not ready mode and starts
     * monitoring the events in the list. If any event on the list is signaled,
     * the state machine will be put back to ready mode and it will automatically
     * advance to the given next state.
     *
     * @param nextState specifies the next state when the state machine
     *                  becomes ready.
     */
    public void waitForEvents(Object nextState)
    {
        waitForEvents(nextState, 0.0, false);
    }   //waitForEvents

    /**
     * This method clears the signaled state of all the events in the list.
     */
    private void clearAllEvents()
    {
        final String funcName =  "clearAllEvents";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.UTIL);
        }

        for (int i = 0; i < eventList.size(); i++)
        {
            TrcEvent event = eventList.get(i);
            event.clear();;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.UTIL);
        }
    }   //clearAllEvents

}   //class TrcStateMachine
