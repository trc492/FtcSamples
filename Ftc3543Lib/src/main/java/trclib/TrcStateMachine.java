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

import java.util.ArrayList;

/**
 * This class implements an event driven state machine. The caller can add multiple events for the state machine
 * to monitor. If one or more events are signaled, the state machine will automatically advance to the specified
 * next state.
 *
 * @param <T> specifies the State enum type that list all possible states.
 */
public class TrcStateMachine<T>
{
    private static final String moduleName = "TrcStateMachine";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private final String instanceName;
    private ArrayList<TrcEvent> eventList = new ArrayList<>();
    private T currState = null;
    private T nextState = null;
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
            dbgTrace = new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
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
     * This method starts the state machine with the given starting state and puts it in ready mode.
     *
     * @param state specifies the starting state.
     */
    public void start(T state)
    {
        final String funcName = "start";

        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
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
    public T getState()
    {
        final String funcName = "getState";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", currState.toString());
        }

        return currState;
    }   //getState

    /**
     * This method checks whether the state machine is ready. If so, it returns the current state. If the state
     * machine is not ready, it returns null.
     *
     * @return current state of the state machine, null if state machine is not ready.
     */
    public T checkReadyAndGetState()
    {
        final String funcName = "checkReadyAndGetState";
        T state = null;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (isReady())
        {
            state = getState();
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", state == null? "NotReady": state);
        }

        return state;
    }   //checkReadyGetState

    /**
     * This method sets the current state of the state machine.
     *
     * @param state specifies the state to set the state machine to.
     */
    public void setState(T state)
    {
        final String funcName = "setState";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "state=%s", state.toString());
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
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Boolean.toString(enabled));
        }

        return enabled;
    }   //isEnabled

    /**
     * This method checks if the state machine is in ready mode. If not, it will enumerate all the events it is
     * monitoring and make sure if any or all of them are signaled as the condition for putting the state machine
     * back in ready mode.
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
        // If the state machine is enabled but not ready, check all events if the state machine should be put back
        // in ready mode.
        //
        if (enabled && !ready)
        {
            //
            // If a timeout was specifies and we have past the timeout time, we will put the state machine back to
            // ready mode but indicate the timeout had expired.
            //
            if (expiredTime > 0.0 && TrcUtil.getCurrentTime() >= expiredTime)
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
                // If waitForAllEvents is true, the number of signaled events must equal to the size of the event
                // list (i.e. all events have signaled). If waitForAllEvents is false, then we just need a non-zero
                // count in order to put the state machine back to ready mode.
                //
                if (!waitForAllEvents && count > 0 || waitForAllEvents && count == eventList.size())
                {
                    ready = true;
                }
            }

            //
            // If we put the state machine back to ready mode, we need to clear all events and the event list to
            // monitor. Then we move the state from the current state to the next state.
            //
            if (ready)
            {
                eventList.clear();
                currState = nextState;
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Boolean.toString(enabled && ready));
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
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Boolean.toString(expired));
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
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "event=%s", event.toString());
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
     * This method puts the state machine into not ready mode and starts monitoring the events in the list. If
     * waitForAllEvents is false and any event on the list is signaled or waitForAllEvents is true and all events
     * are signaled, the state machine will be put back to ready mode and it will automatically advance to the
     * given next state. If timeout is non-zero, the state machine will be put back to ready mode after timeout
     * has expired even though the required event(s) have not been signaled.
     *
     * @param nextState specifies the next state when the state machine becomes ready.
     * @param timeout specifies a timeout value. A zero value means there is no timeout.
     * @param waitForAllEvents specifies true if all events must be signaled for the state machine to go ready.
     *                         If false, any signaled event will cause the state machine to go ready.
     */
    public void waitForEvents(T nextState, double timeout, boolean waitForAllEvents)
    {
        final String funcName = "waitForEvents";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                                "nextState=%s,timeout=%f,waitForAll=%s",
                                nextState.toString(), timeout, Boolean.toString(waitForAllEvents));
        }

        this.nextState = nextState;
        this.expiredTime = timeout;
        if (timeout > 0.0)
        {
            this.expiredTime += TrcUtil.getCurrentTime();
        }
        this.waitForAllEvents = waitForAllEvents;
        ready = false;
        clearAllEvents();

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //waitForEvents

    /**
     * This method puts the state machine into not ready mode and starts monitoring the events in the list. If
     * any event on the list is signaled, the state machine will be put back to ready mode and it will automatically
     * advance to the given next state. If timeout is non-zero, the state machine will be put back to ready mode
     * after timeout has expired even though the required event(s) have not been signaled.
     *
     * @param nextState specifies the next state when the state machine becomes ready.
     * @param timeout specifies a timeout value. A zero value means there is no timeout.
     */
    public void waitForEvents(T nextState, double timeout)
    {
        waitForEvents(nextState, timeout, false);
    }   //waitForEvents

    /**
     * This method puts the state machine into not ready mode and starts monitoring the events in the list. If any
     * event on the list is signaled, the state machine will be put back to ready mode and it will automatically
     * advance to the given next state.
     *
     * @param nextState specifies the next state when the state machine becomes ready.
     */
    public void waitForEvents(T nextState)
    {
        waitForEvents(nextState, 0.0, false);
    }   //waitForEvents

    /**
     * This method puts the state machine into not ready mode and starts monitoring a single event. If the event
     * is signaled, the state machine will be put back to ready mode and it will automatically advance to the given
     * next state. If timeout is non-zero, the state machine will be put back to ready mode after timeout has expired
     * even though the required event have not been signaled.
     *
     * @param event specifies the event to wait for.
     * @param nextState specifies the next state when the state machine becomes ready.
     * @param timeout specifies a timeout value. A zero value means there is no timeout.
     */
    public void waitForSingleEvent(TrcEvent event, T nextState, double timeout)
    {
        final String funcName =  "waitForSingleEvent";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                                "event=%s,nextState=%s,timeout=%f",
                                event.toString(), nextState.toString(), timeout);
        }

        eventList.clear();
        addEvent(event);
        waitForEvents(nextState, timeout, false);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //waitForSingleEvent

    /**
     * This method puts the state machine into not ready mode and starts monitoring a single event. If the event
     * is signaled, the state machine will be put back to ready mode and it will automatically advance to the given
     * next state. If timeout is non-zero, the state machine will be put back to ready mode after timeout has expired
     * even though the required event have not been signaled.
     *
     * @param event specifies the event to wait for.
     * @param nextState specifies the next state when the state machine becomes ready.
     */
    public void waitForSingleEvent(TrcEvent event, T nextState)
    {
        waitForSingleEvent(event, nextState, 0.0);
    }   //waitForSingleEvent

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
            event.clear();
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.UTIL);
        }
    }   //clearAllEvents

}   //class TrcStateMachine
