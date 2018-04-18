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
 * This class implements a Boolean state object. As the name implies, this object can have a state of either true
 * or false. By default, the object is created with a false state but an overloaded constructor can specify the
 * state when it's created.
 */
public class TrcBooleanState
{
    private static final String moduleName = "TrcBooleanState";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private boolean state;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name of the object.
     * @param state specifies the initial state of the object.
     */
    public TrcBooleanState(final String instanceName, boolean state)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.state = state;
    }   //TrcBooleanState

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name of the object.
     */
    public TrcBooleanState(final String instanceName)
    {
        this(instanceName, false);
    }   //TrcBooleanState

    /**
     * This method returns the state of the object.
     *
     * @return state of the object.
     */
    public synchronized boolean getState()
    {
        final String funcName = "getState";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Boolean.toString(state));
        }

        return state;
    }   //getState

    /**
     * This method sets the state of the object.
     *
     * @param state specifies the state to set the object to.
     */
    public synchronized void setState(boolean state)
    {
        final String funcName = "setState";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "state=%s", Boolean.toString(state));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.state = state;
    }   //setState

    /**
     * This method toggles the state of the object and returns the new state of the object.
     *
     * @return new state of the object.
     */
    public synchronized boolean toggleState()
    {
        final String funcName = "toggleState";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        state = !state;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Boolean.toString(state));
        }
        return state;
    }   //toggleState

}   //class TrcBooleanState
