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

import trclib.TrcTaskMgr.TaskType;

/**
 * This class implements a platform independent RGB light. Typically, this class is extended by a platform dependent
 * RGB light device that can be driven by some sort of control signals. The platform dependent RGB light device must
 * implement the abstract methods required by this class. The abstract methods allow this class to control each light
 * channel ON and OFF.
 */
public abstract class TrcRGBLight
{
    protected static final String moduleName = "TrcRGBLight";
    protected static final boolean debugEnabled = false;
    protected static final boolean tracingEnabled = false;
    protected static final boolean useGlobalTracer = false;
    protected static TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    protected static TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    protected TrcDbgTrace dbgTrace = null;

    /**
     * This method returns the state of the RED light.
     *
     * @return true if the RED light is ON, false otherwise.
     */
    public abstract boolean getRed();

    /**
     * This method returns the state of the GREEN light.
     *
     * @return true if the GREEN light is ON, false otherwise.
     */
    public abstract boolean getGreen();

    /**
     * This method returns the state of the BLUE light.
     *
     * @return true if the RED light is BLUE, false otherwise.
     */
    public abstract boolean getBlue();

    /**
     * This method sets the RED light ON or OFF.
     *
     * @param enabled specifies true to turn the RED light ON, false to turn it off.
     */
    public abstract void setRed(boolean enabled);

    /**
     * This method sets the GREEN light ON or OFF.
     *
     * @param enabled specifies true to turn the GREEN light ON, false to turn it off.
     */
    public abstract void setGreen(boolean enabled);

    /**
     * This method sets the BLUE light ON or OFF.
     *
     * @param enabled specifies true to turn the BLUE light ON, false to turn it off.
     */
    public abstract void setBlue(boolean enabled);

    public static final int COLOR_BLACK     = 0;
    public static final int COLOR_RED       = (1 << 0);
    public static final int COLOR_GREEN     = (1 << 1);
    public static final int COLOR_BLUE      = (1 << 2);
    public static final int COLOR_YELLOW    = (COLOR_RED | COLOR_GREEN);
    public static final int COLOR_MAGENTA   = (COLOR_RED | COLOR_BLUE);
    public static final int COLOR_CYAN      = (COLOR_GREEN | COLOR_BLUE);
    public static final int COLOR_WHITE     = (COLOR_RED | COLOR_GREEN | COLOR_BLUE);
    public static final int COLOR_MIN_VALUE = COLOR_BLACK;
    public static final int COLOR_MAX_VALUE = COLOR_WHITE;

    public enum RGBColor
    {
        RGB_INVALID(-1),
        RGB_BLACK(COLOR_BLACK),
        RGB_RED(COLOR_RED),
        RGB_GREEN(COLOR_GREEN),
        RGB_YELLOW(COLOR_YELLOW),
        RGB_BLUE(COLOR_BLUE),
        RGB_MAGENTA(COLOR_MAGENTA),
        RGB_CYAN(COLOR_CYAN),
        RGB_WHITE(COLOR_WHITE);

        private int value;

        /**
         * Constructor: Create an instance of the enum object with the given value.
         *
         * @param value specifies the ordinal value of the color.
         */
        RGBColor(int value)
        {
            this.value = value;
        }   //RGBColor

        /**
         * This method returns the ordinal value of the color.
         *
         * @return ordinal value of the color.
         */
        public int getValue()
        {
            return value;
        }   //getValue

        /**
         * This method returns the color enum with the specified ordinal value.
         *
         * @param value specifies the ordinal value of the color.
         * @return color enum with the specified ordinal value.
         */
        public static RGBColor getColor(int value)
        {
            for (RGBColor color: RGBColor.values())
            {
                if (value == color.getValue())
                {
                    return color;
                }
            }

            return RGB_INVALID;
        }   //getColor

    }   //enum RGBColor

    private enum State
    {
        TURN_ON,
        TURN_OFF,
        DONE
    }   //enum State

    private final String instanceName;
    private final TrcTaskMgr.TaskObject rgbLightTaskObj;
    private final TrcStateMachine<State> sm;
    private final TrcTimer timer;
    private final TrcEvent timerEvent;

    private RGBColor color = RGBColor.RGB_BLACK;
    private double onPeriod = 0.0;
    private double offPeriod = 0.0;
    private TrcEvent notifyEvent = null;
    private TrcNotifier.Receiver notifyReceiver = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public TrcRGBLight(final String instanceName)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
        rgbLightTaskObj = TrcTaskMgr.getInstance().createTask(instanceName + ".rgbLightTask", this::rgbLightTask);
        sm = new TrcStateMachine<>(moduleName);
        timer = new TrcTimer(moduleName);
        timerEvent = new TrcEvent(moduleName + ".timer");
    }   //TrcRGBLight

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
     * This method enables/disables the RGB light task that keeps track of the blinking period of the RGB light.
     *
     * @param enabled specifies true to enable RGB light task, false to disable.
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
            rgbLightTaskObj.registerTask(TaskType.POSTCONTINUOUS_TASK);     //TODO: should use OUTPUT_TASK
        }
        else
        {
            rgbLightTaskObj.unregisterTask(TaskType.POSTCONTINUOUS_TASK);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC);
        }
    }   //setTaskEnabled

    /**
     * This method returns the current color value of the light.
     *
     * @return current color value.
     */
    private int getColorValue()
    {
        final String funcName = "getColorValue";
        int colorValue = 0;

        if (getRed()) colorValue |= COLOR_RED;
        if (getGreen()) colorValue |= COLOR_GREEN;
        if (getBlue()) colorValue |= COLOR_BLUE;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=0xx", colorValue);
        }

        return colorValue;
    }   //getColorValue

    /**
     * This method returns the current color state of the light.
     *
     * @return current color state.
     */
    public RGBColor getColor()
    {
        final String funcName = "getColor";
        RGBColor color;
 
        color = RGBColor.getColor(getColorValue());

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", color.toString());
        }

        return color;
    }   //getColor

    /**
     * This method sets the RGB light with the specified color value. If the specified color value is zero, the light
     * is turned OFF.
     *
     * @param colorValue specifies the color value.
     */
    private void setColorValue(int colorValue)
    {
        final String funcName = "setColorValue";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "color=0x%x", colorValue);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        setRed((colorValue & COLOR_RED) != 0);
        setGreen((colorValue & COLOR_GREEN) != 0);
        setBlue((colorValue & COLOR_BLUE) != 0);
    }   //setColorValue

    /**
     * This method sets the RGB light with the specified color. If the specified color is BLACK, the light is turned
     * OFF.
     *
     * @param color specifies the color.
     */
    public synchronized void setColor(RGBColor color)
    {
        final String funcName = "setColor";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "color=%s", color.toString());
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (sm.isEnabled())
        {
            timer.cancel();
            sm.stop();
        }

        setColorValue(color.getValue());
    }   //setColor

    /**
     * This method blinks the RGB light with the specified color for the specified amount of ON time and OFF time.
     *
     * @param color specifies the color.
     * @param onPeriod specifies the period for the light to stay ON.
     * @param offPeriod specifies the period for the light to stay OFF.
     */
    public synchronized void setColor(RGBColor color, double onPeriod, double offPeriod)
    {
        final String funcName = "setColor";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "color=%s,onPeriod=%f,offPeriod=%f",
                    color.toString(), onPeriod, offPeriod);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        setColor(RGBColor.RGB_BLACK);
        this.color = color;
        this.onPeriod = onPeriod;
        this.offPeriod = offPeriod;
        this.notifyEvent = null;
        sm.start(State.TURN_ON);
        setTaskEnabled(true);
    }   //setColor

    /**
     * This method sets the RGB light ON with the specified color for the specified amount of time. When the amount
     * of time expired, the specified event is signaled.
     *
     * @param color specifies the color.
     * @param onPeriod specifies the period for the light to stay ON.
     * @param event specifies the event to signal when done.
     * @param receiver specifies the notification receiver to call when done.
     */
    public void setColor(RGBColor color, double onPeriod, TrcEvent event, TrcNotifier.Receiver receiver)
    {
        setColor(color, onPeriod, 0.0);
        this.notifyEvent = event;
        this.notifyReceiver = receiver;
    }   //setColor

    /**
     * This method sets the RGB light ON with the specified color for the specified amount of time. When the amount
     * of time expired, the specified event is signaled.
     *
     * @param color specifies the color.
     * @param onPeriod specifies the period for the light to stay ON.
     * @param event specifies the event to signal when done.
     */
    public void setColor(RGBColor color, double onPeriod, TrcEvent event)
    {
        setColor(color, onPeriod, 0.0);
        this.notifyEvent = event;
        this.notifyReceiver = null;
    }   //setColor

    /**
     * This method sets the RGB light ON with the specified color for the specified amount of time. When the amount
     * of time expired, the specified event is signaled.
     *
     * @param color specifies the color.
     * @param onPeriod specifies the period for the light to stay ON.
     * @param receiver specifies the notification receiver to call when done.
     */
    public void setColor(RGBColor color, double onPeriod, TrcNotifier.Receiver receiver)
    {
        setColor(color, onPeriod, 0.0);
        this.notifyEvent = null;
        this.notifyReceiver = receiver;
    }   //setColor

    /**
     * This method sets the RGB light ON with the specified color for the specified amount of time.
     * 
     * @param color specifies the color.
     * @param onPeriod specifies the period for the light to stay ON.
     */
    public void setColor(RGBColor color, double onPeriod)
    {
        setColor(color, onPeriod, 0.0);
    }   //setColor

    /**
     * This method is called periodically to execute the RGB light operation.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is running. (e.g. Autonomous, TeleOp, Test).
     */
    private synchronized void rgbLightTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = "rgbLightTask";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "taskType=%s,runMode=%s", taskType, runMode);
        }

        if (sm.isReady())
        {
            State state = sm.getState();

            switch (state)
            {
                case TURN_ON:
                    setColorValue(color.getValue());
                    if (onPeriod != 0.0)
                    {
                        timer.set(onPeriod, timerEvent);
                        sm.waitForSingleEvent(timerEvent, State.TURN_OFF);
                    }
                    else
                    {
                        sm.setState(State.DONE);
                    }
                    break;

                case TURN_OFF:
                    setColorValue(COLOR_BLACK);
                    if (offPeriod != 0.0)
                    {
                        timer.set(offPeriod, timerEvent);
                        sm.waitForSingleEvent(timerEvent, State.TURN_ON);
                    }
                    else
                    {
                        sm.setState(State.DONE);
                    }
                    break;

                case DONE:
                    if (notifyEvent != null)
                    {
                        notifyEvent.set(true);
                    }
                    if (notifyReceiver != null)
                    {
                        notifyReceiver.notify(this);
                    }
                    sm.stop();
                    setTaskEnabled(false);
                    break;
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //rgbLightTask

}   //class TrcRGBLight
