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

package ftclib;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import trclib.TrcDbgTrace;
import trclib.TrcDigitalInput;

/**
 * This class implements a platform dependent digital input sensor extending TrcDigitalInput. It provides
 * implementation of the abstract methods in TrcDigitalInput.
 */
public class FtcDigitalInput extends TrcDigitalInput
{
    private static final String moduleName = "FtcDigitalInput";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private DigitalChannel digitalInput;
    private boolean inverted = false;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     */
    public FtcDigitalInput(HardwareMap hardwareMap, String instanceName)
    {
        super(instanceName);

        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        digitalInput = hardwareMap.get(DigitalChannel.class, instanceName);
        digitalInput.setMode(DigitalChannel.Mode.INPUT);
    }   //FtcDigitalInput

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FtcDigitalInput(String instanceName)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName);
    }   //FtcDigitalInput

    /**
     * This method inverts the digital input state. It is useful for changing a limit switch from Normal Open to
     * Normal Close, for example.
     *
     * @param inverted specifies true to invert the digital input, false otherwise.
     */
    public synchronized void setInverted(boolean inverted)
    {
        this.inverted = inverted;
    }   //setInverted

    //
    // Implements TrcDigitalInput abstract methods.
    //

    /**
     * This method returns the state of the digital input sensor.
     *
     * @return true if the digital input sensor is active, false otherwise.
     */
    @Override
    public synchronized boolean isActive()
    {
        final String funcName = "isActive";
        if (getInputElapsedTimer != null) getInputElapsedTimer.recordStartTime();
        boolean state = digitalInput.getState() ^ inverted;
        if (getInputElapsedTimer != null) getInputElapsedTimer.recordEndTime();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", state);
        }

        return state;
    }   //isActive

}   //class FtcDigitalInput
