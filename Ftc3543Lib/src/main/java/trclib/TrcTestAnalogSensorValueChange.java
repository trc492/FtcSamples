/*
 * Copyright (c) 2018 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
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
 * This class implements a diagnostics test to monitor if an analog sensor value has changed. If a sensor value
 * doesn't change, it could mean the sensor is malfunctioning or disconnected.
 * This class extends the TrcDiagnostics.Test class. It provides the runTest method that reads the sensor value
 * and compares to the previous sensor value to make sure the value has changed at least a specified minimum amount.
 *
 * @param <T> specifies the group enum type.
 */
public class TrcTestAnalogSensorValueChange<T> extends TrcDiagnostics.Test<T>
{
    private final TrcUtil.DataSupplier<Double> sensor;
    private final double minValueChange;
    private final String errorMsg;
    private final Boolean stickyStatus;
    private double prevValue;
    private Boolean prevStatus = null;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param name specifies the test name.
     * @param group specifies the test group.
     * @param conditional specifies the conditional method that determines whether the test will be run,
     *                    null if none specified in which case, the test will always run.
     * @param defStatus specifies the default test status to be returned if the test is not run because
     *                  conditional was false.
     * @param sensor specifies the analog sensor object.
     * @param minValueChange specifies the minimum value change to expect.
     * @param errorMsg specifies the error message to return if sensor value didn't change the specified amount.
     * @param stickyStatus specifies null if test status is not sticky (i.e. test status could be different every
     *                     time runTest is called), otherwise stickyStatus specifies whether the test is true sticky
     *                     or false sticky (i.e. true sticky: one success will make the test always return success).
     */
    public TrcTestAnalogSensorValueChange(
            String name, T group, TrcUtil.DataSupplier<Boolean> conditional, boolean defStatus,
            TrcUtil.DataSupplier<Double> sensor, double minValueChange, String errorMsg, Boolean stickyStatus)
    {
        super(name, group, conditional, defStatus);
        this.sensor = sensor;
        this.minValueChange = minValueChange;
        this.errorMsg = errorMsg;
        this.stickyStatus = stickyStatus;
        this.prevValue = sensor.get();
    }   //TrcTestAnalogSensorValueChange

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param name specifies the test name.
     * @param group specifies the test group.
     * @param sensor specifies the analog sensor object.
     * @param minValueChange specifies the minimum value change to expect.
     * @param errorMsg specifies the error message to return if sensor value didn't change the specified amount.
     * @param stickyStatus specifies null if test status is not sticky (i.e. test status could be different every
     *                     time runTest is called), otherwise stickyStatus specifies whether the test is true sticky
     *                     or false sticky (i.e. true sticky: one success will make the test always return success).
     */
    public TrcTestAnalogSensorValueChange(
            String name, T group, TrcUtil.DataSupplier<Double> sensor, double minValueChange, String errorMsg,
            Boolean stickyStatus)
    {
        this(name, group, null, false, sensor, minValueChange, errorMsg, stickyStatus);
    }   //TrcTestAnalogSensorValueChange

    /**
     * This method is called periodically to check the sensor value. It compares to the previous value for a
     * change of at least the specified minimum amount.
     * Note: If stickyStatus is specified, a one-time success or failure will make this test always that stick status.
     *
     * @return error message if the change amount is less than the expected amount, null otherwise.
     */
    @Override
    public String runTest()
    {
        final String funcName = "runTest";
        String msg = null;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.CALLBK);
            dbgTrace.traceInfo(funcName, "sticky=%b,prev=%b", stickyStatus, prevStatus);
        }

        if (stickyStatus == null || prevStatus == null || prevStatus != stickyStatus)
        {
            double currValue = sensor.get();
            //
            // Only run the test if either:
            // - no sticky status is specified
            // - a sticky status is specified but there is no previous status (i.e. first time the test is run)
            // - a sticky status is specified and there is a previous status and it is not the same as the sticky status
            //
            if (Math.abs(currValue - prevValue) < minValueChange)
            {
                msg = errorMsg;
            }
            prevStatus = msg == null;
            prevValue = currValue;
        }
        else if (!stickyStatus)
        {
            //
            // A stickyStatus is specified and prevStatus is the same as stickyStatus and prevStatus is false
            // (i.e. test previously failed), return the error message.
            //
            msg = errorMsg;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.CALLBK, "=%s", msg);
        }

        return msg;
    }   //runTest

}   //class TrcTestAnalogSensorValueChange
