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
 * This class implements a diagnostics test to monitor if a digital sensor state has changed. If a sensor state
 * doesn't change, it could mean the sensor is malfunctioning or disconnected.
 * This class extends the TrcDiagnostics.Test class. It provides the runTest method that reads the sensor state
 * and compares to the previous sensor state to make sure the it has changed.
 *
 * @param <T> specifies the group enum type.
 */
public class TrcTestDigitalSensorStateChange<T> extends TrcTestAnalogSensorValueChange<T>
{
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
     * @param errorMsg specifies the error message to return if sensor state didn't change.
     * @param stickyStatus specifies null if test status is not sticky (i.e. test status could be different every
     *                     time runTest is called), otherwise stickyStatus specifies whether the test is true sticky
     *                     or false sticky (i.e. true sticky: one success will make the test always return success).
     */
    public TrcTestDigitalSensorStateChange(
            String name, T group, TrcUtil.DataSupplier<Boolean> conditional, boolean defStatus,
            TrcUtil.DataSupplier<Boolean> sensor, String errorMsg, Boolean stickyStatus)
    {
        super(name, group, conditional, defStatus, ()->sensor.get()? 1.0: 0.0, 1.0, errorMsg, stickyStatus);
    }   //TrcTestDigitalSensorStateChange

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param name specifies the test name.
     * @param group specifies the test group.
     * @param sensor specifies the analog sensor object.
     * @param errorMsg specifies the error message to return if sensor state didn't change.
     * @param stickyStatus specifies null if test status is not sticky (i.e. test status could be different every
     *                     time runTest is called), otherwise stickyStatus specifies whether the test is true sticky
     *                     or false sticky (i.e. true sticky: one success will make the test always return success).
     */
    public TrcTestDigitalSensorStateChange(
            String name, T group, TrcUtil.DataSupplier<Boolean> sensor, String errorMsg, Boolean stickyStatus)
    {
        this(name, group, null, false, sensor, errorMsg, stickyStatus);
    }   //TrcTestDigitalSensorStateChange

}   //class TrcTestDigitalSensorStateChange
