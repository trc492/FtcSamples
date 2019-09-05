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
 * This class implements the Infinite Impulse Response filter. It is useful for filtering noise from the sensor data.
 */
public class TrcIIRFilter extends TrcFilter
{
    private static final double DEF_WEIGHT = 0.9;

    private final String instanceName;
    private double weight;
    private double filteredData;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param weight specifies the weight of the current data point.
     */
    public TrcIIRFilter(final String instanceName, double weight)
    {
        super(instanceName);

        if (weight < 0.0 || weight > 1.0)
        {
            throw new IllegalArgumentException("Weight must be a positive fraction within 1.0.");
        }

        this.instanceName = instanceName;
        this.weight = weight;
        filteredData = 0.0;
    }   //TrcIIRFilter

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public TrcIIRFilter(final String instanceName)
    {
        this(instanceName, DEF_WEIGHT);
    }   //TrcIIRFilter

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

    //
    // Implements TrcFilter abstract methods.
    //

    /**
     * This method returns the filtered data.
     *
     * @param data specifies the data value to be filtered.
     * @return filtered data.
     */
    @Override
    public double filterData(double data)
    {
        final String funcName = "filterData";

        filteredData = filteredData*(1.0 - weight) + data*weight;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "data=%f", data);
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "=%f", filteredData);
        }

        return filteredData;
    }   //filterData

}   //class TrcIIRFilter
