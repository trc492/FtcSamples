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

package trclib;

/**
 * This class implements the Spurious Data filter. It is used for detecting and discarding bogus sensor data.
 * When spurious data is detected, it is discarded and the previous data is returned.
 */
public class TrcSpuriousFilter extends TrcFilter
{
    private final String instanceName;
    private final double distanceThreshold;
    private TrcDbgTrace tracer;
    private Double prevData;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param distanceThreshold specifies the distance threshold from previous data point to be considered spurious.
     * @param tracer specifies the optional tracer to be used to log an entry if spurious data is detected.
     *               Can be null if none provided.
     */
    public TrcSpuriousFilter(final String instanceName, double distanceThreshold, TrcDbgTrace tracer)
    {
        super(instanceName);

        this.instanceName = instanceName;
        this.distanceThreshold = distanceThreshold;
        this.tracer = tracer;
        prevData = null;
    }   //TrcSpuriousFilter

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param distanceThreshold specifies the distance threshold from previous data point to be considered spurious.
     */
    public TrcSpuriousFilter(final String instanceName, double distanceThreshold)
    {
        this(instanceName, distanceThreshold, null);
    }   //TrcSpuriousFilter

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

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "data=%f", data);
        }

        if (prevData != null && Math.abs(data - prevData) >= distanceThreshold)
        {
            if (tracer != null)
            {
                tracer.traceWarn(instanceName, "Spurious data detected (data=%f, prev=%f)", data, prevData);
            }
            data = prevData;
        }
        else
        {
            prevData = data;
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "=%f", data);
        }

        return data;
    }   //filterData

}   //class TrcSpuriousFilter
