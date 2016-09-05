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

public class TrcKalmanFilter extends TrcFilter
{
    private static final String moduleName = "TrcKalmanFilter";
    private static final boolean debugEnabled = false;
    private TrcDbgTrace dbgTrace = null;

    private double kQ;
    private double kR;
    private double prevP;
    private double prevXEst;
    private boolean initialized;

    public TrcKalmanFilter(String instanceName, double kQ, double kR)
    {
        super(instanceName);

        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(
                    moduleName,
                    false,
                    TrcDbgTrace.TraceLevel.API,
                    TrcDbgTrace.MsgLevel.INFO);
        }

        this.kQ = kQ;
        this.kR = kR;
        prevP = 0.0;
        prevXEst = 0.0;
        initialized = false;
    }   //TrcKalmanFilter

    public TrcKalmanFilter(String instanceName)
    {
        this(instanceName, 0.022, 0.617);
    }   //TrcKalmanFilter

    //
    // Implements TrcFilter abstract methods.
    //

    @Override
    public double filterData(double data)
    {
        final String funcName = "filter";

        if (!initialized)
        {
            prevXEst = data;
            initialized = true;
        }

        double tempP = prevP + kQ;
        double k = tempP/(tempP + kR);
        double xEst = prevXEst + k*(data - prevXEst);
        double p = (1 - k)*tempP;

        prevP = p;
        prevXEst = xEst;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "data=%f", data);
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "=%f", prevXEst);
        }

        return prevXEst;
    }   //filterData

}   //class TrcKalmanFilter
