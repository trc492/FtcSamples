/*
 * Copyright (c) 2019 Titan Robotics Club (http://www.titanrobotics.com)
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

import java.util.LinkedList;
import java.util.Queue;

/**
 * This class implements a loop performance monitor to collect loop performance data. If a loop contains time critical
 * code, it is important to make sure the loop is executing with a frequency high enough for the required performance.
 * This class monitors the average loop time/frequency in a given time window.
 */
public class TrcLoopPerformanceMonitor
{
    private static final String moduleName = "TrcLoopPerformanceMonitor";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private final String instanceName;
    private double averageWindow;
    private Queue<Double> periodQueue;
    private Double lastTime;
    private double minPeriod;
    private double maxPeriod;
    private double timeSum;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the name to identify this instance of the timer.
     * @param averageWindow specifies the time window in seconds for averaging performance time.
     */
    public TrcLoopPerformanceMonitor(final String instanceName, double averageWindow)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                    TrcDbgTrace.getGlobalTracer():
                    new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
        this.averageWindow = averageWindow;
        periodQueue = new LinkedList<>();
        reset();
    }   //TrcLoopPerformanceMonitor

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

    /**
     * This method resets the performance data.
     */
    public void reset()
    {
        periodQueue.clear();
        lastTime = null;
        minPeriod = Double.POSITIVE_INFINITY;
        maxPeriod = 0.0;
        timeSum = 0.0;
    }   //reset

    /**
     * This method is called in the loop to have its performance monitored. It logs a number of loop periods in a
     * queue for a given time window.
     */
    public void update()
    {
        double currTime = TrcUtil.getCurrentTime();

        if (lastTime != null)
        {
            double deltaTime = currTime - lastTime;

            if (deltaTime < minPeriod)
            {
                minPeriod = deltaTime;
            }

            if (deltaTime > maxPeriod)
            {
                maxPeriod = deltaTime;
            }

            periodQueue.add(deltaTime);
            timeSum += deltaTime;

            while (timeSum > averageWindow)
            {
                timeSum -= periodQueue.remove();
            }
        }

        lastTime = currTime;
    }   //update

    /**
     * This method calculates the average loop period within the given time window.
     *
     * @return average loop period in seconds.
     */
    public double getAveragePeriod()
    {
        return periodQueue.isEmpty()? 0.0: timeSum/periodQueue.size();
    }   //getPeriod

    /**
     * This method returns the minimum loop period within the given time window.
     *
     * @return minimum loop period in seconds.
     */
    public double getMinPeriod()
    {
        return minPeriod;
    }   //getMinPeriod

    /**
     * This method returns the maximum loop period within the given time window.
     *
     * @return maximum loop period in seconds.
     */
    public double getMaxPeriod()
    {
        return maxPeriod;
    }   //getMaxPeriod

    /**
     * This method calculates the average loop frequency within the given time window.
     *
     * @return average loop frequency per second.
     */
    public double getAverageFrequency()
    {
        return timeSum == 0.0? 0.0: periodQueue.size()/timeSum;
    }   //getFrequency

    /**
     * This method returns the minimum loop frequency within the given time window.
     *
     * @return minimum loop frequency per second.
     */
    public double getMinFrequency()
    {
        return maxPeriod == 0.0? 0.0: 1.0/maxPeriod;
    }   //getMinFrequency

    /**
     * This method returns the maximum loop frequency within the given time window.
     *
     * @return maximum loop frequency per second.
     */
    public double getMaxFrequency()
    {
        return minPeriod == 0.0? 0.0: 1.0/minPeriod;
    }   //getMaxFrequency

}   //class TrcLoopPerformanceMonitor
