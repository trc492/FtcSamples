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
import java.util.Locale;
import java.util.Queue;

/**
 * This class implements an elapsed timer to record elapsed time. It is a performance monitoring tool. It is especially
 * important for PID controlled loops that the loops are executed at a high enough frequency or they will oscillate
 * wildly. This class records the elapsed time of some operations in an averaging window queue. To calculate the
 * average elapsed time, all elapsed time values recorded in the averaging window queue are averaged. It also records
 * the min and max elapsed time values it has seen since the last reset.
 */
public class TrcElapsedTimer
{
    private final String instanceName;
    private final long averageWindow;
    private final Queue<Long> elapsedTimeQueue;
    private long minElapsedTime;
    private long maxElapsedTime;
    private long totalElapsedTime;
    private long startTime;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the name to identify this instance of the timer.
     * @param averageWindow specifies the time window in seconds for averaging elapsed time.
     */
    public TrcElapsedTimer(final String instanceName, double averageWindow)
    {
        this.instanceName = instanceName;
        this.averageWindow = (long)(averageWindow * 1000000000);
        elapsedTimeQueue = new LinkedList<>();
        reset();
    }   //TrcElapsedTimer

    /**
     * This method returns the performance data in string form.
     *
     * @return performance data in string form.
     */
    @Override
    public String toString()
    {
        return String.format(Locale.US, "%s: avgElapsed=%.6f, minElapsed=%.6f, maxElapsed=%.6f",
                instanceName, getAverageElapsedTime(), getMaxElapsedTime(), getMaxElapsedTime());
    }   //toString

    /**
     * This method resets the performance data.
     */
    public synchronized void reset()
    {
        elapsedTimeQueue.clear();
        minElapsedTime = Long.MAX_VALUE;
        maxElapsedTime = 0L;
        totalElapsedTime = 0L;
        startTime = 0L;
    }   //reset

    /**
     * This method is called to record the start time.
     */
    public synchronized void recordStartTime()
    {
        startTime = TrcUtil.getCurrentTimeNanos();
    }   //recordStartTime

    /**
     * This method is called to record the elapsed time since the last start time into the elapsed time queue. It also
     * checks if the elapsed time is the minimum or maximum it has seen since last reset.
     */
    public synchronized void recordEndTime()
    {
        //
        // ElapsedTimer could have been enabled after the recordStartTime call, so it will miss recording startTime.
        // In this case, skip recordEndTime since we don't have a valid startTime.
        //
        if (startTime != 0L)
        {
            long elapsedTime = TrcUtil.getCurrentTimeNanos() - startTime;

            if (elapsedTime < minElapsedTime)
            {
                minElapsedTime = elapsedTime;
            } else if (elapsedTime > maxElapsedTime)
            {
                maxElapsedTime = elapsedTime;
            }

            elapsedTimeQueue.add(elapsedTime);
            totalElapsedTime += elapsedTime;

            while (totalElapsedTime > averageWindow)
            {
                totalElapsedTime -= elapsedTimeQueue.remove();
            }
        }
    }   //recordEndTime

    /**
     * This method is called in a loop to have its period time measured. It records a number of loop periods in a
     * queue for a given time window.
     */
    public synchronized void recordPeriodTime()
    {
        if (startTime == 0L)
        {
            recordStartTime();
        }
        else
        {
            recordEndTime();
            recordStartTime();
        }
    }   //recordPeriodTime

    /**
     * This method returns the last recorded elapsed time in seconds.
     *
     * @return last recorded elapsed time in seconds.
     */
    public synchronized double getLastElapsedTime()
    {
        double elapsedTime = 0.0;

        if (!elapsedTimeQueue.isEmpty())
        {
            elapsedTime = (long)((LinkedList)elapsedTimeQueue).getLast() / 1000000000.0;
        }

        return elapsedTime;
    }   //getLastElapsedTime

    /**
     * This method calculates the average elapsed time within the given time window.
     *
     * @return average elapsed time in seconds.
     */
    public synchronized double getAverageElapsedTime()
    {
        return elapsedTimeQueue.isEmpty()? 0.0: totalElapsedTime/elapsedTimeQueue.size()/1000000000.0;
    }   //getAverageElapsedTime

    /**
     * This method returns the minimum elapsed time since the last reset.
     *
     * @return minimum elapsed time in seconds.
     */
    public synchronized double getMinElapsedTime()
    {
        return minElapsedTime/1000000000.0;
    }   //getMinElapsedTime

    /**
     * This method returns the maximum elapsed time since the last reset.
     *
     * @return maximum elapsed time in seconds.
     */
    public synchronized double getMaxElapsedTime()
    {
        return maxElapsedTime/1000000000.0;
    }   //getMaxElapsedTime

    /**
     * This method prints the elapsed time info using the given tracer.
     *
     * @param tracer specifies the tracer to use for printing elapsed time info.
     */
    public synchronized void printElapsedTime(TrcDbgTrace tracer)
    {
        final String funcName = "printElapsedTime";

        tracer.traceInfo(funcName, "%s", this);
    }   //printElapsedTime

}   //class TrcElapsedTimer
