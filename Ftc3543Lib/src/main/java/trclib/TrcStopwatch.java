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

/**
 * This class implements a stopwatch.
 */
public class TrcStopwatch
{
    private double startTime = 0.0;
    private double lastElapsedTime = 0.0;

    /**
     * This method starts the stopwatch.
     */
    public synchronized void start()
    {
        lastElapsedTime = 0.0;
        startTime = TrcUtil.getCurrentTime();
    }   //start

    /**
     * This method stops the stopwatch.
     */
    public synchronized void stop()
    {
        if (isRunning())
        {
            lastElapsedTime = TrcUtil.getCurrentTime() - startTime;
            startTime = 0.0;
        }
    }   //stop

    /**
     * This method checks if the stopwatch is running.
     *
     * @return true if the stopwatch is running, false otherwise.
     */
    public synchronized boolean isRunning()
    {
        return startTime != 0.0;
    }   //isRunning

    /**
     * This method returns the elapsed time since the start time. If the stopwatch is not running, it returns the
     * last elapsed time.
     *
     * @return elapsed time since start, -1 if the stopwatch wasn't started.
     */
    public synchronized double getElapsedTime()
    {
        return isRunning() ? TrcUtil.getCurrentTime() - startTime : lastElapsedTime;
    }   //getElapsedTime;

}   //class TrcStopwatch
