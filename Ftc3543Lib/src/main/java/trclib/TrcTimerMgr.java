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

import java.util.ArrayList;
import java.util.HashMap;

/**
 * This class implements the TrcTimer manager that uses a single thread to monitor all TrcTimers.
 */
public class TrcTimerMgr
{
    private static final String moduleName = "TrcTimerMgr";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private static TrcTimerMgr instance = null;
    private final ArrayList<TrcTimer> timerList;
    private final HashMap<TrcTimer, Double> securityKeyMap;
    private Thread timerThread = null;
    //
    // The following class variables need to be thread-safe protected.
    //
    private long nextExpireTimeInMsec = 0;
    private TrcTimer preemptingTimer = null;

    /**
     * Constructor: Creates an instance of the timer manager.
     */
    private TrcTimerMgr()
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName, tracingEnabled, traceLevel, msgLevel);
        }

        timerList = new ArrayList<>();
        securityKeyMap = new HashMap<>();
    }   //TrcTimerMgr

    /**
     * This method returns the instance of TrcTimerMgr. If this is the first time it's called, TrcTimerMgr is created.
     *
     * @return instance of TrcTimerMgr.
     */
    public static TrcTimerMgr getInstance()
    {
        if (instance == null)
        {
            instance = new TrcTimerMgr();
        }

        if (instance.timerThread == null)
        {
            instance.timerThread = new Thread(instance::timerTask, moduleName);
            instance.timerThread.start();
        }

        return instance;
    }   //getInstance

    /**
     * This method is called by the TrcTaskMgr to shut down TrcTimerMgr when it is exiting.
     */
    public static void shutdown()
    {
        if (instance != null)
        {
            instance.timerThread.interrupt();
        }
    }   //shutdown

    /**
     * This method adds the timer to the timer list in the order of expiration.
     *
     * @param timer specifies the timer to be added to the list.
     * @param callerID specifies the security token for identifying the caller.
     * @return securityKey that combines the caller's ID and a unique identifier assigned by TrcTimerMgr to the timer.
     */
    public double add(TrcTimer timer, double callerID)
    {
        final String funcName = "add";
        double securityKey = 0.0;
        long expiredTimeInMsec = timer.getExpiredTimeInMsec();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                    "timer=%s,callerID=%f", timer, callerID);
        }

        synchronized (timerList)
        {
            int position = -1;

            if (expiredTimeInMsec < nextExpireTimeInMsec)
            {
                if (debugEnabled)
                {
                    dbgTrace.traceInfo(funcName, "Adding preempting timer %s: currTime=%.3f, expiredTime=%.3f",
                            timer, TrcUtil.getCurrentTime(), expiredTimeInMsec/1000.0);
                }
                //
                // The added new timer expires sooner than the one we are sleeping on. Let's interrupt its sleep and
                // process this one first.
                //
                preemptingTimer = timer;
                timerThread.interrupt();
                //
                // The final security key is the sum of the list position and the caller's identification key.
                // Since this timer is not added to the list, the position will be -1.
                //
                securityKey = position + callerID;
                securityKeyMap.put(timer, securityKey);
            }
            else
            {
                for (int i = 0; i < timerList.size(); i++)
                {
                    if (expiredTimeInMsec < timerList.get(i).getExpiredTimeInMsec())
                    {
                        position = i;
                        break;
                    }
                }

                if (position == -1)
                {
                    //
                    // This must be the last in the list or the only one in the list, add it to the end.
                    //
                    position = timerList.size();
                }

                if (debugEnabled)
                {
                    dbgTrace.traceInfo(
                            funcName, "Adding timer %s to queue position %d: currTime=%.3f, expiredTime=%.3f",
                            timer, position, TrcUtil.getCurrentTime(), expiredTimeInMsec/1000.0);
                }
                timerList.add(position, timer);
                //
                // The final security key is the sum of the list position and the caller's identification key.
                //
                securityKey = position + callerID;
                securityKeyMap.put(timer, position + callerID);
                //
                // In case this is the first and only timer in the list, kick start the timer thread.
                //
                timerList.notify();
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", securityKey);
            dbgTrace.traceInfo("add", "Adding timer %s: securityKey=%.3f",
                    timer, securityKey);
        }

        return securityKey;
    }   //add

    /**
     * This method removes a timer from the list.
     *
     * @param timer specifies the timer to be removed.
     * @param securityKey specifies the security key identifying the owner of the timer to be removed.
     * @return true if the timer is removed, false otherwise.
     */
    public boolean remove(TrcTimer timer, double securityKey)
    {
        final String funcName = "remove";
        boolean success = false;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                    "timer=%s,securityKey=%f", timer, securityKey);
            dbgTrace.traceInfo(funcName, "Removing timer %s.", timer);
        }

        synchronized (timerList)
        {
            if (securityKey == securityKeyMap.get(timer))
            {
                success = timerList.remove(timer);
            }
            else
            {
                throw new SecurityException("Only the owner of the timer is allowed to remove it from the list.");
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", success);
            dbgTrace.traceInfo(funcName, "timer=%s,securityKey=%f,timerSecurity=%f",
                    timer, securityKey, securityKeyMap.get(timer));
        }

        return success;
    }   //remove

    /**
     * This method runs by the timer thread to wait for the next timer in the list and signal the timer object when
     * it expires.
     */
    private void timerTask()
    {
        final String funcName = "timerTask";

        if (debugEnabled)
        {
            dbgTrace.traceInfo("%s is starting...", moduleName);
        }

        while (!Thread.currentThread().isInterrupted())
        {
            TrcTimer nextTimerToExpire = null;

            try
            {
                long sleepTimeInMsec;

                synchronized (timerList)
                {
                    if (preemptingTimer == null && timerList.isEmpty())
                    {
                        //
                        // No more timer to process, let's wait.
                        //
                        if (debugEnabled)
                        {
                            dbgTrace.traceInfo(funcName, "Waiting for timer...");
                        }
                        timerList.wait();
                        if (debugEnabled)
                        {
                            dbgTrace.traceInfo(funcName, "Timer arrived.");
                        }
                    }
                    //
                    // Got a new incoming timer, either a new timer in the list or a preempting timer.
                    // Let's process it.
                    //
                    if (preemptingTimer != null)
                    {
                        //
                        // When a preempting timer is set, the exception handler below will put the next expiring
                        // timer back in the queue and we loop back up here.
                        //
                        nextTimerToExpire = preemptingTimer;
                        preemptingTimer = null;
                    }
                    else
                    {
                        nextTimerToExpire = timerList.remove(0);
                    }
                    nextExpireTimeInMsec = nextTimerToExpire.getExpiredTimeInMsec();
                    sleepTimeInMsec = nextExpireTimeInMsec - TrcUtil.getCurrentTimeMillis();
                    if (debugEnabled)
                    {
                        dbgTrace.traceInfo(funcName, "[%.3f]: timer=%s, sleepTime=%.3f",
                                TrcUtil.getCurrentTime(), nextTimerToExpire, sleepTimeInMsec/1000.0);
                    }
                }

                if (sleepTimeInMsec > 0)
                {
                    Thread.sleep(sleepTimeInMsec);
                }

                if (debugEnabled)
                {
                    dbgTrace.traceInfo(funcName, "[%.3f]: timer=%s expired.",
                            TrcUtil.getCurrentTime(), nextTimerToExpire);
                }
                synchronized (timerList)
                {
                    //
                    // Since we are no longer waiting on any timer now, there is nothing to preempt during an add.
                    // Next time around the loop we will get the earliest timer.
                    //
                    nextExpireTimeInMsec = 0;
                }
                //
                // Timer has expired, signal it.
                //
                nextTimerToExpire.setExpired(securityKeyMap.get(nextTimerToExpire));
            }
            catch (InterruptedException e)
            {
                synchronized (timerList)
                {
                    if (preemptingTimer != null)
                    {
                        //
                        // Somebody just added a timer that will expire sooner than the one we are sleeping on. Push
                        // this timer back to the front of the list and continue the next loop so the preempting timer
                        // will be processed first.
                        //
                        if (debugEnabled)
                        {
                            dbgTrace.traceInfo(funcName, "Timer %s is preempting %s.",
                                    preemptingTimer, nextTimerToExpire);
                        }
                        timerList.add(0, nextTimerToExpire);
                        nextExpireTimeInMsec = 0;
                    }
                    else
                    {
                        //
                        // Somebody is trying to terminate the timer thread. Let's quit.
                        //
                        if (debugEnabled)
                        {
                            dbgTrace.traceInfo(funcName, "Terminating %s", moduleName);
                        }
                        break;
                    }
                }
            }
        }
        //
        // The thread is terminating, cancel all pending timers before exiting.
        //
        synchronized (timerList)
        {
            if (debugEnabled)
            {
                dbgTrace.traceInfo(funcName, "Terminating: canceling %d timers", timerList.size());
            }

            for (TrcTimer timer: timerList)
            {
                if (debugEnabled)
                {
                    dbgTrace.traceInfo(funcName, "Canceling %s", timer);
                }
                timer.cancel();
            }

            timerList.clear();
        }

        if (debugEnabled)
        {
            dbgTrace.traceInfo(funcName, "%s is terminated", moduleName);
        }
        //
        // The thread is now terminated. Destroy this instance so we will recreate the thread the next time around.
        //
        timerThread = null;
    }   //timerTask

}   //class TrcTimerMgr
