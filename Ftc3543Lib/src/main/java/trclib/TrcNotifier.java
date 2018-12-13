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

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

/**
 * This class implements a global Notifier object. There should only be one global instance of TrcNotifier. Once
 * instantiated, it creates a thread pool with a fixed number of threads. If a component running on a time critical
 * thread wants to make a callback to another component and worries about the callback may take too long to return,
 * it can use one of the threads in TrcNotifier to do the callback thus freeing itself from waiting for the callback
 * to return.
 */
public class TrcNotifier
{
    private static final String moduleName = "TrcNotifier";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    /**
     * This interface provides a notification callback mechanism. It is used by a component that needs to wait for an
     * event to occur. There are two ways to wait for something to happen. One is to create a TrcEvent and pass it to
     * the component that will signal the event when the operation is completed. Another way is to make yourself
     * implementing the TrcNotifier.Receiver interface and pass yourself along to the component that will make a
     * callback to you when the operation is completed.
     */
    public interface Receiver
    {
        /**
         * This method is called to notify the occurrence of an event. In general, notification handler should do its
         * processing and return very quickly because the notification may be running on a time critical thread. For
         * example, if this is a timer notification, notify is called on the {@link TrcTimerMgr} thread. Any delay in
         * the notify call will prevent other timers from expiring on-time.
         *
         * @param context specifies the context of the event, can be null.
         */
        void notify(Object context);

    }   //interface Receiver

    /**
     * This class implements a notification handler that will be executed by a pool thread doing the the notification
     * callback.
     */
    private class Handler implements Runnable
    {
        private Receiver receiver;
        private final Object context;

        /**
         * Constructor: Create an instance of the handler.
         *
         * @param receiver specifies the notification receiver.
         * @param context specifies the context object to be passed back to the receiver.
         */
        public Handler(Receiver receiver, Object context)
        {
            this.receiver = receiver;
            this.context = context;
        }   //Handler

        /**
         * This method is run by the pool thread to do the notification callback.
         */
        public void run()
        {
            receiver.notify(context);
        }   //run

    }   //class Handler

    private static final int DEF_POOL_SIZE = 5;
    private static TrcNotifier instance = null;
    private ExecutorService threadPool;

    /**
     * Constructor: Create an instance of the object. There is only one global instance of TrcNotifier, so the
     * constructor is private and clients will call getInstance() to get the instance of TrcNotifier. If none
     * existed, the global TrcNotifier instance will be created.
     */
    private TrcNotifier()
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                    TrcDbgTrace.getGlobalTracer(): new TrcDbgTrace(moduleName, tracingEnabled, traceLevel, msgLevel);
        }

        threadPool = Executors.newFixedThreadPool(DEF_POOL_SIZE);
    }   //TrcNotifier

    /**
     * This method returns the global instance of the TrcNotifier. If none exist before, one will be created.
     *
     * @return global instance of TrcNotifier.
     */
    public static TrcNotifier getInstance()
    {
        if (instance == null)
        {
            instance = new TrcNotifier();
        }

        return instance;
    }   //getInstance

    /**
     * This method is called by a component that needs to do a notification callback using a pool thread.
     *
     * @param receiver specifies the notification callback recipient.
     * @param context specifies the context object to be passed back to the callback recipient.
     */
    public void sendNotification(Receiver receiver, Object context)
    {
        final String funcName = "sendNotification";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "receiver=%s,context=%s",
                    receiver, context);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        threadPool.execute(new Handler(receiver, context));
    }

    /**
     * This method initiates an orderly shutdown of the TrcNotifier in which previously submitted notification
     * callbacks are executed, but no new notification will be accepted.
     */
    public static void shutdown()
    {
        if (instance != null)
        {
            instance.threadPool.shutdown();
        }

        instance = null;
    }   //shutdown

    /**
     * This method attempts to stop all active notification callbacks and halts the processing of waiting callbacks.
     */
    public static void shutdownNow()
    {
        if (instance != null)
        {
            instance.threadPool.shutdownNow();
            instance.threadPool = null;
            instance = null;
        }
    }   //shutdownNow

    /**
     * This method initiates an orderly shutdown of the TrcNotifier and wait for its termination.
     */
    public static void shutdownAndAwaitTermination()
    {
        final String funcName = "shutdownAndAwaitTermination";

        if (instance != null)
        {
            //
            // Disable new tasks from being submitted.
            //
            instance.threadPool.shutdown();
            try
            {
                //
                // Wait a while for existing tasks to terminate.
                //
                if (!instance.threadPool.awaitTermination(60, TimeUnit.SECONDS))
                {
                    //
                    // Cancel currently executing tasks.
                    //
                    instance.threadPool.shutdownNow();
                    //
                    // Wait a while for tasks to respond to being cancelled.
                    //
                    if (!instance.threadPool.awaitTermination(60, TimeUnit.SECONDS))
                    {
                        TrcDbgTrace.getGlobalTracer().traceErr(funcName, "Thread pool failed to terminate.");
                    }
                }
            }
            catch (InterruptedException e)
            {
                //
                // (Re-)Cancel if current thread also interrupted.
                //
                instance.threadPool.shutdownNow();
                //
                // Preserve interrupt status.
                //
                Thread.currentThread().interrupt();
            }
            finally
            {
                instance.threadPool = null;
                instance = null;
            }
        }
    }   //shutdownAndAwaitTermination

}   //class TrcNotifier
