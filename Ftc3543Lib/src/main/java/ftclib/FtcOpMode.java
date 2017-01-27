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

package ftclib;

import android.speech.tts.TextToSpeech;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Locale;

import hallib.HalDashboard;
import trclib.TrcDbgTrace;
import trclib.TrcRobot;
import trclib.TrcTaskMgr;
import trclib.TrcUtil;

/**
 * This class implements a cooperative multi-tasking scheduler extending LinearOpMode.
 */
public abstract class FtcOpMode extends LinearOpMode implements TrcRobot.RobotMode
{
    private static final String moduleName = "FtcOpMode";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private static TrcDbgTrace globalTracer = null;
    private static String opModeName = null;
    private TextToSpeech textToSpeech = null;

    /**
     * This method is called to initialize the robot. In FTC, this is called when the "Init" button on the Driver
     * Station phone is pressed.
     */
    public abstract void initRobot();

    private final static String OPMODE_AUTO     = "FtcAuto";
    private final static String OPMODE_TELEOP   = "FtcTeleOp";
    private final static String OPMODE_TEST     = "FtcTest";

    private final static long LOOP_PERIOD = 20;
    private static FtcOpMode instance = null;
    private static double opModeStartTime = 0.0;
    private static double opModeElapsedTime = 0.0;
    private static double loopStartTime = 0.0;

    private TrcTaskMgr taskMgr;

    /**
     * Constructor: Creates an instance of the object. It calls the constructor of the LinearOpMode class and saves
     * an instance of this class.
     */
    public FtcOpMode()
    {
        super();

        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName, tracingEnabled, traceLevel, msgLevel);
        }

        instance = this;
        //
        // Create task manager. There is only one global instance of task manager.
        //
        taskMgr = new TrcTaskMgr();
    }   //FtcOpMode

    /**
     * This method returns the saved instance. This is a static method. So other class can get to this class instance
     * by calling getInstance(). This is very useful for other classes that need to access the public fields such as
     * hardwareMap, gamepad1 and gamepad2.
     *
     * @return save instance of this class.
     */
    public static FtcOpMode getInstance()
    {
        if (instance == null) throw new NullPointerException("You are not using FtcOpMode!");
        return instance;
    }   //getInstance

    /**
     * This method returns a global debug trace object for tracing OpMode code. If it doesn't exist yet, one is
     * created. This is an easy way to quickly get some debug output without a whole lot of setup overhead as the
     * full module-based debug tracing.
     *
     * @return global opMode trace object.
     */
    public static TrcDbgTrace getGlobalTracer()
    {
        if (globalTracer == null)
        {
            globalTracer = new TrcDbgTrace(opModeName != null? opModeName: "globalTracer", false,
                                           TrcDbgTrace.TraceLevel.API, TrcDbgTrace.MsgLevel.INFO);
        }

        return globalTracer;
    }   //getGlobalTracer

    /**
     * This method returns a TextToSpeech object. If it doesn't exist yet, one is created.
     *
     * @param locale specifies the language locale.
     * @return TextToSpeech object.
     */
    public TextToSpeech getTextToSpeech(final Locale locale)
    {
        if (textToSpeech == null)
        {
            textToSpeech = new TextToSpeech(hardwareMap.appContext,
                                            new TextToSpeech.OnInitListener()
                                            {
                                                @Override
                                                public void onInit(int status)
                                                {
                                                    if (status != TextToSpeech.ERROR)
                                                    {
                                                        textToSpeech.setLanguage(locale);
                                                    }
                                                }
                                            });
        }

        return textToSpeech;
    }   //getTextToSpeech

    /**
     * This method returns a TextToSpeech object with US locale.
     *
     * @return TextToSpeech object.
     */
    public TextToSpeech getTextToSpeech()
    {
        return getTextToSpeech(Locale.US);
    }   //getTextToSpeech

    /**
     * This method sets the global tracer configuration. The OpMode trace object was created with default
     * configuration of disabled method tracing, method tracing level is set to API and message trace level
     * set to INFO. Call this method if you want to change the configuration.
     *
     * @param traceEnabled specifies true if enabling method tracing.
     * @param traceLevel specifies the method tracing level.
     * @param msgLevel specifies the message tracing level.
     */
    public static void setGlobalTracerConfig(
            boolean traceEnabled, TrcDbgTrace.TraceLevel traceLevel, TrcDbgTrace.MsgLevel msgLevel)
    {
        globalTracer.setDbgTraceConfig(traceEnabled, traceLevel, msgLevel);
    }   //setGlobalTracerConfig

    /**
     * This method returns the name of the active OpMode.
     *
     * @return active OpMode name.
     */
    public static String getOpModeName()
    {
        return opModeName;
    }   //getOpModeName

    /**
     * This method returns the elapsed time since competition starts. This is the elapsed time after robotInit() is
     * called and after waitForStart() has returned (i.e. The "Play" button is pressed on the Driver Station.
     *
     * @return OpMode elapsed time in seconds.
     */
    public static double getOpModeElapsedTime()
    {
        opModeElapsedTime = TrcUtil.getCurrentTime() - opModeStartTime;
        return opModeElapsedTime;
    }   //getElapsedTime

    /**
     * This method returns the start time of the time slice loop. This is useful for the caller to determine if it
     * is in the same time slice as a previous operation for optimization purposes.
     *
     * @return time slice loop start time.
     */
    public static double getLoopStartTime()
    {
        return loopStartTime;
    }   //getElapsedTime

    //
    // Implements LinearOpMode
    //

    /**
     * This method is called when our OpMode is loaded and the "Init" button on the Driver Station is pressed.
     */
    @Override
    public void runOpMode()
    {
        final String funcName = "runOpMode";
        HalDashboard dashboard = HalDashboard.createInstance(telemetry);
        TrcRobot.RunMode runMode;

        if (debugEnabled)
        {
            if (dbgTrace == null)
            {
                dbgTrace = new TrcDbgTrace(moduleName, false, TrcDbgTrace.TraceLevel.API, TrcDbgTrace.MsgLevel.INFO);
            }
        }

        //
        // Determine run mode. Note that it means the OpMode must have "FtcAuto", "FtcTeleOp" or "FtcTest" in its name.
        //
        String opModeFullName = this.toString();
        opModeName = "Invalid";

        if (debugEnabled)
        {
            dbgTrace.traceInfo(funcName, "opModeFullName=<%s>", opModeFullName);
        }

        if (opModeFullName.contains(OPMODE_AUTO))
        {
            runMode = TrcRobot.RunMode.AUTO_MODE;
            opModeName = "Auto";
        }
        else if (opModeFullName.contains(OPMODE_TELEOP))
        {
            runMode = TrcRobot.RunMode.TELEOP_MODE;
            opModeName = "TeleOp";
        }
        else if (opModeFullName.contains(OPMODE_TEST))
        {
            runMode = TrcRobot.RunMode.TEST_MODE;
            opModeName = "Test";
        }
        else
        {
            throw new IllegalStateException("Invalid OpMode (must be either FtcAuto, FtcTeleOp or FtcTest.");
        }

        if (debugEnabled)
        {
            dbgTrace.traceInfo(funcName, "runMode=%s", runMode.toString());
        }

        //
        // robotInit contains code to initialize the robot.
        //
        if (debugEnabled)
        {
            dbgTrace.traceInfo(funcName, "Running robotInit ...");
        }
        dashboard.displayPrintf(0, "initRobot starting...");
        initRobot();
        dashboard.displayPrintf(0, "initRobot completed!");

        //
        // Run initPeriodic while waiting for competition to start.
        //
        if (debugEnabled)
        {
            dbgTrace.traceInfo(funcName, "Running initPeriodic ...");
        }
        dashboard.displayPrintf(0, "initPeriodic starting...");
        while (!opModeIsActive())
        {
            initPeriodic();
        }
        dashboard.displayPrintf(0, "initPeriodic completed!");
        opModeStartTime = TrcUtil.getCurrentTime();

        //
        // Prepare for starting the run mode.
        //
        if (debugEnabled)
        {
            dbgTrace.traceInfo(funcName, "Running Start Mode Tasks ...");
        }
        taskMgr.executeTaskType(TrcTaskMgr.TaskType.START_TASK, runMode);

        if (debugEnabled)
        {
            dbgTrace.traceInfo(funcName, "Running startMode ...");
        }
        startMode();

        long nextPeriodTime = TrcUtil.getCurrentTimeMillis();
        while (opModeIsActive())
        {
            loopStartTime = TrcUtil.getCurrentTime();
            opModeElapsedTime = loopStartTime - opModeStartTime;

            if (debugEnabled)
            {
                dbgTrace.traceInfo(funcName, "Running PreContinuous Tasks ...");
            }
            taskMgr.executeTaskType(TrcTaskMgr.TaskType.PRECONTINUOUS_TASK, runMode);

            if (debugEnabled)
            {
                dbgTrace.traceInfo(funcName, "Running runContinuous ...");
            }
            runContinuous(opModeElapsedTime);

            if (debugEnabled)
            {
                dbgTrace.traceInfo(funcName, "Running PostContinuous Tasks ...");
            }
            taskMgr.executeTaskType(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK, runMode);

            if (TrcUtil.getCurrentTimeMillis() >= nextPeriodTime)
            {
                dashboard.displayPrintf(0, "%s: %.3f", opModeName, opModeElapsedTime);
                nextPeriodTime += LOOP_PERIOD;

                if (debugEnabled)
                {
                    dbgTrace.traceInfo(funcName, "Running PrePeriodic Tasks ...");
                }
                taskMgr.executeTaskType(TrcTaskMgr.TaskType.PREPERIODIC_TASK, runMode);

                if (debugEnabled)
                {
                    dbgTrace.traceInfo(funcName, "Running runPeriodic ...");
                }
                runPeriodic(opModeElapsedTime);

                if (debugEnabled)
                {
                    dbgTrace.traceInfo(funcName, "Running PostPeriodic Tasks ...");
                }

                taskMgr.executeTaskType(TrcTaskMgr.TaskType.POSTPERIODIC_TASK, runMode);
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceInfo(funcName, "Running stopMode ...");
        }
        stopMode();

        if (debugEnabled)
        {
            dbgTrace.traceInfo(funcName, "Running Stop Mode Tasks ...");
        }
        taskMgr.executeTaskType(TrcTaskMgr.TaskType.STOP_TASK, runMode);
    }   //runOpMode

    /**
     * This method is called periodically after initRobot() is called but before competition starts. Typically,
     * you override this method and put code that will check and display robot status in this method. For example,
     * one may monitor the gyro heading in this method to make sure there is no major gyro drift before competition
     * starts. By default, this method is doing exactly what waitForStart() does.
     */
    public synchronized void initPeriodic()
    {
        try
        {
            this.wait();
        }
        catch (InterruptedException e)
        {
            Thread.currentThread().interrupt();
        }
    }   //initPeriodic

    /**
     * This method is called when the competition mode is about to start. In FTC, this is called when the "Play"
     * button on the Driver Station phone is pressed. Typically, you put code that will prepare the robot for start
     * of competition here such as resetting the encoders/sensors and enabling some sensors to start sampling.
     */
    @Override
    public void startMode()
    {
    }   //startMode

    /**
     * This method is called when competition mode is about to end. Typically, you put code that will do clean up
     * here such as disabling the sampling of some sensors.
     */
    @Override
    public void stopMode()
    {
    }   //stopMode

    /**
     * This method is called periodically about 50 times a second. Typically, you put code that doesn't require
     * frequent update here. For example, TeleOp joystick code can be put here since human responses are considered
     * slow.
     *
     * @param elapsedTime specifies the elapsed time since the mode started.
     */
    @Override
    public void runPeriodic(double elapsedTime)
    {
    }   //runPeriodic

    /**
     * This method is called periodically as fast as the control system allows. Typically, you put code that requires
     * servicing at a higher frequency here. To make the robot as responsive and as accurate as possible especially
     * in autonomous mode, you will typically put that code here.
     *
     * @param elapsedTime specifies the elapsed time since the mode started.
     */
    @Override
    public void runContinuous(double elapsedTime)
    {
    }   //runContinuous

}   //class FtcOpMode
