/*
 * Titan Robotics Framework Library
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.net)
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import hallib.HalDashboard;
import hallib.HalUtil;
import trclib.TrcDbgTrace;
import trclib.TrcRobot;
import trclib.TrcTaskMgr;

/**
 * This class implements a cooperative multi-tasking scheduler
 * extending LinearOpMode.
 */
public abstract class FtcOpMode extends LinearOpMode implements TrcRobot.RobotMode
{
    private static final String moduleName = "FtcOpMode";
    private static final boolean debugEnabled = false;
    private TrcDbgTrace dbgTrace = null;

    private static TrcDbgTrace opModeTracer = null;
    private static HalDashboard dashboard = null;
    private static String opModeName = null;

    /**
     * This method is called to initialize the robot. In FTC, this is called when the
     * "Init" button on the Driver Station phone is pressed.
     */
    public abstract void initRobot();

    private final static String OPMODE_AUTO     = "FtcAuto";
    private final static String OPMODE_TELEOP   = "FtcTeleOp";
    private final static String OPMODE_TEST     = "FtcTest";

    private final static long LOOP_PERIOD = 20;
    private TrcRobot.RunMode runMode = TrcRobot.RunMode.INVALID_MODE;
    private static FtcOpMode instance = null;
    private static double startTime = 0.0;
    private static double elapsedTime = 0.0;

    /**
     * Constructor: Creates an instance of the object. It calls the constructor
     * of the LinearOpMode class and saves an instance of this class.
     */
    public FtcOpMode()
    {
        super();
        instance = this;
        //
        // Create task manager. There is only one global instance of task manager.
        //
        TrcTaskMgr taskMgr = new TrcTaskMgr();
    }   //FtcOpMode

    /**
     * This method returns the saved instance. This is a static method. So other
     * class can get to this class instance by calling getInstance(). This is very
     * useful for other classes that need to access the public fields such as
     * hardwareMap, gamepad1 and gamepad2.
     *
     * @return save instance of this class.
     */
    public static FtcOpMode getInstance()
    {
        return instance;
    }   //getInstance

    /**
     * This method returns a global debug trace object for tracing OpMode code.
     * If it doesn't exist yet, one is created. This is an easy way to quickly
     * get some debug output without a whole lot of setup overhead as the full
     * module-based debug tracing.
     *
     * @return global opMode trace object.
     */
    public static TrcDbgTrace getOpModeTracer()
    {
        if (opModeTracer == null)
        {
            opModeTracer = new TrcDbgTrace(
                    opModeName != null? opModeName: "OpModeTracer",
                    false, TrcDbgTrace.TraceLevel.API, TrcDbgTrace.MsgLevel.INFO);
        }

        return opModeTracer;
    }   //getOpModeTracer

    /**
     * This method returns a global dashboard object for accessing the dashboard on the Driver Station.
     *
     * @return dashboard object.
     */
    public static HalDashboard getDashboard()
    {
        return dashboard;
    }   //getDashboard

    /**
     * This method sets the OpMode trace configuration. The OpMode trace object was
     * created with default configuration of disabled method tracing, method tracing
     * level is set to API and message trace level set to INFO. Call this method if
     * you want to change the configuration.
     *
     * @param traceEnabled specifies true if enabling method tracing.
     * @param traceLevel specifies the method tracing level.
     * @param msgLevel specifies the message tracing level.
     */
    public static void setOpModeTracerConfig(
            boolean traceEnabled, TrcDbgTrace.TraceLevel traceLevel, TrcDbgTrace.MsgLevel msgLevel)
    {
        opModeTracer.setDbgTraceConfig(traceEnabled, traceLevel, msgLevel);
    }   //setOpModeTracerConfig

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
     * This method returns the elapsed time since competition starts.
     * This is the elapsed time after robotInit() is called and after
     * waitForStart() has returned (i.e. The "Play" button is pressed
     * on the Driver Station.
     *
     * @return OpMode start elapsed time in seconds.
     */
    public static double getElapsedTime()
    {
        elapsedTime = HalUtil.getCurrentTime() - startTime;
        return elapsedTime;
    }   //getElapsedTime

    //
    // Implements LinearOpMode
    //

    /**
     * This method is called when our OpMode is loaded and the "Init" button
     * on the Driver Station is pressed.
     *
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException
    {
        final String funcName = "runOpMode";
        TrcTaskMgr taskMgr = TrcTaskMgr.getInstance();
        dashboard = new HalDashboard(telemetry);

        if (debugEnabled)
        {
            if (dbgTrace == null)
            {
                dbgTrace = new TrcDbgTrace(
                        moduleName, false, TrcDbgTrace.TraceLevel.API, TrcDbgTrace.MsgLevel.INFO);
            }
        }

        //
        // Determine run mode.
        // Note that it means the OpMode must have "FtcAuto", "FtcTeleOp" or "FtcTest"
        // in its name.
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
            throw new IllegalStateException(
                    "Invalid OpMode (must be either FtcAuto, FtcTeleOp or FtcTest.");
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
            dbgTrace.traceInfo(funcName, "Runing robotInit ...");
        }
        initRobot();

        //
        // Wait for the start of autonomous mode.
        //
        if (debugEnabled)
        {
            dbgTrace.traceInfo(funcName, "Waiting to start ...");
        }
        waitForStart();
        startTime = HalUtil.getCurrentTime();

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

        long nextPeriodTime = HalUtil.getCurrentTimeMillis();
        try
        {
            while (opModeIsActive())
            {
                elapsedTime = HalUtil.getCurrentTime() - startTime;

                if (debugEnabled)
                {
                    dbgTrace.traceInfo(funcName, "Runing PreContinuous Tasks ...");
                }
                taskMgr.executeTaskType(TrcTaskMgr.TaskType.PRECONTINUOUS_TASK, runMode);

                if (debugEnabled)
                {
                    dbgTrace.traceInfo(funcName, "Runing runContinuous ...");
                }
                runContinuous(elapsedTime);

                if (debugEnabled)
                {
                    dbgTrace.traceInfo(funcName, "Runing PostContinuous Tasks ...");
                }
                taskMgr.executeTaskType(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK, runMode);

                if (HalUtil.getCurrentTimeMillis() >= nextPeriodTime)
                {
                    dashboard.displayPrintf(0, "%s: %.3f", opModeName, elapsedTime);
                    nextPeriodTime += LOOP_PERIOD;

                    if (debugEnabled)
                    {
                        dbgTrace.traceInfo(funcName, "Runing PrePeriodic Tasks ...");
                    }
                    taskMgr.executeTaskType(TrcTaskMgr.TaskType.PREPERIODIC_TASK, runMode);

                    if (debugEnabled)
                    {
                        dbgTrace.traceInfo(funcName, "Runing runPeriodic ...");
                    }
                    runPeriodic(elapsedTime);

                    if (debugEnabled)
                    {
                        dbgTrace.traceInfo(funcName, "Runing PostPeriodic Tasks ...");
                    }

                    taskMgr.executeTaskType(TrcTaskMgr.TaskType.POSTPERIODIC_TASK, runMode);
                }
                idle();
            }
        }
        finally
        {
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
        }

    }   //runOpMode

    /**
     * This method is called when the competition mode is about to start. In FTC, this is
     * called when the "Play" button on the Driver Station phone is pressed. Typically,
     * you put code that will prepare the robot for start of competition here such as
     * resetting the encoders/sensors and enabling some sensors to start sampling.
     */
    @Override
    public void startMode()
    {
    }   //startMode

    /**
     * This method is called when competition mode is about to end. Typically, you put code
     * that will do clean up here such as disabling the sampling of some sensors.
     */
    @Override
    public void stopMode()
    {
    }   //stopMode

    /**
     * This method is called periodically about 50 times a second. Typically, you put code
     * that doesn't require frequent update here. For example, TeleOp joystick code can be
     * put here since human responses are considered slow.
     *
     * @param elapsedTime specifies the elapsed time since the mode started.
     */
    @Override
    public void runPeriodic(double elapsedTime)
    {
    }   //runPeriodic

    /**
     * This method is called periodically as fast as the control system allows. Typically,
     * you put code that requires servicing at a higher frequency here. To make the robot
     * as responsive and as accurate as possible especially in autonomous mode, you will
     * typically put that code here.
     *
     * @param elapsedTime specifies the elapsed time since the mode started.
     */
    @Override
    public void runContinuous(double elapsedTime)
    {
    }   //runContinuous

}   //class FtcOpMode
