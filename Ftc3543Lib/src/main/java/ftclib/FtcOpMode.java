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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.lang.annotation.Annotation;
import java.util.Locale;

import hallib.HalDashboard;
import trclib.TrcDbgTrace;
import trclib.TrcMotor;
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

    private static String opModeName = null;
    private TextToSpeech textToSpeech = null;

    /**
     * This method is called to initialize the robot. In FTC, this is called when the "Init" button on the Driver
     * Station phone is pressed.
     */
    public abstract void initRobot();

    protected final static int NUM_DASHBOARD_LINES = 16;
    private final static long LOOP_PERIOD_NANO = 50000000;
    private static FtcOpMode instance = null;
    private static long opModeStartNanoTime = 0;
    private static double opModeElapsedTime = 0.0;
    private static long loopStartNanoTime = 0;
    private static long loopCounter = 0;

    private TrcTaskMgr taskMgr;
    private long periodicTotalNanoTime = 0;
    private int periodicTimeSlotCount = 0;
    private long continuousTotalNanoTime = 0;
    private int continuousTimeSlotCount = 0;
    private long sdkTotalNanoTime = 0;

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
    }   //FtcOpMode

    /**
     * This method returns the saved instance. This is a static method. So other class can get to this class instance
     * by calling getInstance(). This is very useful for other classes that need to access the public fields and
     * methods.
     *
     * @return save instance of this class.
     */
    public static FtcOpMode getInstance()
    {
        if (instance == null) throw new NullPointerException("You are not using FtcOpMode!");
        return instance;
    }   //getInstance

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
        opModeElapsedTime = (TrcUtil.getCurrentTimeNanos() - opModeStartNanoTime)/1000000000.0;
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
        return loopStartNanoTime/1000000000.0;
    }   //getElapsedTime

    /**
     * This method returns the loop counter. This is very useful for code to determine if it is called multiple times
     * in the same loop. For example, it can be used to optimize sensor access so that if the sensor is accessed in
     * the same loop, there is no reason to create a new bus transaction to get "fresh" data from the sensor.
     *
     * @return loop counter value.
     */
    public static long getLoopCounter()
    {
        return loopCounter;
    }   //getLoopCounter

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
     * This method returns the annotation object of the specifies opmode type if it is present.
     *
     * @param opmodeType specifies the opmode type.
     * @return annotation object of the specified opmode type if present, null if not.
     */
    public Annotation getOpmodeAnnotation(Class opmodeType)
    {
        return getClass().getAnnotation(opmodeType);
    }   //getOpmodeAnnotation

    /**
     * This method returns the opmode type name.
     *
     * @param opmodeType specifies Autonomous.class for autonomous opmode and TeleOp.class for TeleOp opmode.
     * @return opmode type name.
     */
    public String getOpmodeTypeName(Class opmodeType)
    {
        String opmodeTypeName = null;

        Annotation annotation = getOpmodeAnnotation(opmodeType);
        if (annotation != null)
        {
            if (opmodeType == Autonomous.class)
            {
                opmodeTypeName = ((Autonomous)annotation).name();
            }
            else if (opmodeType == TeleOp.class)
            {
                opmodeTypeName = ((TeleOp)annotation).name();
            }
        }

        return opmodeTypeName;
    }   //getOpmodeTypeName

    /**
     * This method returns the opmode type group.
     *
     * @param opmodeType specifies Autonomous.class for autonomous opmode and TeleOp.class for TeleOp opmode.
     * @return opmode type group.
     */
    public String getOpmodeTypeGroup(Class opmodeType)
    {
        String opmodeTypeGroup = null;

        Annotation annotation = getOpmodeAnnotation(opmodeType);
        if (annotation != null)
        {
            if (opmodeType == Autonomous.class)
            {
                opmodeTypeGroup = ((Autonomous)annotation).group();
            }
            else if (opmodeType == TeleOp.class)
            {
                opmodeTypeGroup = ((TeleOp)annotation).group();
            }
        }

        return opmodeTypeGroup;
    }   //getOpmodeTypeGroup

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
        //
        // Create task manager if not already. There is only one global instance of task manager.
        //
        taskMgr = TrcTaskMgr.getInstance();
        //
        // Create dashboard here. If any earlier, telemetry may not exist yet.
        //
        HalDashboard dashboard = HalDashboard.createInstance(telemetry, NUM_DASHBOARD_LINES);
        TrcRobot.RunMode runMode;

        if (debugEnabled)
        {
            if (dbgTrace == null)
            {
                dbgTrace = new TrcDbgTrace(
                        moduleName, false, TrcDbgTrace.TraceLevel.API, TrcDbgTrace.MsgLevel.INFO);
            }
        }
        //
        // Determine run mode. Note that it means the OpMode must be annotated with group="FtcAuto", group="FtcTeleOp"
        // or group="FtcTest".
        //
        opModeName = getOpmodeTypeName(Autonomous.class);
        if (opModeName != null)
        {
            runMode = TrcRobot.RunMode.AUTO_MODE;
        }
        else
        {
            opModeName = getOpmodeTypeName(TeleOp.class);
            if (opModeName != null)
            {
                if (getOpmodeTypeGroup(TeleOp.class).startsWith("FtcTest"))
                {
                    runMode = TrcRobot.RunMode.TEST_MODE;
                }
                else
                {
                    runMode = TrcRobot.RunMode.TELEOP_MODE;
                }
            }
            else
            {
                throw new IllegalStateException(
                        "Invalid OpMode annotation, OpMode must be annotated with either @Autonomous or @TeleOp.");
            }
        }
        TrcRobot.setRunMode(runMode);

        if (TrcMotor.getNumOdometryMotors() > 0)
        {
            if (debugEnabled)
            {
                dbgTrace.traceWarn(funcName, "Odometry motors list is not empty (numMotors=%d)!",
                        TrcMotor.getNumOdometryMotors());
            }
            TrcMotor.clearOdometryMotorsList();
        }

        try
        {
            //
            // robotInit contains code to initialize the robot.
            //
            if (debugEnabled)
            {
                dbgTrace.traceInfo(funcName, "Current RunMode: %s", runMode);
                dbgTrace.traceInfo(funcName, "Running initRobot");
            }
            dashboard.displayPrintf(0, "initRobot starting...");
            initRobot();
            dashboard.displayPrintf(0, "initRobot completed!");

            //
            // Run initPeriodic while waiting for competition to start.
            //
            if (debugEnabled)
            {
                dbgTrace.traceInfo(funcName, "Running initPeriodic");
            }
            loopCounter = 0;
            dashboard.displayPrintf(0, "initPeriodic starting...");
            while (!isStarted())
            {
                loopCounter++;
                loopStartNanoTime = TrcUtil.getCurrentTimeNanos();
                if (debugEnabled)
                {
                    dbgTrace.traceInfo(funcName, "[%d:%.3f]: InitPeriodic loop",
                            loopCounter, loopStartNanoTime/1000000000.0);
                }
                initPeriodic();
            }
            dashboard.displayPrintf(0, "initPeriodic completed!");
            opModeStartNanoTime = TrcUtil.getCurrentTimeNanos();

            //
            // Prepare for starting the run mode.
            //
            if (debugEnabled)
            {
                dbgTrace.traceInfo(funcName, "Running Start Mode Tasks");
            }
            taskMgr.executeTaskType(TrcTaskMgr.TaskType.START_TASK, runMode);

            if (debugEnabled)
            {
                dbgTrace.traceInfo(funcName, "Running startMode");
            }
            startMode(null, runMode);

            long nextPeriodNanoTime = TrcUtil.getCurrentTimeNanos();
            long startNanoTime = TrcUtil.getCurrentTimeNanos();

            loopCounter = 0;
            while (opModeIsActive())
            {
                loopStartNanoTime = TrcUtil.getCurrentTimeNanos();
                loopCounter++;
                sdkTotalNanoTime += loopStartNanoTime - startNanoTime;
                opModeElapsedTime = (loopStartNanoTime - opModeStartNanoTime)/1000000000.0;

                if (debugEnabled)
                {
                    dbgTrace.traceInfo(funcName, "[%d:%.3f]: OpMode loop",
                            loopCounter, loopStartNanoTime/1000000000.0);
                    dbgTrace.traceInfo(funcName, "Running PreContinuous Tasks");
                }
                taskMgr.executeTaskType(TrcTaskMgr.TaskType.PRECONTINUOUS_TASK, runMode);

                if (debugEnabled)
                {
                    dbgTrace.traceInfo(funcName, "Running runContinuous");
                }
                startNanoTime = TrcUtil.getCurrentTimeNanos();
                runContinuous(opModeElapsedTime);
                continuousTotalNanoTime += TrcUtil.getCurrentTimeNanos() - startNanoTime;
                continuousTimeSlotCount++;

                if (debugEnabled)
                {
                    dbgTrace.traceInfo(funcName, "Running PostContinuous Tasks");
                }
                taskMgr.executeTaskType(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK, runMode);

                if (TrcUtil.getCurrentTimeNanos() >= nextPeriodNanoTime)
                {
                    dashboard.displayPrintf(0, "%s: %.3f", opModeName, opModeElapsedTime);
                    nextPeriodNanoTime += LOOP_PERIOD_NANO;

                    if (debugEnabled)
                    {
                        dbgTrace.traceInfo(funcName, "Running PrePeriodic Tasks");
                    }
                    taskMgr.executeTaskType(TrcTaskMgr.TaskType.PREPERIODIC_TASK, runMode);

                    if (debugEnabled)
                    {
                        dbgTrace.traceInfo(funcName, "Running runPeriodic");
                    }
                    startNanoTime = TrcUtil.getCurrentTimeNanos();
                    runPeriodic(opModeElapsedTime);
                    periodicTotalNanoTime += TrcUtil.getCurrentTimeNanos() - startNanoTime;
                    periodicTimeSlotCount++;

                    if (debugEnabled)
                    {
                        dbgTrace.traceInfo(funcName, "Running PostPeriodic Tasks");
                    }

                    taskMgr.executeTaskType(TrcTaskMgr.TaskType.POSTPERIODIC_TASK, runMode);
                }

                startNanoTime = TrcUtil.getCurrentTimeNanos();
            }

            if (debugEnabled)
            {
                dbgTrace.traceInfo(funcName, "Running stopMode");
            }
            stopMode(runMode, null);

            if (debugEnabled)
            {
                dbgTrace.traceInfo(funcName, "Running Stop Mode Tasks");
            }
            taskMgr.executeTaskType(TrcTaskMgr.TaskType.STOP_TASK, runMode);
        }
        finally
        {
            //
            // Make sure we properly clean up and shut down even if the code throws an exception but we are not
            // catching the exception and let it propagate up.
            //
            TrcMotor.clearOdometryMotorsList();
            taskMgr.shutdown();
        }
    }   //runOpMode

    /**
     * This method prints the performance metrics of all loops and taska.
     *
     * @param tracer specifies the tracer to be used for printing the performance metrics.
     */
    public void printPerformanceMetrics(TrcDbgTrace tracer)
    {
        tracer.traceInfo(
                moduleName,
                "%16s: Periodic=%.6f, Continuous=%.6f, SDK=%.6f",
                opModeName,
                (double)periodicTotalNanoTime/periodicTimeSlotCount/1000000000,
                (double)continuousTotalNanoTime/continuousTimeSlotCount/1000000000,
                (double)sdkTotalNanoTime/loopCounter/1000000000);
        taskMgr.printTaskPerformanceMetrics(tracer);
    }   //printPerformanceMetrics

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
     * button on the Driver Station phone is pressed. Typically, you put code that will prepare the robot for
     * start of competition here such as resetting the encoders/sensors and enabling some sensors to start
     * sampling.
     *
     * @param prevMode specifies the previous RunMode it is coming from (always null for FTC).
     * @param nextMode specifies the next RunMode it is going into.
     */
    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
    }   //startMode

    /**
     * This method is called when competition mode is about to end. Typically, you put code that will do clean
     * up here such as disabling the sampling of some sensors.
     *
     * @param prevMode specifies the previous RunMode it is coming from.
     * @param nextMode specifies the next RunMode it is going into (always null for FTC).
     */
    @Override
    public void stopMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
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
