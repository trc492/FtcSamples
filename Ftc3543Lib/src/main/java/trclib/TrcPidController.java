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

import hallib.HalDashboard;

import java.util.EmptyStackException;
import java.util.Locale;
import java.util.Stack;

/**
 * This class implements a PID controller. A PID controller takes a target set point and an input from a feedback
 * device to calculate the output power of an effector usually a motor or a set of motors.
 */
public class TrcPidController
{
    protected static final String moduleName = "TrcPidController";
    protected static final boolean debugEnabled = false;
    protected static final boolean tracingEnabled = false;
    protected static final boolean useGlobalTracer = false;
    protected static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    protected static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    protected TrcDbgTrace dbgTrace = null;

    /**
     * This class encapsulates all the PID coefficients into a single object and makes it more efficient to pass them
     * around.
     */
    public static class PidCoefficients
    {
        public double kP;
        public double kI;
        public double kD;
        public double kF;

        /**
         * Constructor: Create an instance of the object.
         *
         * @param kP specifies the Proportional constant.
         * @param kI specifies the Integral constant.
         * @param kD specifies the Differential constant.
         * @param kF specifies the Feed forward constant.
         */
        public PidCoefficients(double kP, double kI, double kD, double kF)
        {
            this.kP = Math.abs(kP);
            this.kI = Math.abs(kI);
            this.kD = Math.abs(kD);
            this.kF = Math.abs(kF);
        }   //PidCoefficients

        /**
         * Constructor: Create an instance of the object.
         */
        public PidCoefficients()
        {
            this(1.0, 0.0, 0.0, 0.0);
        }   //PidCoefficients

        /**
         * Constructor: Create an instance of the object.
         *
         * @param kP specifies the Proportional constant.
         * @param kI specifies the Integral constant.
         * @param kD specifies the Differential constant.
         */
        public PidCoefficients(double kP, double kI, double kD)
        {
            this(kP, kI, kD, 0.0);
        }   //PidCoefficients

        /**
         * Constructor: Create an instance of the object.
         *
         * @param kP specifies the Proportional constant.
         */
        public PidCoefficients(double kP)
        {
            this(kP, 0.0, 0.0, 0.0);
        }   //PidCoefficients

        /**
         * This method returns all PID coefficients in string form.
         *
         * @return PID coefficients string.
         */
        @Override
        public String toString()
        {
            return String.format("(%f,%f,%f,%f)", kP, kI, kD, kF);
        }   //toString

    }   //class PidCoefficients

    /**
     * PID controller needs input from a feedback device for calculating the output power. Whoever is providing this
     * input must implement this interface.
     */
    public interface PidInput
    {
        /**
         * This method is called by the PID controller to get input data from the feedback device. The feedback
         * device can be motor encoders, gyro, ultrasonic sensor, light sensor etc.
         *
         * @return input value of the feedback device.
         */
        double get();

    }   //interface PidInput

    public static final double DEF_SETTLING_TIME = 0.2;

    private HalDashboard dashboard;
    private String instanceName;
    private PidCoefficients pidCoefficients;
    private double tolerance;
    private double settlingTime;
    private PidInput pidInput;

    private boolean inverted = false;
    private boolean absSetPoint = false;
    private boolean noOscillation = false;
    private double minTarget = 0.0;
    private double maxTarget = 0.0;
    private double minOutput = -1.0;
    private double maxOutput = 1.0;
    private double outputLimit = 1.0;
    private Double rampRate = null;
    private Stack<Double> outputLimitStack = new Stack<>();

    private double prevTime = 0.0;
    private double currError = 0.0;
    private double totalError = 0.0;
    private double settlingStartTime = 0.0;
    private double setPoint = 0.0;
    private double setPointSign = 1.0;
    private double currInput = 0.0;
    private double output = 0.0;
    private double prevOutputTime = 0.0; // time that getOutput() was called last. Used for ramp rates.

    private TrcDbgTrace debugTracer = null;
    private double pTerm;
    private double iTerm;
    private double dTerm;
    private double fTerm;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName    specifies the instance name.
     * @param pidCoefficients specifies the PID constants.
     * @param tolerance       specifies the target tolerance.
     * @param settlingTime    specifies the minimum on target settling time.
     * @param pidInput        specifies the input provider.
     */
    public TrcPidController(final String instanceName, PidCoefficients pidCoefficients, double tolerance,
        double settlingTime, PidInput pidInput)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer ?
                TrcDbgTrace.getGlobalTracer() :
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        dashboard = HalDashboard.getInstance();
        this.instanceName = instanceName;
        this.pidCoefficients = pidCoefficients;
        this.tolerance = Math.abs(tolerance);
        this.settlingTime = Math.abs(settlingTime);
        this.pidInput = pidInput;
    }   //TrcPidController

    /**
     * Constructor: Create an instance of the object. This constructor is not public. It is only for classes
     * extending this class (e.g. Cascade PID Controller) that cannot make itself as an input provider in its
     * constructor (Java won't allow it). Instead, we provide another protected method setPidInput so it can
     * set the PidInput outside of the super() call.
     *
     * @param instanceName    specifies the instance name.
     * @param pidCoefficients specifies the PID constants.
     * @param tolerance       specifies the target tolerance.
     * @param pidInput        specifies the input provider.
     */
    public TrcPidController(final String instanceName, PidCoefficients pidCoefficients, double tolerance,
        PidInput pidInput)
    {
        this(instanceName, pidCoefficients, tolerance, DEF_SETTLING_TIME, pidInput);
    }   //TrcPidController

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
     * This method displays the PID information on the dashboard for debugging and tuning purpose. Note that the
     * PID info occupies two dashboard lines.
     *
     * @param lineNum specifies the starting line number of the dashboard to display the info.
     */
    public synchronized void displayPidInfo(int lineNum)
    {
        dashboard.displayPrintf(
                lineNum, "%s:Target=%.1f,Input=%.1f,Error=%.1f", instanceName, setPoint, currInput, currError);
        dashboard.displayPrintf(
                lineNum + 1, "minOutput=%.1f,Output=%.1f,maxOutput=%.1f", minOutput, output, maxOutput);
    }   //displayPidInfo

    /**
     * This method prints the PID information to the tracer console. If no tracer is provided, it will attempt to
     * use the debug tracer in this module but if the debug tracer is not enabled, no output will be produced.
     *
     * @param tracer    specifies the tracer object to print the PID info to.
     * @param timestamp specifies the timestamp to be printed.
     * @param verbose specifies true to print verbose info, false to print summary info.
     * @param battery   specifies the battery object to get battery info, can be null if not provided.
     */
    public synchronized void printPidInfo(TrcDbgTrace tracer, double timestamp, boolean verbose, TrcRobotBattery battery)
    {
        final String funcName = "printPidInfo";

        if (tracer == null)
        {
            tracer = dbgTrace;
        }

        if (tracer != null)
        {
            StringBuilder msg = new StringBuilder();

            if (timestamp != 0.0)
            {
                msg.append(String.format(Locale.US, "[%.3f] ", timestamp));
            }

            msg.append(String.format(
                    Locale.US, "%s: Target=%6.1f, Input=%6.1f, Error=%6.1f",
                    instanceName, setPoint, currInput, currError));

            if (verbose)
            {
                msg.append(String.format(
                        Locale.US, ", PIDTerms=%6.3f/%6.3f/%6.3f/%6.3f, Output=%6.3f(%6.3f/%5.3f)",
                        pTerm, iTerm, dTerm, fTerm, output, minOutput,
                        maxOutput));
            }

            if (battery != null)
            {
                msg.append(String.format(Locale.US, ", Volt=%.1f(%.1f)",
                        battery.getVoltage(), battery.getLowestVoltage()));
            }

            tracer.traceInfo(funcName, msg.toString());
        }
    }   //printPidInfo

    /**
     * This method prints the PID information to the tracer console. If no tracer is provided, it will attempt to
     * use the debug tracer in this module but if the debug tracer is not enabled, no output will be produced.
     *
     * @param tracer    specifies the tracer object to print the PID info to.
     * @param timestamp specifies the timestamp to be printed.
     * @param verbose specifies true to print verbose info, false to print summary info.
     */
    public void printPidInfo(TrcDbgTrace tracer, double timestamp, boolean verbose)
    {
        printPidInfo(tracer, timestamp, verbose, null);
    }   //printPidInfo

    /**
     * This method prints the PID information to the tracer console. If no tracer is provided, it will attempt to
     * use the debug tracer in this module but if the debug tracer is not enabled, no output will be produced.
     *
     * @param tracer    specifies the tracer object to print the PID info to.
     * @param timestamp specifies the timestamp to be printed.
     */
    public void printPidInfo(TrcDbgTrace tracer, double timestamp)
    {
        printPidInfo(tracer, timestamp, false, null);
    }   //printPidInfo

    /**
     * This method prints the PID information to the tracer console. If no tracer is provided, it will attempt to
     * use the debug tracer in this module but if the debug tracer is not enabled, no output will be produced.
     *
     * @param tracer specifies the tracer object to print the PID info to.
     */
    public void printPidInfo(TrcDbgTrace tracer)
    {
        printPidInfo(tracer, 0.0, false, null);
    }   //printPidInfo

    /**
     * This method prints the PID information to the default debug tracer.
     */
    public void printPidInfo()
    {
        printPidInfo(null, 0.0, false, null);
    }   //printPidInfo

    /**
     * This method allows the caller to dynamically enable/disable debug tracing of the output calculation. It is
     * very useful for debugging or tuning PID control.
     *
     * @param tracer  specifies the tracer to be used for debug tracing.
     * @param enabled specifies true to enable the debug tracer, false to disable.
     */
    public synchronized void setDebugTraceEnabled(TrcDbgTrace tracer, boolean enabled)
    {
        debugTracer = enabled ? tracer : null;
    }   //setDebugTraceEnabled

    /**
     * This method inverts the sign of the calculated error. Normally, the calculated error starts with a large
     * positive number and goes down. However, in some sensors such as the ultrasonic sensor, the target is a small
     * number and the error starts with a negative value and increases. In order to calculate a correct output which
     * will go towards the target, the error sign must be inverted.
     *
     * @param inverted specifies true to invert the sign of the calculated error, false otherwise.
     */
    public synchronized void setInverted(boolean inverted)
    {
        final String funcName = "setInverted";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "inverted=%s", Boolean.toString(inverted));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.inverted = inverted;
    }   //setInverted

    /**
     * This method sets the set point mode to be absolute. PID controller always calculates the output with an
     * absolute set point comparing to a sensor value representing an absolute input. But by default, it will
     * treat the set point as a value relative to its current input. So it will add the relative set point value
     * to the current input as the absolute set point in its calculation. This method allows the caller to treat
     * the set point as absolute set point.
     *
     * @param absolute specifies true if set point is absolute, false otherwise.
     */
    public synchronized void setAbsoluteSetPoint(boolean absolute)
    {
        final String funcName = "setAbsoluteSetPoint";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "absolute=%s", Boolean.toString(absolute));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.absSetPoint = absolute;
    }   //setAbsoluteSetPoint

    /**
     * This method returns true if setpoints are absolute, false otherwise.
     *
     * @return true if setpoints are absolute, false otherwise.
     */
    public synchronized boolean hasAbsoluteSetPoint()
    {
        final String funcName = "hasAbsoluteSetPoint";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%b", absSetPoint);
        }

        return absSetPoint;
    }   //hasAbsoluteSetPoint

    /**
     * This method enables/disables NoOscillation mode. In PID control, if the PID constants are not tuned quite
     * correctly, it may cause oscillation that could waste a lot of time. In some scenarios, passing the target
     * beyond the tolerance may be acceptable. This method allows the PID controller to declare "On Target" even
     * though it passes the target beyond tolerance so it doesn't oscillate.
     *
     * @param noOscillation specifies true to enable no oscillation, false to disable.
     */
    public synchronized void setNoOscillation(boolean noOscillation)
    {
        final String funcName = "setNoOscillation";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "noOsc=%s", Boolean.toString(noOscillation));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.noOscillation = noOscillation;
    }   //setNoOscillation

    /**
     * This method returns the current PID coefficients.
     *
     * @return current PID coefficients.
     */
    public synchronized PidCoefficients getPidCoefficients()
    {
        final String funcName = "getPidCoefficients";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=(%f,%f,%f,%f)", pidCoefficients.kP,
                pidCoefficients.kI, pidCoefficients.kD, pidCoefficients.kF);
        }

        return pidCoefficients;
    }   //getPidCoefficients

    /**
     * This method sets new PID coefficients.
     *
     * @param pidCoefficients specifies new PID coefficients.
     */
    public synchronized void setPidCoefficients(PidCoefficients pidCoefficients)
    {
        final String funcName = "setPidCoefficients";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "Kp=%f,Ki=%f,Kd=%f,Kf=%f", pidCoefficients.kP,
                pidCoefficients.kI, pidCoefficients.kD, pidCoefficients.kF);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.pidCoefficients = pidCoefficients;
    }   //setPidCoefficients

    /**
     * This method sets the ramp rate of the PID controller output. It is sometimes useful to limit the acceleration
     * of the output of the PID controller. For example, the strafing PID controller on a mecanum drive base may
     * benefit from a lower acceleration to minimize wheel slipperage.
     *
     * @param rampRate specifies the ramp rate in percent power per second.
     */
    public synchronized void setRampRate(Double rampRate)
    {
        this.rampRate = rampRate;
    }   //setRampRate

    /**
     * This method sets a new target tolerance.
     *
     * @param tolerance specifies the new target tolerance.
     */
    public synchronized void setTargetTolerance(double tolerance)
    {
        this.tolerance = tolerance;
    }   //setTargetTolerance

    /**
     * This method sets a range limit on the target set point.
     *
     * @param minTarget specifies the target set point lower range limit.
     * @param maxTarget specifies the target set point higher range limit.
     */
    public synchronized void setTargetRange(double minTarget, double maxTarget)
    {
        final String funcName = "setTargetRange";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "min=%f,max=%f", minTarget, maxTarget);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.minTarget = minTarget;
        this.maxTarget = maxTarget;
    }   //setTargetRange

    /**
     * This method sets a range limit on the calculated output. It is very useful to limit the output range to
     * less than full power for scenarios such as using mecanum wheels on a drive train to prevent wheel slipping
     * or slow down a PID drive in order to detect a line etc.
     *
     * @param minOutput specifies the PID output lower range limit.
     * @param maxOutput specifies the PID output higher range limit.
     */
    public synchronized void setOutputRange(double minOutput, double maxOutput)
    {
        final String funcName = "setOutputRange";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "min=%f,max=%f", minOutput, maxOutput);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (maxOutput <= minOutput)
        {
            throw new IllegalArgumentException("maxOutput must be greater than minOutput");
        }

        if (Math.abs(minOutput) == Math.abs(maxOutput))
        {
            outputLimit = maxOutput;
        }

        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
    }   //setOutputRange

    /**
     * This method sets the output to the range -limit to +limit. It calls setOutputRange. If the caller wants
     * to limit the output power symmetrically, this is the method to call, not setOutputRange.
     *
     * @param limit specifies the output limit as a positive number.
     */
    public synchronized void setOutputLimit(double limit)
    {
        limit = Math.abs(limit);
        setOutputRange(-limit, limit);
    }   //setOutputLimit

    /**
     * This method returns the last set output limit. It is sometimes useful to temporarily change the output
     * range of the PID controller for an operation and restore it afterwards. This method allows the caller to
     * save the last set output limit and restore it later on.
     *
     * @return last set output limit.
     */
    public synchronized double getOutputLimit()
    {
        final String funcName = "getOutputLimit";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", outputLimit);
        }

        return outputLimit;
    }   //getOutputLimit

    /**
     * This method saves the current output limit of the PID controller and sets it to the given new limit.
     * This is useful if the caller wants to temporarily set a limit for an operation and restore it afterwards.
     * Note: this is implemented with a stack so it is assuming the saving and restoring calls are nested in
     * nature. If this is called in a multi-threading environment where saving and restoring can be interleaved
     * by different threads, unexpected result may happen. It is recommended to avoid this type of scenario if
     * possible.
     *
     * @param limit specifies the new output limit.
     * @return return the previous output limit.
     */
    public synchronized double saveAndSetOutputLimit(double limit)
    {
        final String funcName = "saveAndSetOutputLimit";
        double prevLimit = outputLimit;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "limit=%f", limit);
        }

        outputLimitStack.push(outputLimit);
        setOutputLimit(limit);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", prevLimit);
        }

        return prevLimit;
    }   //saveAndSetOutputLimit

    /**
     * This method restores the last saved output limit and return its value. If there was no previous call to
     * saveAndSetOutputLimit, the current output limit is returned and the limit is not changed.
     *
     * @return last saved output limit.
     */
    public synchronized double restoreOutputLimit()
    {
        final String funcName = "restoreOutputLimit";
        double limit;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        try
        {
            limit = outputLimitStack.pop();
            setOutputRange(-limit, limit);
        }
        catch (EmptyStackException e)
        {
            //
            // There was no previous saveAndSetOutputLimit call, don't do anything and just return the current
            // output limit.
            //
            limit = outputLimit;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", limit);
        }

        return limit;
    }   //restoreOutputLimit

    /**
     * This method returns the current set point value.
     *
     * @return current set point.
     */
    public synchronized double getTarget()
    {
        final String funcName = "getTarget";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", setPoint);
        }

        return setPoint;
    }   //getTarget

    /**
     * This methods sets the target set point.
     *
     * @param target    specifies the target set point.
     * @param warpSpace specifies the warp space object if the target is in one, null if not.
     */
    public void setTarget(double target, TrcWarpSpace warpSpace)
    {
        final String funcName = "setTarget";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "target=%f,warpSpace=%s", target, warpSpace);
        }
        //
        // Read from input device without holding a lock on this object, since this could
        // be a long-running call.
        //
        final double input = pidInput.get();

        synchronized (this)
        {
            if (!absSetPoint)
            {
                //
                // Set point is relative, add target to current input to get absolute set point.
                //
                setPoint = input + target;
                currError = target;
            }
            else
            {
                //
                // Set point is absolute, use as is but optimize it if it is in warp space.
                //
                setPoint = target;
                if (warpSpace != null)
                {
                    setPoint = warpSpace.getOptimizedTarget(setPoint, input);
                }
                currError = setPoint - input;
            }

            if (inverted)
            {
                currError = -currError;
            }

            setPointSign = Math.signum(currError);
            //
            // If there is a valid target range, limit the set point to this range.
            //
            if (maxTarget > minTarget)
            {
                if (setPoint > maxTarget)
                {
                    setPoint = maxTarget;
                }
                else if (setPoint < minTarget)
                {
                    setPoint = minTarget;
                }
            }

            totalError = 0.0;
            prevTime = settlingStartTime = TrcUtil.getCurrentTime();
            // Only init the prevOutputTime if this setTarget is called after a reset()
            // If it's called mid-operation, we don't want to reset the prevOutputTime clock
            if (prevOutputTime == 0.0)
            {
                prevOutputTime = prevTime;
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setTarget

    /**
     * This methods sets the target set point.
     *
     * @param target specifies the target set point.
     */
    public void setTarget(double target)
    {
        setTarget(target, null);
    }   //setTarget

    /**
     * This method returns the error of a previous output calculation.
     *
     * @return previous error.
     */
    public synchronized double getError()
    {
        final String funcName = "getError";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", currError);
        }

        return currError;
    }   //getError

    /**
     * This method resets the PID controller clearing the set point, error, total error and output.
     */
    public synchronized void reset()
    {
        final String funcName = "reset";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        currError = 0.0;
        prevTime = 0.0;
        prevOutputTime = 0.0;
        totalError = 0.0;
        setPoint = 0.0;
        setPointSign = 1.0;
        output = 0.0;
    }   //reset

    /**
     * This method determines if we have reached the set point target. It is considered on target if the previous
     * error is smaller than the tolerance and is maintained for at least settling time. If NoOscillation mode is
     * set, it is considered on target if we are within tolerance or pass target regardless of setting time.
     *
     * @return true if we reached target, false otherwise.
     */
    public synchronized boolean isOnTarget()
    {
        final String funcName = "isOnTarget";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        boolean onTarget = false;

        if (noOscillation)
        {
            //
            // Don't allow oscillation, so if we are within tolerance or we pass target, just quit.
            // If setPointSign is positive, it means the target is "forward". So if currError <= tolerance,
            // it means we are either within tolerance or have passed the target.
            // If setPointSign is negative, it means the target is "backward". So if -currError <= tolerance,
            // it means we are either within tolerance or have passed the target.
            //
            if (currError * setPointSign <= tolerance)
            {
                onTarget = true;
            }
        }
        else if (Math.abs(currError) > tolerance)
        {
            settlingStartTime = TrcUtil.getCurrentTime();
        }
        else if (TrcUtil.getCurrentTime() >= settlingStartTime + settlingTime)
        {
            onTarget = true;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Boolean.toString(onTarget));
        }

        return onTarget;
    }   //isOnTarget

    /**
     * This method returns the current PID input value.
     *
     * @return current PID input value.
     */
    public double getCurrentInput()
    {
        return pidInput.get();
    }   //getCurrentInput

    /**
     * This method calculates the PID output applying the PID equation to the given set point target and current
     * input value.
     *
     * @return PID output value.
     */
    public double getOutput()
    {
        //
        // Read from input device without holding a lock on this object, since this could
        // be a long-running call.
        //
        final double currentInputValue = pidInput.get();

        synchronized (this)
        {
            final String funcName = "getOutput";

            if (debugEnabled)
            {
                dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            }

            double prevError = currError;
            double currTime = TrcUtil.getCurrentTime();
            double deltaTime = currTime - prevTime;
            prevTime = currTime;
            currInput = currentInputValue;
            currError = setPoint - currInput;
            if (inverted)
            {
                currError = -currError;
            }

            if (pidCoefficients.kI != 0.0)
            {
                //
                // Make sure the total error doesn't get wound up too much exceeding maxOutput.
                //
                double potentialGain = (totalError + currError * deltaTime) * pidCoefficients.kI;
                if (potentialGain >= maxOutput)
                {
                    totalError = maxOutput / pidCoefficients.kI;
                }
                else if (potentialGain > minOutput)
                {
                    totalError += currError * deltaTime;
                }
                else
                {
                    totalError = minOutput / pidCoefficients.kI;
                }
            }

            pTerm = pidCoefficients.kP * currError;
            iTerm = pidCoefficients.kI * totalError;
            dTerm = deltaTime > 0.0 ? pidCoefficients.kD * (currError - prevError) / deltaTime : 0.0;
            fTerm = pidCoefficients.kF * setPoint;
            double lastOutput = output;
            output = pTerm + iTerm + dTerm + fTerm;

            output = TrcUtil.clipRange(output, minOutput, maxOutput);

            if (rampRate != null)
            {
                if (prevOutputTime != 0.0)
                {
                    double dt = currTime - prevOutputTime;
                    double maxChange = rampRate * dt;
                    double change = output - lastOutput;
                    change = TrcUtil.clipRange(change, -maxChange, maxChange);
                    output = lastOutput + change;
                }
                prevOutputTime = currTime;
            }

            if (debugTracer != null)
            {
                printPidInfo(debugTracer);
            }

            if (debugEnabled)
            {
                dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", output);
            }

            return output;
        }
    }   //getOutput

}   //class TrcPidController
