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

/**
 * This class implements a PID controller. A PID controller takes a target set point and an input from a feedback
 * device to calculate the output power of an effector usually a motor or a set of motors.
 */
public class TrcPidController
{
    private static final String moduleName = "TrcPidController";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

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
         * @param pidCtrl specifies this PID controller so the provider can identify what sensor to read if it is
         *                a provider for multiple PID controllers.
         * @return input value of the feedback device.
         */
        double getInput(TrcPidController pidCtrl);

    }   //interface PidInput

    private HalDashboard dashboard;
    private final String instanceName;
    private double kP;
    private double kI;
    private double kD;
    private double kF;
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

    private double prevTime = 0.0;
    private double prevError = 0.0;
    private double totalError = 0.0;
    private double settlingStartTime = 0.0;
    private double setPoint = 0.0;
    private double setPointSign = 1.0;
    private double input = 0.0;
    private double output = 0.0;

    private TrcDbgTrace debugTracer = null;
    private double pTerm;
    private double iTerm;
    private double dTerm;
    private double fTerm;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param kP specifies the proportional constant.
     * @param kI specifies the integral constant.
     * @param kD specifies the differential constant.
     * @param kF specifies the feed forward constant.
     * @param tolerance specifies the target tolerance.
     * @param settlingTime specifies the minimum on target settling time.
     * @param pidInput specifies the input provider.
     */
    public TrcPidController(
            final String instanceName,
            double       kP,
            double       kI,
            double       kD,
            double       kF,
            double       tolerance,
            double       settlingTime,
            PidInput     pidInput)
    {
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        dashboard = HalDashboard.getInstance();
        this.instanceName = instanceName;
        this.kP = Math.abs(kP);
        this.kI = Math.abs(kI);
        this.kD = Math.abs(kD);
        this.kF = Math.abs(kF);
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
     * @param instanceName specifies the instance name.
     * @param kP specifies the proportional constant.
     * @param kI specifies the integral constant.
     * @param kD specifies the differential constant.
     * @param kF specifies the feed forward constant.
     * @param tolerance specifies the target tolerance.
     * @param settlingTime specifies the minimum on target settling time.
     */
    protected TrcPidController(
            final String instanceName,
            double       kP,
            double       kI,
            double       kD,
            double       kF,
            double       tolerance,
            double       settlingTime)
    {
        this(instanceName, kP, kI, kD, kF, tolerance, settlingTime, null);
    }   //TrcPidController

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method can only be called by classes extending this class to set the input provider.
     *
     * @param pidInput specifies the input provider.
     */
    protected void setPidInput(PidInput pidInput)
    {
        this.pidInput = pidInput;
    }   //setPidInput

    /**
     * This method displays the PID information on the dashboard for debugging and tuning purpose. Note that the
     * PID info occupies two dashboard lines.
     *
     * @param lineNum specifies the starting line number of the dashboard to display the info.
     */
    public void displayPidInfo(int lineNum)
    {
        dashboard.displayPrintf(
                lineNum, "%s:Target=%.1f,Input=%.1f,Error=%.1f", instanceName, setPoint, input, prevError);
        dashboard.displayPrintf(
                lineNum + 1, "minOutput=%.1f,Output=%.1f,maxOutput=%.1f", minOutput, output, maxOutput);
    }   //displayPidInfo

    /**
     * This method prints the PID information to the tracer console. If no tracer is provided, it will attempt to
     * use the debug tracer in this module but if the debug tracer is not enabled, no output will be produced.
     *
     * @param tracer specifies the tracer object to print the PID info to.
     */
    public void printPidInfo(TrcDbgTrace tracer)
    {
        final String funcName = "printPidInfo";

        if (tracer == null)
        {
            tracer = dbgTrace;
        }

        if (tracer != null)
        {
            tracer.traceInfo(
                    funcName,
                    "%s: Target=%6.1f, Input=%6.1f, Error=%6.1f, PIDTerms=%6.3f/%6.3f/%6.3f/%6.3f, Output=%6.3f(%6.3f/%5.3f)",
                    instanceName, setPoint, input, prevError, pTerm, iTerm, dTerm, fTerm, output, minOutput, maxOutput);
        }
    }   //printPidInfo

    /**
     * This method prints the PID information to the default debug tracer.
     */
    public void printPidInfo()
    {
        printPidInfo(null);
    }   //printPidInfo

    /**
     * This method allows the caller to dynamically enable/disable debug tracing of the output calculation. It is
     * very useful for debugging or tuning PID control.
     *
     * @param tracer specifies the tracer to be used for debug tracing.
     * @param enabled specifies true to enable the debug tracer, false to disable.
     */
    public void setDebugTraceEnabled(TrcDbgTrace tracer, boolean enabled)
    {
        debugTracer = enabled? tracer: null;
    }   //setDebugTraceEnabled

    /**
     * This method inverts the sign of the calculated error. Normally, the calculated error starts with a large
     * positive number and goes down. However, in some sensors such as the ultrasonic sensor, the target is a small
     * number and the error starts with a negative value and increases. In order to calculate a correct output which
     * will go towards the target, the error sign must be inverted.
     *
     * @param inverted specifies true to invert the sign of the calculated error, false otherwise.
     */
    public void setInverted(boolean inverted)
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
    public void setAbsoluteSetPoint(boolean absolute)
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
     * This method enables/disables NoOscillation mode. In PID control, if the PID constants are not tuned quite
     * correctly, it may cause oscillation that could waste a lot of time. In some scenarios, passing the target
     * beyond the tolerance may be acceptable. This method allows the PID controller to declare "On Target" even
     * though it passes the target beyond tolerance so it doesn't oscillate.
     *
     * @param noOscillation specifies true to enable no oscillation, false to disable.
     */
    public void setNoOscillation(boolean noOscillation)
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
     * This method returns the current proportional constant.
     *
     * @return current proportional constant.
     */
    public double getKp()
    {
        final String funcName = "getKp";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", kP);
        }

        return kP;
    }   //getKp

    /**
     * This method returns the current integral constant.
     *
     * @return current integral constant.
     */
    public double getKi()
    {
        final String funcName = "getKi";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", kI);
        }

        return kI;
    }   //getKi

    /**
     * This method returns the current differential constant.
     *
     * @return current differential constant.
     */
    public double getKd()
    {
        final String funcName = "getKd";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", kD);
        }

        return kD;
    }   //getKd

    /**
     * This method returns the current feed forward constant.
     *
     * @return current feed forward constant.
     */
    public double getKf()
    {
        final String funcName = "getKf";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", kF);
        }

        return kF;
    }   //getKf

    /**
     * This method sets a new proportional constant.
     *
     * @param kP specifies a new proportional constant.
     */
    public void setKp(double kP)
    {
        final String funcName = "setKp";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "Kp=%f", kP);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.kP = kP;
    }   //setKp

    /**
     * This method sets a new integral constant.
     *
     * @param kI specifies a new integral constant.
     */
    public void setKi(double kI)
    {
        final String funcName = "setKi";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "Ki=%f", kI);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.kI = kI;
    }   //setKi

    /**
     * This method sets a new differential constant.
     *
     * @param kD specifies a new differential constant.
     */
    public void setKd(double kD)
    {
        final String funcName = "setKd";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "Kd=%f", kD);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.kD = kD;
    }   //setKd

    /**
     * This method sets a new feed forward constant.
     *
     * @param kF specifies a new feed forward constant.
     */
    public void setKf(double kF)
    {
        final String funcName = "setKf";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "Kf=%f", kF);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.kF = kF;
    }   //setKf

    /**
     * This method sets a new set of PID constants.
     *
     * @param kP specifies the new proportional constant.
     * @param kI specifies the new integral constant.
     * @param kD specifies the new differential constant.
     * @param kF specifies the new feed forward constant.
     */
    public void setPID(double kP, double kI, double kD, double kF)
    {
        final String funcName = "setPID";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "Kp=%f,Ki=%f,Kd=%f,Kf=%f", kP, kI, kD, kF);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }   //setPID

    /**
     * This method sets a new target tolerance.
     *
     * @param tolerance specifies the new target tolerance.
     */
    public void setTargetTolerance(double tolerance)
    {
        this.tolerance = tolerance;
    }   //setTargetTolerance

    /**
     * This method sets a range limit on the target set point.
     *
     * @param minTarget specifies the target set point lower range limit.
     * @param maxTarget specifies the target set point higher range limit.
     */
    public void setTargetRange(double minTarget, double maxTarget)
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
    public void setOutputRange(double minOutput, double maxOutput)
    {
        final String funcName = "setOutputRange";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "min=%f,max=%f",
                    minOutput, maxOutput);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
    }   //setOutputRange

    /**
     * This method returns the current set point value.
     *
     * @return current set point.
     */
    public double getTarget()
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
     * @param target specifies the target set point.
     */
    public void setTarget(double target)
    {
        final String funcName = "setTarget";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "target=%f", target);
        }

        double input = pidInput.getInput(this);
        if (!absSetPoint)
        {
            //
            // Set point is relative, add target to current input to get absolute set point.
            //
            setPoint = input + target;
            prevError = target;
        }
        else
        {
            //
            // Set point is absolute, use as is.
            //
            setPoint = target;
            prevError = setPoint - input;
        }

        if (inverted)
        {
            prevError = -prevError;
        }

        setPointSign = Math.signum(prevError);

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

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setTarget

    /**
     * This method returns the error of a previous output calculation.
     *
     * @return previous error.
     */
    public double getError()
    {
        final String funcName = "getError";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", prevError);
        }

        return prevError;
    }   //getError

    /**
     * This method resets the PID controller clearing the set point, error, total error and output.
     */
    public void reset()
    {
        final String funcName = "reset";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        prevError = 0.0;
        prevTime = 0.0;
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
    public boolean isOnTarget()
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
            //
            if (prevError*setPointSign <= tolerance)
            {
                onTarget = true;
            }
        }
        else if (Math.abs(prevError) > tolerance)
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
     * This method calculates the PID output applying the PID equation to the given set point target and current
     * input value.
     *
     * @return PID output value.
     */
    public double getOutput()
    {
        final String funcName = "getOutput";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        double currTime = TrcUtil.getCurrentTime();
        double deltaTime = currTime - prevTime;
        prevTime = currTime;
        input = pidInput.getInput(this);
        double error = setPoint - input;
        if (inverted)
        {
            error = -error;
        }

        if (kI != 0.0)
        {
            //
            // Make sure the total error doesn't get wound up too much exceeding maxOutput.
            //
            double potentialGain = (totalError + error * deltaTime) * kI;
            if (potentialGain >= maxOutput)
            {
                totalError = maxOutput / kI;
            }
            else if (potentialGain > minOutput)
            {
                totalError += error * deltaTime;
            }
            else
            {
                totalError = minOutput / kI;
            }
        }

        pTerm = kP*error;
        iTerm = kI*totalError;
        dTerm = deltaTime > 0.0? kD*(error - prevError)/deltaTime: 0.0;
        fTerm = kF*setPoint;
        output = fTerm + pTerm + iTerm + dTerm;

        prevError = error;
        if (output > maxOutput)
        {
            output = maxOutput;
        }
        else if (output < minOutput)
        {
            output = minOutput;
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
    }   //getOutput

}   //class TrcPidController
