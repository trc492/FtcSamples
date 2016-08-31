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

package trclib;

import ftclib.FtcOpMode;
import hallib.HalDashboard;
import hallib.HalUtil;

public class TrcPidController
{
    private static final String moduleName = "TrcPidController";
    private static final boolean debugEnabled = false;
    private TrcDbgTrace dbgTrace = null;

    public interface PidInput
    {
        public double getInput(TrcPidController pidCtrl);
    }   //interface PidInput

    private HalDashboard dashboard;
    private double kP;
    private double kI;
    private double kD;
    private double kF;
    private double tolerance;
    private double settlingTime;
    private PidInput pidInput;

    private boolean inverted = false;
    private boolean absSetPoint = false;
    private boolean speedControl = false;
    private boolean noOscillation = false;
    private double minInput = 0.0;
    private double maxInput = 0.0;
    private double minOutput = -1.0;
    private double maxOutput = 1.0;

    private double prevError = 0.0;
    private double totalError = 0.0;
    private double settlingStartTime = 0.0;
    private double setPoint = 0.0;
    private double output = 0.0;

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
            dbgTrace = new TrcDbgTrace(
                    moduleName + "." + instanceName,
                    false,
                    TrcDbgTrace.TraceLevel.API,
                    TrcDbgTrace.MsgLevel.INFO);
        }

        dashboard = FtcOpMode.getDashboard();
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.tolerance = tolerance;
        this.settlingTime = settlingTime;
        this.pidInput = pidInput;
    }   //TrcPidController

    public void displayPidInfo(int lineNum)
    {
        dashboard.displayPrintf(
                lineNum,
                "Target=%.1f, Input=%.1f, Error=%.1f",
                setPoint, pidInput.getInput(this), prevError);
        dashboard.displayPrintf(
                lineNum + 1,
                "minOutput=%.1f, Output=%.1f, maxOutput=%.1f",
                minOutput, output, maxOutput);
    }   //displayPidInfo

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
                    "Target=%6.1f, Input=%6.1f, Error=%6.1f, Output=%6.3f(%6.3f/%5.3f)",
                    setPoint, pidInput.getInput(this), prevError, output,
                    minOutput, maxOutput);
        }
    }   //printPidInfo

    public void printPidInfo()
    {
        printPidInfo(null);
    }   //printPidInfo

    public void setInverted(boolean inverted)
    {
        final String funcName = "setInverted";

        this.inverted = inverted;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                                "inverted=%s", Boolean.toString(inverted));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setInverted

    public void setAbsoluteSetPoint(boolean absolute)
    {
        final String funcName = "setAbsoluteSetPoint";

        this.absSetPoint = absolute;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                                "absolute=%s", Boolean.toString(absolute));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setAbsoluteSetPoint

    public void setSpeedControlMode(boolean speedControl)
    {
        final String funcName = "setSpeedControlMode";

        this.speedControl = speedControl;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                                "speedControl=%s", Boolean.toString(speedControl));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setSpeedControlMode

    public void setNoOscillation(boolean noOscillation)
    {
        final String funcName = "setNoOscillation";

        this.noOscillation = noOscillation;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                                "noOsc=%s", Boolean.toString(noOscillation));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setNoOscillation

    public double getKp()
    {
        final String funcName = "getKp";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "=%f", kP);
        }

        return kP;
    }   //getKp

    public double getKi()
    {
        final String funcName = "getKi";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "=%f", kI);
        }

        return kI;
    }   //getKi

    public double getKd()
    {
        final String funcName = "getKd";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "=%f", kD);
        }

        return kD;
    }   //getKd

    public double getKf()
    {
        final String funcName = "getKf";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "=%f", kF);
        }

        return kF;
    }   //getKf

    public void setKp(double kP)
    {
        final String funcName = "setKp";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                    "Kp=%f", kP);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.kP = kP;
    }   //setKp

    public void setKi(double kI)
    {
        final String funcName = "setKi";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "Ki=%f", kI);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.kI = kI;
    }   //setKi

    public void setKd(double kD)
    {
        final String funcName = "setKd";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "Kd=%f", kD);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.kD = kD;
    }   //setKd

    public void setKf(double kF)
    {
        final String funcName = "setKf";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "Kf=%f", kF);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.kF = kF;
    }   //setKf

    public void setPID(double kP, double kI, double kD, double kF)
    {
        final String funcName = "setPID";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "Kp=%f,Ki=%f,Kd=%f,Kf=%f",
                    kP, kI, kD, kF);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }   //setPID

    public void setInputRange(double minInput, double maxInput)
    {
        final String funcName = "setInputRange";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "min=%f,max=%f",
                    minInput, maxInput);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.minInput = minInput;
        this.maxInput = maxInput;
    }   //setInputRange

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

    public double getTarget()
    {
        final String funcName = "getTarget";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "=%f", setPoint);
        }

        return setPoint;
    }   //getTarget

    public void setTarget(double target)
    {
        final String funcName = "setTarget";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "target=%f", target);
        }

        double input = pidInput.getInput(this);
        setPoint = target;
        if (!absSetPoint)
        {
            setPoint += input;
        }

        if (maxInput > minInput)
        {
            if (setPoint > maxInput)
            {
                setPoint = maxInput;
            }
            else if (setPoint < minInput)
            {
                setPoint = minInput;
            }
        }

        prevError = setPoint - input;
        if (inverted)
        {
            prevError = -prevError;
        }
        totalError = 0.0;
        settlingStartTime = HalUtil.getCurrentTime();

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setTarget

    public double getError()
    {
        final String funcName = "getError";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "=%f", prevError);
        }

        return prevError;
    }   //getError

    public void reset()
    {
        final String funcName = "reset";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        prevError = 0.0;
        totalError = 0.0;
        setPoint = 0.0;
        output = 0.0;
    }   //reset

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
            if (Math.abs(prevError) <= tolerance)
            {
                onTarget = true;
            }
        }
        else if (Math.abs(prevError) > tolerance)
        {
            settlingStartTime = HalUtil.getCurrentTime();
        }
        else if (HalUtil.getCurrentTime() >= settlingStartTime + settlingTime)
        {
            onTarget = true;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "=%s", Boolean.toString(onTarget));
        }

        return onTarget;
    }   //isOnTarget

    public double getOutput()
    {
        final String funcName = "getOutput";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        double error = setPoint - pidInput.getInput(this);
        if (inverted)
        {
            error = -error;
        }

        if (kI != 0.0)
        {
            double potentialGain = (totalError + error)*kI;
            if (potentialGain >= maxOutput)
            {
                totalError = maxOutput/kI;
            }
            else if (potentialGain > minOutput)
            {
                totalError += error;
            }
            else
            {
                totalError = minOutput/kI;
            }
        }

        output =
                kP*error +
                kI*totalError +
                kD*(error - prevError) +
                kF*setPoint;

        prevError = error;
        if (output > maxOutput)
        {
            output = maxOutput;
        }
        else if (output < minOutput)
        {
            output = minOutput;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "=%f", output);
        }

        return output;
    }   //getOutput

}   //class TrcPidController
