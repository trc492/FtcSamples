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

public class TrcCascadeController extends TrcPidController
{
    private static final String moduleName = "TrcCascadeController";
    private static final boolean debugEnabled = false;
    private TrcDbgTrace dbgTrace = null;

    public interface CascadeInput
    {
        public double getPrimaryInput(TrcCascadeController cascadeCtrl);
        public double getSecondaryInput(TrcCascadeController cascadeCtrl);
    }   //interface CascadeInput

    private TrcPidController secondaryCtrl;

    public TrcCascadeController(
            final String instanceName,
            double       kP,
            double       kI,
            double       kD,
            double       kF,
            double       tolerance,
            double       settlingTime,
            PidInput     pidInput,
            double       secondaryKp,
            double       secondaryKi,
            double       secondaryKd,
            double       secondaryKf)
    {
        super(instanceName + ".primary",
              kP, kI, kD, kF, tolerance, settlingTime, pidInput);

        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(
                    moduleName + "." + instanceName,
                    false,
                    TrcDbgTrace.TraceLevel.API,
                    TrcDbgTrace.MsgLevel.INFO);
        }

        secondaryCtrl =
                new TrcPidController(
                        instanceName + ".secondary",
                        secondaryKp, secondaryKi, secondaryKd, secondaryKf,
                        tolerance, settlingTime, pidInput);
    }   //TrcCascadeController

    public double getSecondaryKp()
    {
        final String funcName = "getSecondaryKp";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        double kp = secondaryCtrl.getKp();

        if (debugEnabled)
        {
            dbgTrace.traceExit(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "=%f", kp);
        }
        return kp;
    }   //getSecondaryKp

    public double getSecondaryKi()
    {
        final String funcName = "getSecondaryKi";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        double ki = secondaryCtrl.getKi();

        if (debugEnabled)
        {
            dbgTrace.traceExit(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "=%f", ki);
        }

        return ki;
    }   //getSecondaryKi

    public double getSecondaryKd()
    {
        final String funcName = "getSecondaryKd";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        double kd = secondaryCtrl.getKd();

        if (debugEnabled)
        {
            dbgTrace.traceExit(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "=%f", kd);
        }

        return kd;
    }   //getSecondaryKd

    public double getSecondaryKf()
    {
        final String funcName = "getSecondaryKf";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        double kf = secondaryCtrl.getKf();

        if (debugEnabled)
        {
            dbgTrace.traceExit(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "=%f", kf);
        }

        return kf;
    }   //getSecondaryKf

    public void setSecondaryKp(double kp)
    {
        final String funcName = "setSecondaryKp";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                    "Kp=%f", kp);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        secondaryCtrl.setKp(kp);
    }   //setSecondaryKp

    public void setSecondaryKi(double ki)
    {
        final String funcName = "setSecondaryKi";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                    "Ki=%f", ki);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        secondaryCtrl.setKp(ki);
    }   //setSecondaryKi

    public void setSecondaryKd(double kd)
    {
        final String funcName = "setSecondaryKd";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                    "Kd=%f", kd);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        secondaryCtrl.setKp(kd);
    }   //setSecondaryKd

    public void setSecondaryKf(double kf)
    {
        final String funcName = "setSecondaryKf";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                    "Kf=%f", kf);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        secondaryCtrl.setKf(kf);
    }   //setSecondaryKf

    public void setSecondaryPID(double kp, double ki, double kd, double kf)
    {
        final String funcName = "setSecondaryPID";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "Kp=%f,Ki=%f,Kd=%f,Kf=%f",
                    kp, ki, kd, kf);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        secondaryCtrl.setPID(kp, ki, kd, kf);
    }   //setSecondaryPID

    public void setSecondaryInputRange(double minInput, double maxInput)
    {
        final String funcName = "setSecondaryInputRange";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "min=%f,max=%f",
                    minInput, maxInput);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        secondaryCtrl.setInputRange(minInput, maxInput);
    }   //setSecondaryInputRange

    public void setSecondaryOutputRange(double minOutput, double maxOutput)
    {
        final String funcName = "setSecondaryOutputRange";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "min=%f,max=%f",
                    minOutput, maxOutput);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        secondaryCtrl.setOutputRange(minOutput, maxOutput);
    }   //setSecondaryOutputRange

    public void reset()
    {
        final String funcName = "reset";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        secondaryCtrl.reset();
        super.reset();
    }   //reset

    public double getOutput()
    {
        final String funcName = "getOutput";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        double output = super.getOutput();

        if (debugEnabled)
        {
            dbgTrace.traceExit(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "=%f", output);
        }

        return output;
    }   //getOutput

}   //class TrcCascadeController
