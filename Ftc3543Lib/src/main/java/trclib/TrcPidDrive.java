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

import hallib.HalUtil;

public class TrcPidDrive implements TrcTaskMgr.Task
{
    private static final String moduleName = "TrcPidDrive";
    private static final boolean debugEnabled = false;
    private TrcDbgTrace dbgTrace = null;

    private static final int PIDDRIVEF_ENABLED          = (1 << 0);
    private static final int PIDDRIVEF_HOLD_TARGET      = (1 << 1);
    private static final int PIDDRIVEF_TURN_ONLY        = (1 << 2);
    private static final int PIDDRIVEF_SET_HEADING      = (1 << 3);
    private static final int PIDDRIVEF_CANCELED         = (1 << 4);

    private String instanceName;
    private TrcDriveBase driveBase;
    private TrcPidController xPidCtrl;
    private TrcPidController yPidCtrl;
    private TrcPidController turnPidCtrl;
    private TrcEvent notifyEvent;
    private double expiredTime;
    private int flags;
    private double manualX;
    private double manualY;

    public TrcPidDrive(
        final String instanceName,
        TrcDriveBase driveBase,
        TrcPidController xPidCtrl,
        TrcPidController yPidCtrl,
        TrcPidController turnPidCtrl)
    {
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(
                    moduleName + "." + instanceName,
                    false,
                    TrcDbgTrace.TraceLevel.API,
                    TrcDbgTrace.MsgLevel.INFO);
        }

        this.instanceName = instanceName;
        this.driveBase = driveBase;
        this.xPidCtrl = xPidCtrl;
        this.yPidCtrl = yPidCtrl;
        this.turnPidCtrl = turnPidCtrl;
        this.notifyEvent = null;
        this.expiredTime = 0.0;
        this.flags = 0;
        this.manualX = 0.0;
        this.manualY = 0.0;
    }   //TrcPidDrive

    public void setPidPower(double xPower, double yPower, double turnPower)
    {
        final String funcName = "setPidPower";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API, "xPwr=%f,yPwr=%f,turnPwr=%f", xPower, yPower, turnPower);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setPidPower

    public void setTarget(
            double xTarget,
            double yTarget,
            double turnTarget,
            boolean holdTarget,
            TrcEvent event,
            double timeout)
    {
        final String funcName = "setTarget";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "x=%f,y=%f,turn=%f,hold=%s,event=%s,timeout=%f",
                    xTarget, yTarget, turnTarget, Boolean.toString(holdTarget),
                    event.toString(), timeout);
        }

        if (xPidCtrl != null)
        {
            xPidCtrl.setTarget(xTarget);
        }

        if (yPidCtrl != null)
        {
            yPidCtrl.setTarget(yTarget);
        }

        if (turnPidCtrl != null)
        {
            turnPidCtrl.setTarget(turnTarget);
        }

        if (event != null)
        {
            event.clear();
        }
        this.notifyEvent = event;
        this.expiredTime = timeout;
        if (timeout != 0)
        {
            this.expiredTime += HalUtil.getCurrentTime();
        }

        flags = 0;
        if (holdTarget)
        {
            flags |= PIDDRIVEF_HOLD_TARGET;
        }

        if (xTarget == 0.0 && yTarget == 0.0 && turnTarget != 0.0)
        {
            flags |= PIDDRIVEF_TURN_ONLY;
        }

        setEnabled(true);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setTarget

    public void setTarget(
            double xTarget,
            double yTarget,
            double turnTarget,
            boolean holdTarget,
            TrcEvent event)
    {
        setTarget(xTarget, yTarget, turnTarget, holdTarget, event, 0.0);
    }   //setTarget

    public void setTarget(
            double yTarget,
            double turnTarget,
            boolean holdTarget,
            TrcEvent event,
            double timeout)
    {
        setTarget(
                0.0,
                yTarget,
                turnTarget,
                holdTarget,
                event,
                timeout);
    }   //setTarget

    public void setTarget(
            double yTarget,
            double turnTarget,
            boolean holdTarget,
            TrcEvent event)
    {
        setTarget(
                0.0,
                yTarget,
                turnTarget,
                holdTarget,
                event,
                0.0);
    }   //setTarget

    public void setHeadingTarget(
            double xPower,
            double yPower,
            double headingTarget)
    {
        final String funcName = "setHeadingTarget";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "xPower=%f,yPower=%f,heading=%f",
                    xPower, yPower, headingTarget);
        }

        if (xPidCtrl != null)
        {
            manualX = xPower;
            manualY = yPower;
            if (turnPidCtrl != null)
            {
                turnPidCtrl.setTarget(headingTarget);
            }
            flags = PIDDRIVEF_SET_HEADING;
            setEnabled(true);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setHeadingTarget

    public boolean isEnabled()
    {
        final String funcName = "isEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "=%s", Boolean.toString((flags & PIDDRIVEF_ENABLED) != 0));
        }

        return (flags & PIDDRIVEF_ENABLED) != 0;
    }   //isEnabled

    public void cancel()
    {
        final String funcName = "cancel";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if ((flags & PIDDRIVEF_ENABLED) != 0)
        {
            stop();
            flags |= PIDDRIVEF_CANCELED;
            if (notifyEvent != null)
            {
                notifyEvent.cancel();
                notifyEvent = null;
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //cancel

    public boolean isCanceled()
    {
        final String funcName = "isCanceled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "=%s", Boolean.toString((flags & PIDDRIVEF_CANCELED) != 0));
        }

        return (flags & PIDDRIVEF_CANCELED) != 0;
    }   //isCanceled

    private void stop()
    {
        final String funcName = "stop";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC);
        }

        setEnabled(false);
        driveBase.stop();

        if (xPidCtrl != null)
        {
            xPidCtrl.reset();
        }

        if (yPidCtrl != null)
        {
            yPidCtrl.reset();
        }

        if (turnPidCtrl != null)
        {
            turnPidCtrl.reset();
        }

        flags = 0;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC);
        }
    }   //stop

    private void setEnabled(boolean enabled)
    {
        final String funcName = "setEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.FUNC,
                    "enabled=%s", Boolean.toString(enabled));
        }

        TrcTaskMgr taskMgr = TrcTaskMgr.getInstance();
        if (enabled)
        {
            taskMgr.registerTask(
                    instanceName,
                    this,
                    TrcTaskMgr.TaskType.STOP_TASK);
            taskMgr.registerTask(
                    instanceName,
                    this,
                    TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
            flags |= PIDDRIVEF_ENABLED;
        }
        else
        {
            taskMgr.unregisterTask(
                    this,
                    TrcTaskMgr.TaskType.STOP_TASK);
            taskMgr.unregisterTask(
                    this,
                    TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
            flags &= ~PIDDRIVEF_ENABLED;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC);
        }
    }   //setEnabled

    //
    // Implements TrcTaskMgr.Task
    //

    @Override
    public void startTask(TrcRobot.RunMode runMode)
    {
    }   //startTask

    @Override
    public void stopTask(TrcRobot.RunMode runMode)
    {
        final String funcName = "stopTask";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.TASK,
                    "mode=%s", runMode.toString());
        }

        stop();

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //stopTask

    @Override
    public void prePeriodicTask(TrcRobot.RunMode runMode)
    {
    }   //prePeriodicTask

    @Override
    public void postPeriodicTask(TrcRobot.RunMode runMode)
    {
    }   //postPeriodicTask

    @Override
    public void preContinuousTask(TrcRobot.RunMode runMode)
    {
    }   //preContinuousTask

    @Override
    public void postContinuousTask(TrcRobot.RunMode runMode)
    {
        final String funcName = "postPeriodic";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.TASK,
                    "mode=%s", runMode.toString());
        }

        double xPower =
                (((flags & PIDDRIVEF_TURN_ONLY) != 0) || (xPidCtrl == null))?
                0.0: xPidCtrl.getOutput();
        double yPower =
                (((flags & PIDDRIVEF_TURN_ONLY) != 0) || (yPidCtrl == null))?
                0.0: yPidCtrl.getOutput();
        double turnPower = (turnPidCtrl == null)? 0.0: turnPidCtrl.getOutput();

        boolean expired =
                expiredTime != 0.0 && HalUtil.getCurrentTime() >= expiredTime;
        boolean xOnTarget = xPidCtrl == null || xPidCtrl.isOnTarget();
        boolean yOnTarget = yPidCtrl == null || yPidCtrl.isOnTarget();
        boolean turnOnTarget = turnPidCtrl == null || turnPidCtrl.isOnTarget();

        if ((flags & PIDDRIVEF_SET_HEADING) != 0)
        {
            driveBase.mecanumDrive_Cartesian(manualX, manualY, turnPower, false, 0.0);
        }
        else if (expired ||
                 turnOnTarget &&
                 ((flags & PIDDRIVEF_TURN_ONLY) != 0 ||
                  xOnTarget && yOnTarget))
        {
            if ((flags & PIDDRIVEF_HOLD_TARGET) == 0)
            {
                stop();
                if (notifyEvent != null)
                {
                    notifyEvent.set(true);
                    notifyEvent = null;
                }
            }
            else if (xPidCtrl != null)
            {
                driveBase.mecanumDrive_Cartesian(0.0, 0.0, 0.0, false, 0.0);
            }
            else
            {
                driveBase.drive(0.0, 0.0);
            }
        }
        else if (xPidCtrl != null)
        {
            driveBase.mecanumDrive_Cartesian(xPower, yPower, turnPower, false, 0.0);
        }
        else
        {
            driveBase.arcadeDrive(yPower, turnPower);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //postContinuousTask

}   //class TrcPidDrive
