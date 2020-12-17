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

import java.io.File;
import java.util.Locale;

import hallib.HalDbgLog;

/**
 * This class implements the Debug Tracer.
 */
public class TrcDbgTrace
{
    /**
     * This enum specifies the different debug tracing levels. They are used in the traceEnter and traceExit methods.
     */
    public enum TraceLevel
    {
        QUIET(0),
        INIT(1),
        API(2),
        CALLBK(3),
        EVENT(4),
        FUNC(5),
        TASK(6),
        UTIL(7),
        HIFREQ(8);

        private int value;

        TraceLevel(int value)
        {
            this.value = value;
        }   //TraceLevel

        public int getValue()
        {
            return this.value;
        }   //getValue

    }   //enum TraceLevel

    /**
     * This enum specifies the different debug message levels. They are used in the traceMsg methods.
     */
    public enum MsgLevel
    {
        FATAL(1),
        ERR(2),
        WARN(3),
        INFO(4),
        VERBOSE(5);

        private int value;

        MsgLevel(int value)
        {
            this.value = value;
        }   //MsgLevel

        public int getValue()
        {
            return this.value;
        }   //getValue

    }   //enum MsgLevel

    private static TrcDbgTrace globalTracer = null;
    private static int indentLevel = 0;

    private String instanceName;
    private boolean traceEnabled;
    private TraceLevel traceLevel;
    private MsgLevel msgLevel;
    private TrcTraceLogger traceLogger = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param traceEnabled specifies true to enable debug tracing, false to disable.
     * @param traceLevel specifies the trace level.
     * @param msgLevel specifies the message level.
     */
    public TrcDbgTrace(final String instanceName, boolean traceEnabled, TraceLevel traceLevel, MsgLevel msgLevel)
    {
        this.instanceName = instanceName;
        setDbgTraceConfig(traceEnabled, traceLevel, msgLevel);
    }   //TrcDbgTrace

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
            globalTracer = new TrcDbgTrace(
                "GlobalTracer", false, TrcDbgTrace.TraceLevel.API, TrcDbgTrace.MsgLevel.INFO);
        }

        return globalTracer;
    }   //getGlobalTracer

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
     * This method opens a log file for writing all the trace messages to it.
     *
     * @param traceLogName specifies the full trace log file path name.
     * @return true if log file is successfully opened, false if it failed.
     */
    public boolean openTraceLog(final String traceLogName)
    {
        boolean success = false;

        if (traceLogger == null)
        {
            traceLogger = new TrcTraceLogger(traceLogName);
            success = true;
        }

        return success;
    }   //openTraceLog

    /**
     * This method opens a log file for writing all the trace messages to it. The log file is written to the specified
     * folder. The file name will be formed by concatenating the date-time stamp with the specified file name.
     *
     * @param folderPath specifies the folder path.
     * @param fileName specifies the file name, null if none provided.
     * @return true if log file is successfully opened, false if it failed.
     */
    public boolean openTraceLog(final String folderPath, final String fileName)
    {
        //
        // Create the folder if it doesn't exist.
        //
        File folder = new File(folderPath);
        folder.mkdir();
        //
        // Create full log file path.
        //
        String logFileName = folderPath + File.separator + TrcUtil.getTimestamp();

        if (fileName != null)
        {
            logFileName += "!" + fileName;
        }
        logFileName += ".log";

        return openTraceLog(logFileName);
    }   //openTraceLog

    /**
     * This method closes the trace log file.
     */
    public void closeTraceLog()
    {
        if (traceLogger != null)
        {
            traceLogger.setEnabled(false);
            traceLogger = null;
        }
    }   //closeTraceLog

    /**
     * This method checks if the tracer log is opened.
     *
     * @return true if tracer log is opened, false otherwise.
     */
    public boolean tracerLogIsOpened()
    {
        return traceLogger != null;
    }   //tracerLogIsOpened

    /**
     * This method returns the trace log file name if one is active.
     *
     * @return trace log file name if one is active, null if none.
     */
    public String getTraceLogName()
    {
        return traceLogger != null? traceLogger.toString(): null;
    }   //getTraceLogName

    /**
     * This method enables/disables the trace log.
     *
     * @param enabled specifies true to enable trace log, false otherwise.
     */
    public void setTraceLogEnabled(boolean enabled)
    {
        if (traceLogger != null)
        {
            traceLogger.setEnabled(enabled);
        }
    }   //setTraceLogEnabled

    /**
     * This method sets the trace level, message level of the debug tracer. It can also enables/disables function
     * tracing.
     *
     * @param traceEnabled specifies true to enable function tracing, false to disable.
     * @param traceLevel specifies the trace level.
     * @param msgLevel specifies the message level.
     */
    public void setDbgTraceConfig(boolean traceEnabled, TraceLevel traceLevel, MsgLevel msgLevel)
    {
        this.traceEnabled = traceEnabled;
        this.traceLevel = traceLevel;
        this.msgLevel = msgLevel;
    }   //setDbgTraceConfig

    /**
     * This method logs a MsgLevel.INFO entry that contains information about the match. The entry is in XML format
     * and is intended to be parsed by tools such as TrcTraceLogVisualizer.
     *
     * @param funcName specifies the calling method name.
     * @param infoName specifies the name to identify the information.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public void logInfo(String funcName, String infoName, String format, Object... args)
    {
        traceMsg(funcName, MsgLevel.INFO, "<Info name=\"" + infoName + "\" " + format + " />", args);
    }   //logInfo

    /**
     * This method logs a MsgLevel.INFO entry that contains an event. The entry is in XML format and is intended to be
     * parsed by tools such as TrcTraceLogVisualizer.
     *
     * @param funcName specifies the calling method name.
     * @param eventName specifies the name to identify the event.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public void logEvent(final String funcName, final String eventName, final String format, Object... args)
    {
        String newFormat = String.format(Locale.US, "<Event name=\"%s\" time=\"%.3f\" %s />",
                eventName, TrcUtil.getModeElapsedTime(), format);
        traceMsg(funcName, MsgLevel.INFO, newFormat, args);
    }   //logEvent

    /**
     * This method logs a state info event using the global tracer. The state info event can be used to debug an
     * autonomous state machine. If the state involves PID controlled driving, it also logs the robot's movement.
     *
     * @param state specifies the current state of the state machine.
     * @param driveBase specifies the robot drive base, can be null if the state does not involve robot movement.
     * @param pidDrive specifies the pidDrive object, can be null if the state does not involve robot movement.
     * @param battery specifies the robot battery object, can be null if not interested in battery info.
     */
    public void traceStateInfo(Object state, TrcDriveBase driveBase, TrcPidDrive pidDrive, TrcRobotBattery battery)
    {
        StringBuilder msg = new StringBuilder();

        msg.append(String.format(Locale.US, "tag=\">>>>>\" state=\"%s\"", state));
        if (driveBase != null && pidDrive != null && pidDrive.isActive())
        {
            TrcPose2D robotPose = driveBase.getFieldPosition();
            TrcPose2D targetPose = pidDrive.getAbsoluteTargetPose();

            if (pidDrive.getXPidCtrl() != null)
            {
                msg.append(String.format(Locale.US, " xPos=%6.2f xTarget=%6.2f", robotPose.x, targetPose.x));
            }

            if (pidDrive.getYPidCtrl() != null)
            {
                msg.append(String.format(Locale.US, " yPos=%6.2f yTarget=%6.2f", robotPose.y, targetPose.y));
            }

            if (pidDrive.getTurnPidCtrl() != null)
            {
                msg.append(String.format(Locale.US, " heading=%6.2f headingTarget=%6.2f",
                        robotPose.angle, targetPose.angle));
            }
        }

        if (battery != null)
        {
            msg.append(String.format(Locale.US,
                    " volt=\"%5.2fV(%5.2fV)\"", battery.getVoltage(), battery.getLowestVoltage()));
        }

        logEvent("traceStateInfo", "StateInfo", "%s", msg);
    }   //traceStateInfo

    /**
     * This method logs a state info event. The state info event can be used to debug an autonomous state machine.
     * If the state involves PID controlled driving, it also logs the robot's movement.
     *
     * @param state specifies the current state of the state machine.
     * @param driveBase specifies the robot drive base, can be null if the state does not involve robot movement.
     * @param pidDrive specifies the pidDrive object, can be null if the state does not involve robot movement.
     */
    public void traceStateInfo(Object state, TrcDriveBase driveBase, TrcPidDrive pidDrive)
    {
        traceStateInfo(state, driveBase, pidDrive, null);
    }   //traceStateInfo

    /**
     * This method logs a state info event. The state info event can be used to debug an autonomous state machine.
     *
     * @param state specifies the current state of the state machine.
     * @param battery specifies the robot battery object, can be null if not interested in battery info.
     */
    public void traceStateInfo(Object state, TrcRobotBattery battery)
    {
        traceStateInfo(state, null, null, battery);
    }   //traceStateInfo

    /**
     * This method logs a state info event. The state info event can be used to debug an autonomous state machine.
     *
     * @param state specifies the current state of the state machine.
     */
    public void traceStateInfo(Object state)
    {
        traceStateInfo(state, null, null, null);
    }   //traceStateInfo

    /**
     * This method is typically called at the beginning of a method to trace the entry parameters of the method.
     *
     * @param funcName specifies the calling method name.
     * @param funcLevel specifies the trace level.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public void traceEnter(final String funcName, final TraceLevel funcLevel, final String format, Object... args)
    {
        if (traceEnabled && funcLevel.getValue() <= traceLevel.getValue())
        {
            HalDbgLog.traceMsg(tracePrefix(funcName, true, false) + String.format(format, args) + ")\n");
        }
    }   //traceEnter

    /**
     * This method is typically called at the beginning of a method.
     *
     * @param funcName specifies the calling method name.
     * @param funcLevel specifies the trace level.
     */
    public void traceEnter(final String funcName, final TraceLevel funcLevel)
    {
        if (traceEnabled && funcLevel.getValue() <= traceLevel.getValue())
        {
            HalDbgLog.traceMsg(tracePrefix(funcName, true, true));
        }
    }   //traceEnter

    /**
     * This method is typically called at the end of a method to trace the return value of the method.
     *
     * @param funcName specifies the calling method name.
     * @param funcLevel specifies the trace level.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public void traceExit(final String funcName, final TraceLevel funcLevel, final String format, Object... args)
    {
        if (traceEnabled && funcLevel.getValue() <= traceLevel.getValue())
        {
            HalDbgLog.traceMsg(tracePrefix(funcName, false, false) + String.format(format, args) + "\n");
        }
    }   //traceExitMsg

    /**
     * This method is typically called at the end of a method.
     * @param funcName specifies the calling method name.
     * @param funcLevel specifies the trace level.
     */
    public void traceExit(final String funcName, final TraceLevel funcLevel)
    {
        if (traceEnabled && funcLevel.getValue() <= traceLevel.getValue())
        {
            HalDbgLog.traceMsg(tracePrefix(funcName, false, true));
        }
    }   //traceExit

    /**
     * This method is called to print a fatal message.
     *
     * @param funcName specifies the calling method name.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public void traceFatal(final String funcName, final String format, Object... args)
    {
        traceMsg(funcName, MsgLevel.FATAL, format, args);
    }   //traceFatal

    /**
     * This method is called to print an error message.
     *
     * @param funcName specifies the calling method name.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public void traceErr(final String funcName, final String format, Object... args)
    {
        traceMsg(funcName, MsgLevel.ERR, format, args);
    }   //traceErr

    /**
     * This method is called to print a warning message.
     *
     * @param funcName specifies the calling method name.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public void traceWarn(final String funcName, final String format, Object... args)
    {
        traceMsg(funcName, MsgLevel.WARN, format, args);
    }   //traceWarn

    /**
     * This method is called to print an information message.
     *
     * @param funcName specifies the calling method name.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public void traceInfo(final String funcName, final String format, Object... args)
    {
        traceMsg(funcName, MsgLevel.INFO, format, args);
    }   //traceInfo

    /**
     * This method is called to print a verbose message.
     *
     * @param funcName specifies the calling method name.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public void traceVerbose(final String funcName, final String format, Object... args)
    {
        traceMsg(funcName, MsgLevel.VERBOSE, format, args);
    }   //traceVerbose

    /**
     * This method is called to print a message only if the given interval timer has expired since the last
     * periodic message. This is useful to print out periodic status without overwhelming the debug console.
     *
     * @param funcName specifies the calling method name.
     * @param timer specifies the interval timer.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public void traceInfoAtInterval(final String funcName, TrcIntervalTimer timer, final String format, Object... args)
    {
        if (timer.hasExpired())
        {
            traceMsg(funcName, MsgLevel.INFO, format, args);
        }
    }   //traceInfoAtInterval

    /**
     * This method prints a debug message to the debug console.
     *
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public void tracePrintf(String format, Object... args)
    {
        HalDbgLog.traceMsg(String.format(format, args));
    }   //tracePrintf

    /**
     * This method is the common worker for all the trace message methods.
     *
     * @param funcName specifies the calling method name.
     * @param level specifies the message level.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    private void traceMsg(final String funcName, MsgLevel level, final String format, Object... args)
    {
        if (level.getValue() <= msgLevel.getValue())
        {
            String msg = msgPrefix(funcName, level) + String.format(format, args);
            HalDbgLog.msg(level, msg + "\n");
            if (traceLogger != null)
            {
                traceLogger.logMessage(msg);
            }
        }
    }   //traceMsg

    /**
     * This method returns a trace prefix string. The trace prefix includes the indentation, the instance name and
     * calling method name.
     *
     * @param funcName specifies the calling method name.
     * @param enter specifies true if it is a traceEnter call, false if it is a traceExit call.
     * @param newline specifies true if it should print a newline, false otherwise.
     * @return trace prefix string.
     */
    private String tracePrefix(final String funcName, boolean enter, boolean newline)
    {
        String prefix = "";

        if (enter)
        {
            indentLevel++;
        }

        for (int i = 0; i < indentLevel; i++)
        {
            prefix += "| ";
        }

        prefix += instanceName + "." + funcName;

        if (enter)
        {
            prefix += newline? "()\n": "(";
        }
        else
        {
            prefix += newline? "!\n": "";
            indentLevel--;
        }

        return prefix;
    }   //tracePrefix

    /**
     * This method returns a message prefix string.
     *
     * @param funcName specifies the calling method name.
     * @param level specifies the message level.
     * @return message prefix string.
     */
    private String msgPrefix(final String funcName, MsgLevel level)
    {
        String prefix = instanceName + "." + funcName;

        switch (level)
        {
            case FATAL:
                prefix += "_Fatal: ";
                break;

            case ERR:
                prefix += "_Err: ";
                break;

            case WARN:
                prefix += "_Warn: ";
                break;

            case INFO:
                prefix += "_Info: ";
                break;

            case VERBOSE:
                prefix += "_Verbose: ";
                break;

            default:
                prefix += "_Unk: ";
                break;
        }

        return prefix;
    }   //msgPrefix

}   //class TrcDbgTrace
