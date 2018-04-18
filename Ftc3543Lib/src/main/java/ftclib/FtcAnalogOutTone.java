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

import com.qualcomm.robotcore.hardware.AnalogOutput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import trclib.TrcDbgTrace;
import trclib.TrcRobot;
import trclib.TrcTone;
import trclib.TrcTaskMgr;
import trclib.TrcUtil;

/**
 * This class implements a platform dependent sound player that can play a tone with specified waveform, frequency,
 * duration and volume using the Analog Output Port on the Core Device Interface Module as a tone generator.
 * The Analog Output port can be programmed to generate sine, square or triangle wave with specified amplitude
 * so it is perfect to be used as a tone generator.
 */
public class FtcAnalogOutTone extends TrcTone
{
    private static final String moduleName = "FtcAnalogOutTone";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private static final int MAX_VOLTAGE = 1023;
    private static final Waveform DEF_WAVEFORM = Waveform.TRIANGLE_WAVE;

    private String instanceName;
    private AnalogOutput analogOut;
    private TrcTaskMgr.TaskObject stopTaskObj;
    private TrcTaskMgr.TaskObject postContinuousTaskObj;
    private boolean playing = false;
    private double expiredTime = 0.0;

    /**
     * Constructor: Create and initialize an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     * @param defWaveform specifies the default waveform type.
     */
    public FtcAnalogOutTone(HardwareMap hardwareMap, String instanceName, Waveform defWaveform)
    {
        super(instanceName, defWaveform);

        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
        analogOut = hardwareMap.analogOutput.get(instanceName);
        stopTaskObj = TrcTaskMgr.getInstance().createTask(
                instanceName + ".stopTask", this::stopTask);
        postContinuousTaskObj = TrcTaskMgr.getInstance().createTask(
                instanceName + ".postContinuous", this::postContinuousTask);
    }   //FtcAnalogOutTone

    /**
     * Constructor: Create and initialize an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param defWaveform specifies the default waveform type.
     */
    public FtcAnalogOutTone(String instanceName, Waveform defWaveform)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, defWaveform);
    }   //FtcAnalogOutTone

    /**
     * Constructor: Create and initialize an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FtcAnalogOutTone(String instanceName)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, DEF_WAVEFORM);
    }   //FtcAnalogOutTone

    /**
     * This method enables/disables the periodic task that checks the expiration of the playing tone.
     *
     * @param enabled specifies true to enable the state machine task, false otherwise.
     */
    private void setTaskEnabled(boolean enabled)
    {
        final String funcName = "setTaskEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enabled=%s", Boolean.toString(enabled));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (enabled)
        {
            stopTaskObj.registerTask(TrcTaskMgr.TaskType.STOP_TASK);
            postContinuousTaskObj.registerTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        }
        else
        {
            stopTaskObj.unregisterTask(TrcTaskMgr.TaskType.STOP_TASK);
            postContinuousTaskObj.unregisterTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        }
    }   //setTaskEnabled

    //
    // Implements TrcTone abstract methods.
    //

    /**
     * This method plays a tone with the specified waveform, frequency, duration and volume.
     *
     * @param waveform specifies the waveform type.
     * @param frequency specifies the tone frequency in Hz.
     * @param duration specifies the duration in seconds.
     * @param volume specifies the volume in the range 0.0 to 1.0.
     */
    @Override
    public void playTone(Waveform waveform, double frequency, double duration, double volume)
    {
        final String funcName = "playTone";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                                "waveform=%s,freq=%.0f,dur=%.3f,vol=%.1f",
                                waveform.toString(), frequency, duration, volume);
        }

        if (volume < 0.0 || volume > 1.0)
        {
            throw new IllegalArgumentException("Volume must be in the range 0.0 to 1.0.");
        }

        if (frequency < 0.0)
        {
            throw new IllegalArgumentException("Frequency cannot be negative.");
        }

        if (duration < 0.0)
        {
            throw new IllegalArgumentException("Duration cannot be negative.");
        }

        byte outputMode = 0;
        switch (waveform)
        {
            case SINE_WAVE:
                outputMode = 1;
                break;

            case SQUARE_WAVE:
                outputMode = 2;
                break;

            case TRIANGLE_WAVE:
                outputMode = 3;
                break;
        }

        analogOut.setAnalogOutputMode(outputMode);
        analogOut.setAnalogOutputFrequency((int)frequency);
        analogOut.setAnalogOutputVoltage((int)(MAX_VOLTAGE*volume));
        expiredTime = TrcUtil.getCurrentTime() + duration;
        playing = true;
        setTaskEnabled(true);

        if (debugEnabled)
        {
            dbgTrace.traceInfo(funcName,
                               "mode=%d,freq=%d,dur=%.3f,vol=%.1f", outputMode, (int)frequency, duration, volume);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //playTone

    /**
     * This method stops the playing of the sound in progress.
     */
    @Override
    public void stop()
    {
        final String funcName = "stop";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (playing)
        {
            analogOut.setAnalogOutputMode((byte) 0);
            analogOut.setAnalogOutputFrequency(0);
            analogOut.setAnalogOutputVoltage(0);
            playing = false;
            expiredTime = 0.0;
            setTaskEnabled(false);
        }
    }   //stop

    /**
     * This method checks if the sound is still playing.
     *
     * @return true if the sound is still playing, false otherwise.
     */
    @Override
    public boolean isPlaying()
    {
        final String funcName = "isPlaying";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Boolean.toString(playing));
        }

        return playing;
    }   //isPlaying

    //
    // Implements TrcTaskMgr.Task interface.
    //

    /**
     * This method contains code that will clean up the task before
     * a competition mode is about to end. It stops the sound playing.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is about to
     *                end (e.g. Autonomous, TeleOp).
     */
    public void stopTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        stop();
    }   //stopTask

    /**
     * This method is called periodically to check if the duration of the sound playing has expired.
     * If so, it will stop the sound.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is running.
     */
    public void postContinuousTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = "postContinuousTask";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "runMode=%s", runMode.toString());
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }

        if (expiredTime > 0.0 && TrcUtil.getCurrentTime() >= expiredTime)
        {
            stop();
        }
    }   //postContinuousTask

}   //class FtcAnalogOutTone
