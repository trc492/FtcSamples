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

import com.qualcomm.robotcore.hardware.AnalogOutput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import hallib.HalUtil;
import trclib.TrcDbgTrace;
import trclib.TrcRobot;
import trclib.TrcTone;
import trclib.TrcTaskMgr;

/**
 * This class implements a platform dependent sound player that can play a tone with specified waveform, frequency,
 * duration and volume using the Analog Output Port on the Core Device Interface Module as a tone generator.
 * The Analog Output port can be programmed to generate sine, square or triangle wave with specified amplitude
 * so it is perfect to be used as a tone generator.
 */
public class FtcAnalogOutTone extends TrcTone implements TrcTaskMgr.Task
{
    private static final String moduleName = "FtcAnalogOutTone";
    private static final boolean debugEnabled = false;
    private TrcDbgTrace dbgTrace = null;

    private static final int MAX_VOLTAGE = 1023;
    private static final Waveform DEF_WAVEFORM = Waveform.TriangleWave;

    private String instanceName;
    private AnalogOutput analogOut;
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
            dbgTrace = new TrcDbgTrace(
                    moduleName + "." + instanceName,
                    false,
                    TrcDbgTrace.TraceLevel.API,
                    TrcDbgTrace.MsgLevel.INFO);
        }

        this.instanceName = instanceName;
        analogOut = hardwareMap.analogOutput.get(instanceName);
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
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API, "enabled=%s", Boolean.toString(enabled));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (enabled)
        {
            TrcTaskMgr.getInstance().registerTask(instanceName, this, TrcTaskMgr.TaskType.STOP_TASK);
            TrcTaskMgr.getInstance().registerTask(instanceName, this, TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        }
        else
        {
            TrcTaskMgr.getInstance().unregisterTask(this, TrcTaskMgr.TaskType.STOP_TASK);
            TrcTaskMgr.getInstance().unregisterTask(this, TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
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
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "waveform=%s,freq=%.0f,dur=%.3f,vol=%.1f", waveform.toString(), frequency, duration, volume);
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
            case SineWave:
                outputMode = 1;
                break;

            case SquareWave:
                outputMode = 2;
                break;

            case TriangleWave:
                outputMode = 3;
                break;
        }

        analogOut.setAnalogOutputMode(outputMode);
        analogOut.setAnalogOutputFrequency((int)frequency);
        analogOut.setAnalogOutputVoltage((int)(MAX_VOLTAGE*volume));
        expiredTime = HalUtil.getCurrentTime() + duration;
        playing = true;
        setTaskEnabled(true);

        if (debugEnabled)
        {
            dbgTrace.traceInfo(
                    funcName, "mode=%d,freq=%d,dur=%.3f,vol=%.1f", outputMode, (int)frequency, duration, volume);
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

    @Override
    public void startTask(TrcRobot.RunMode runMode)
    {
    }   //startTask

    /**
     * This method contains code that will clean up the task before
     * a competition mode is about to end. It stops the sound playing.
     *
     * @param runMode specifies the competition mode that is about to
     *                end (e.g. Autonomous, TeleOp).
     */
    @Override
    public void stopTask(TrcRobot.RunMode runMode)
    {
        stop();
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

    /**
     * This method is called periodically to check if the duration of the sound playing has expired.
     * If so, it will stop the sound.
     *
     * @param runMode specifies the competition mode that is running.
     */
    @Override
    public void postContinuousTask(TrcRobot.RunMode runMode)
    {
        final String funcName = "postContinuousTask";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "runMode=%s", runMode.toString());
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }

        if (expiredTime > 0.0 && HalUtil.getCurrentTime() >= expiredTime)
        {
            stop();
        }
    }   //postContinuousTask

}   //class FtcAnalogOutTone
