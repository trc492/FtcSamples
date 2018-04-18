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

/**
 * This class implements a platform independent tone player that can play a tone with specified waveform, frequency,
 * duration and volume. This class should be extended by a platform dependent class that will implement the tone
 * playing abstract methods.
 */
public abstract class TrcTone
{
    private static final String moduleName = "TrcTone";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    /**
     * This enum type specifies the sound waveform to be used in playing a note.
     */
    public enum Waveform
    {
        SINE_WAVE,
        SQUARE_WAVE,
        TRIANGLE_WAVE
    }   //enum Waveform

    //
    // Abstract methods implemented by subclass.
    //

    /**
     * This method plays a tone with the specified waveform, frequency, duration and volume.
     *
     * @param waveform specifies the waveform type.
     * @param frequency specifies the tone frequency in Hz.
     * @param duration specifies the duration in seconds.
     * @param volume specifies the volume in the range 0.0 to 1.0.
     */
    public abstract void playTone(Waveform waveform, double frequency, double duration, double volume);

    /**
     * This method stops the playing tone.
     */
    public abstract void stop();

    /**
     * This method determines if the tone is still playing.
     *
     * @return true if tone is still playing, false otherwise.
     */
    public abstract boolean isPlaying();

    private Waveform defWaveform;

    /**
     * Constructor: Create and initialize an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param defWaveform specifies the default waveform type.
     */
    public TrcTone(final String instanceName, Waveform defWaveform)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.defWaveform = defWaveform;
    }   //TrcTone

    /**
     * This method plays a tone with the specified frequency, duration and volume using default waveform.
     *
     * @param frequency specifies the tone frequency in Hz.
     * @param duration specifies the duration in seconds.
     * @param volume specifies the volume in the range 0.0 to 1.0.
     */
    public void playTone(double frequency, double duration, double volume)
    {
        playTone(defWaveform, frequency, duration, volume);
    }   //playTone

    /**
     * This method plays a tone with the specified frequency and duration using default waveform and full volume.
     *
     * @param frequency specifies the tone frequency in Hz.
     * @param duration specifies the duration in seconds.
     */
    public void playTone(double frequency, double duration)
    {
        playTone(defWaveform, frequency, duration, 1.0);
    }   //playTone

    /**
     * This method generates the wave data for a sine wave with the specified frequency and volume. It is intended
     * to be called by the platform dependent wave player that extends this class.
     *
     * @param buffer specifies the buffer to hold the generated waveform data.
     * @param sampleRate specifies the sampling rate of tbe waveform.
     * @param frequency specifies the tone frequency in Hz.
     * @param volume specifies the volume in the range 0.0 to 1.0.
     */
    protected void genSineWave(short[] buffer, int sampleRate, double frequency, double volume)
    {
        final String funcName = "genSineWave";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC,
                                "sampleRate=%d,freq=%.0f,vol=%.2f", sampleRate, frequency, volume);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC);
        }

        for (int i = 0; i < buffer.length; i++)
        {
            buffer[i] = (short)(Math.sin(2*Math.PI*i/(sampleRate/frequency))*Short.MAX_VALUE*volume);
        }
    }   //genSineWave

    /**
     * This method generates the wave data for a square wave with the specified frequency and volume. It is intended
     * to be called by the platform dependent wave player that extends this class.
     *
     * @param buffer specifies the buffer to hold the generated waveform data.
     * @param sampleRate specifies the sampling rate of tbe waveform.
     * @param frequency specifies the tone frequency in Hz.
     * @param volume specifies the volume in the range 0.0 to 1.0.
     */
    protected void genSquareWave(short[] buffer, int sampleRate, double frequency, double volume)
    {
        final String funcName = "genSquareWave";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC,
                                "sampleRate=%d,freq=%.0f,vol=%.2f", sampleRate, frequency, volume);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC);
        }

        for (int i = 0; i < buffer.length; i++)
        {
            double data = ((i/(sampleRate/frequency))%1.0 >= 0.5)? Short.MIN_VALUE: Short.MAX_VALUE;
            buffer[i] = (short)(data*volume);
        }
    }   //genSquareWave

    /**
     * This method generates the wave data for a triangle wave with the specified frequency and volume. It is
     * intended to be called by the platform dependent wave player that extends this class.
     *
     * @param buffer specifies the buffer to hold the generated waveform data.
     * @param sampleRate specifies the sampling rate of tbe waveform.
     * @param frequency specifies the tone frequency in Hz.
     * @param volume specifies the volume in the range 0.0 to 1.0.
     */
    protected void genTriangleWave(short[] buffer, int sampleRate, double frequency, double volume)
    {
        final String funcName = "genTriangleWave";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC,
                                "sampleRate=%d,freq=%.0f,vol=%.2f", sampleRate, frequency, volume);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC);
        }

        for (int i = 0; i < buffer.length; i++)
        {
            double phase = (i/(sampleRate/frequency))%1.0;
            double data = phase <= 0.25?
                            4*phase*Short.MAX_VALUE:
                          phase <= 0.75?
                            (1 - 2*(phase - 0.25))*(Short.MAX_VALUE - Short.MIN_VALUE + 1) + Short.MIN_VALUE:
                            (1 - 4*(phase - 0.75))*Short.MIN_VALUE;
            buffer[i] = (short)(data*volume);
        }
    }   //genTriangleWave

    /**
     * This method applies the sound envelope to the sound data.
     *
     * @param buffer specifies the buffer that contains the wave data.
     * @param sampleRate specifies the sampling rate of tbe waveform.
     * @param attack specifies the attack time in seconds.
     * @param decay specifies the decay time in seconds.
     * @param sustain specifies the sustain level in proportion (0.0 to 1.0).
     * @param release specifies the release time in seconds.
     */
    protected void applySoundEnvelope(
            short[] buffer, int sampleRate, double attack, double decay, double sustain, double release)
    {
        final String funcName = "applySoundEnvelope";
        int index;
        int length = 0;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC,
                                "sampleRate=%d,attack=%.3f,decay=%.3f,sustain=%.3f,release=%.3f",
                                sampleRate, attack, decay, sustain, release);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC);
        }
        //
        // Apply attack.
        //
        index = 0;
        if (index < buffer.length)
        {
            length = Math.min((int)(sampleRate*attack), buffer.length - index);
            if (debugEnabled)
            {
                dbgTrace.traceInfo(funcName, "Attack=%.3f,index=%d,len=%d", attack, index, length);
            }
            scaleData(buffer, index, length, 1.0/length, 0.0);
        }
        //
        // Apply decay.
        //
        index += length;
        if (index < buffer.length)
        {
            length = Math.min((int)(sampleRate*decay), buffer.length - index);
            if (debugEnabled)
            {
                dbgTrace.traceInfo(funcName, "Decay=%.3f,index=%d,len=%d", decay, index, length);
            }
            scaleData(buffer, index, length, (sustain - 1.0)/length, 1.0);
        }
        //
        // Apply sustain.
        //
        index += length;
        if (index < buffer.length)
        {
            length = buffer.length - index - (int)(sampleRate*release);
            if (length < 0) length = 0;
            length = Math.min(length, buffer.length - index);
            if (debugEnabled)
            {
                dbgTrace.traceInfo(funcName, "Sustain=%.3f,index=%d,len=%d", sustain, index, length);
            }
            scaleData(buffer, index, length, 0.0, sustain);
        }
        //
        // Apply release.
        //
        index += length;
        if (index < buffer.length)
        {
            length = buffer.length - index;
            if (debugEnabled)
            {
                dbgTrace.traceInfo(funcName, "Release=%.3f,index=%d,len=%d", release, index, length);
            }
            scaleData(buffer, index, length, -sustain/length, sustain);
        }
    }   //applySoundEnvelope

    /**
     * This method scales the sound data using the envelope segment specified by the equation y = mx + b
     * where m is the slope and b is the zeroIntercept.
     *
     * @param buffer specifies the buffer contains the wave data.
     * @param startIndex specifies the start index of the envelope segment.
     * @param length specifies the length of the envelope segment.
     * @param slope specifies the slope of the envelope segment.
     * @param zeroIntercept specifies the zero intercept of the envelope segment.
     */
    private void scaleData(short[] buffer, int startIndex, int length, double slope, double zeroIntercept)
    {
        for (int i = 0; i < length; i++)
        {
            buffer[startIndex + i] *= slope*i + zeroIntercept;
        }
    }   //scaleData

}   //class TrcTone
