/*
 * Copyright (c) 2017 Titan Robotics Club (http://www.titanrobotics.com)
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

import java.io.UnsupportedEncodingException;

/**
 * This class implements a platform independent Emic2 text to speech device that is connected to a Serial Port.
 * This class should be extended by a platform dependent Emic2 device class that provides the asynchronous access
 * to the serial port the device is connected to.
 */
public abstract class TrcEmic2TextToSpeech implements TrcNotifier.Receiver
{
    protected static final String moduleName = "TrcEmic2TextToSpeech";
    protected static final TrcDbgTrace globalTracer = TrcDbgTrace.getGlobalTracer();
    protected static final boolean debugEnabled = false;
    protected static final boolean tracingEnabled = false;
    protected static final boolean useGlobalTracer = false;
    protected static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    protected static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    protected TrcDbgTrace dbgTrace = null;

    public static final int MIN_VOLUME = -48;
    public static final int MAX_VOLUME = 18;

    public enum Voice
    {
        PerfectPaul(0),
        HugeHarry(1),
        BeautifulBetty(2),
        UppityUrsula(3),
        DoctorDennis(4),
        KitTheKid(5),
        FrailFrank(6),
        RoughRita(7),
        WhisperingWendy(8);

        public int value;

        Voice(int value)
        {
            this.value = value;
        }
    }   //enum Voice

    public enum DemoMsg
    {
        Speaking(0),
        Singing(1),
        Spanish(2);

        public int value;

        DemoMsg(int value)
        {
            this.value = value;
        }
    }   //enum DemoMsg

    public enum Language
    {
        USEnglish(0),
        CastilianSpanish(1),
        LatinSpanish(2);

        public int value;

        Language(int value)
        {
            this.value = value;
        }
    }   //enum Language

    public enum Parser
    {
        DECTalk(0),
        Epson(1);

        public int value;

        Parser(int value)
        {
            this.value = value;
        }
    }   //enum Parser

    /**
     * This method issues an asynchronous read of a text string from the device.
     *
     * @param requestTag specifies the tag to identify the request. Can be null if none was provided.
     */
    public abstract void asyncReadString(RequestTag requestTag);

    /**
     * This method writes the string to the device asynchronously.
     *
     * @param text specifies the text string to be written to the device.
     * @param preemptive specifies true for immediate write without queuing, false otherwise.
     */
    public abstract void asyncWriteString(String text, boolean preemptive);

    /**
     * This is used identify the request type.
     */
    public static enum RequestTag
    {
        PROMPT,
        CONFIG_MSG,
        VERSION_MSG,
        HELP_MSG
    }   //enum RequestTag

    private final String instanceName;
    private volatile String configMsg = null;
    private volatile String versionMsg = null;
    private volatile String helpMsg = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public TrcEmic2TextToSpeech(final String instanceName)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer? globalTracer:
                    new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
    }   //TrcEmic2TextToSpeech

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
     * This method is called to start the device in which it would send a read request for the prompt string.
     */
    public void start()
    {
        asyncReadString(RequestTag.PROMPT);
    }   //start

    /**
     * This method speaks the specified message.
     *
     * @param msg specifies the message to be spoken.
     */
    public void speak(String msg)
    {
        final String funcName = "speak";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "msg=%s", msg);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        asyncWriteString("S" + msg + "\n", false);
        asyncReadString(RequestTag.PROMPT);
    }   //speak

    /**
     * This method plays the specified demo message.
     *
     * @param msg specifies the demo message.
     */
    public void playDemoMessage(DemoMsg msg)
    {
        final String funcName = "playDemoMessage";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "msg=%s", msg);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        asyncWriteString(String.format("D%d\n", msg.value), false);
        asyncReadString(RequestTag.PROMPT);
    }   //playDemoMessage

    /**
     * This method aborts the spoken sentence in progress.
     */
    public void stopPlayback()
    {
        final String funcName = "stopPlayback";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        asyncWriteString("X\n", true);
    }   //stopPlayback

    /**
     * This method is called to pause/resume the spoken sentence in progress.
     */
    public void togglePlayback()
    {
        final String funcName = "togglePlayback";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        asyncWriteString("Z\n", true);
    }   //togglePlayback

    /**
     * This method selects the spoken voice.
     *
     * @param voice specifies the voice to be used.
     */
    public void selectVoice(Voice voice)
    {
        final String funcName = "selectVoice";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "voice=%s", voice);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        asyncWriteString(String.format("N%d\n", voice.value), false);
        asyncReadString(RequestTag.PROMPT);
        configMsg = null;
    }   //selectVoice

    /**
     * This method sets the speaking volume. Valid value is between -48 to 18.
     *
     * @param vol specifies the speaking volume.
     */
    public void setVolume(int vol)
    {
        final String funcName = "setVolume";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "vol=%d", vol);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (vol < MIN_VOLUME)
            vol = MIN_VOLUME;
        else if (vol > MAX_VOLUME)
            vol = MAX_VOLUME;

        asyncWriteString(String.format("V%d\n", vol), false);
        asyncReadString(RequestTag.PROMPT);
        configMsg = null;
    }   //setVolume

    /**
     * This method sets the speaking volume. Valid value is between 0 and 1.0. 0 for mute and 1.0 for full volume.
     *
     * @param vol specifies the speaking volume.
     */
    public void setVolume(double vol)
    {
        vol = TrcUtil.clipRange(vol);
        setVolume((int)((MAX_VOLUME - MIN_VOLUME)*vol + MIN_VOLUME));
    }   //setVolume

    /**
     * This method sets the speaking rate in words per minute.
     *
     * @param rate specifies the speaking rate.
     */
    public void setSpeakingRate(int rate)
    {
        final String funcName = "setSpeakingRate";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "rate=%d", rate);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (rate < 75 || rate > 600)
        {
            throw new IllegalArgumentException("Invalid speaking rate, must be between 75 to 600 words/min.");
        }

        asyncWriteString(String.format("W%d\n", rate), false);
        asyncReadString(RequestTag.PROMPT);
        configMsg = null;
    }   //setSpeakingRate

    /**
     * This method sets the spoken language.
     *
     * @param lang specifies the spoken language.
     */
    public void setLanguage(Language lang)
    {
        final String funcName = "setLanguage";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "lang=%s", lang);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        asyncWriteString(String.format("L%d\n", lang.value), false);
        asyncReadString(RequestTag.PROMPT);
        configMsg = null;
    }   //setLanguage

    /**
     * This method selects the parser that parses the sentence.
     *
     * @param parser specifies the parser to use.
     */
    public void selectParser(Parser parser)
    {
        final String funcName = "selectParser";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "parser=%s", parser);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        asyncWriteString(String.format("P%d\n", parser.value), false);
        asyncReadString(RequestTag.PROMPT);
        configMsg = null;
    }   //selectParser

    /**
     * This method sets the text-to-speech back to default configuration.
     */
    public void revertDefaultConfig()
    {
        final String funcName = "revertDefaultConfig";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        asyncWriteString("R\n", false);
        asyncReadString(RequestTag.PROMPT);
        configMsg = null;
    }   //revertDefaultConfig

    /**
     * This method returns the current text-to-speech configuration.
     *
     * @param wait specifies true for synchronous access.
     * @return current configuration string if wait is true, null otherwise.
     */
    public String getCurrentConfig(boolean wait)
    {
        if (configMsg == null)
        {
            asyncWriteString("C\n", false);
            asyncReadString(RequestTag.CONFIG_MSG);
            asyncReadString(RequestTag.PROMPT);
        }

        if (wait)
        {
            while (configMsg == null)
            {
                Thread.yield();
            }
        }

        return configMsg;
    }   //getCurrentConfig

    /**
     * This method returns the firmware version.
     *
     * @param wait specifies true for synchronous access.
     * @return firmware version string if wait is true, null otherwise.
     */
    public String getVersion(boolean wait)
    {
        if (versionMsg == null)
        {
            asyncWriteString("V\n", false);
            asyncReadString(RequestTag.VERSION_MSG);
            asyncReadString(RequestTag.PROMPT);
        }

        if (wait)
        {
            while (versionMsg == null)
            {
                Thread.yield();
            }
        }

        return versionMsg;
    }   //getVersion

    /**
     * This method returns the help message.
     *
     * @param wait specifies true for synchronous access.
     * @return help message string if wait is true, null otherwise.
     */
    public String getHelpMessage(boolean wait)
    {
        if (helpMsg == null)
        {
            asyncWriteString("H\n", false);
            asyncReadString(RequestTag.HELP_MSG);
            asyncReadString(RequestTag.PROMPT);
        }

        if (wait)
        {
            while (helpMsg == null)
            {
                Thread.yield();
            }
        }

        return helpMsg;
    }   //getHelpMessage

    //
    // Implements TrcNotifier.Receiver interface.
    //

    /**
     * This method is called when the read request is completed.
     *
     * @param context specifies the read request.
     */
    @Override
    public void notify(Object context)
    {
        final String funcName = "notify";
        TrcSerialBusDevice.Request request = (TrcSerialBusDevice.Request) context;
        String reply = null;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.CALLBK, "request=%s", request);
        }

        if (request.readRequest && !request.error && !request.canceled && request.buffer != null)
        {
            try
            {
                reply = new String(request.buffer, "US-ASCII");
                if (debugEnabled)
                {
                    dbgTrace.traceInfo(funcName, "reply=<%s>", reply);
                }
            }
            catch (UnsupportedEncodingException e)
            {
                globalTracer.traceErr(funcName, "Unsupported Encoding: %s", e.getMessage());
                e.printStackTrace();
            }
        }

        if (reply != null)
        {
            switch ((RequestTag)request.requestCtxt)
            {
                case PROMPT:
                    if (reply.equals("."))
                    {
                        //
                        // There was a pause/unpause command, retry the prompt request.
                        //
                        asyncReadString(RequestTag.PROMPT);
                    }
                    break;

                case CONFIG_MSG:
                    configMsg = reply;
                    break;

                case VERSION_MSG:
                    versionMsg = reply;
                    break;

                case HELP_MSG:
                    helpMsg = reply;
                    break;
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.CALLBK);
        }
    }   //notify

}   //class FrcEmic2TextToSpeech
