/*
 * Copyright (c) 2018 Titan Robotics Club (http://www.titanrobotics.com)
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

import java.util.Arrays;

/**
 * This class implements a platform independent REV Blinkin device. This class is intended to be extended by a
 * platform dependent device class which provides the abstract methods required by this class.
 */
public abstract class TrcRevBlinkin
{
    protected static final String moduleName = "TrcRevBlinkin";
    protected static final boolean debugEnabled = false;
    protected static final boolean tracingEnabled = false;
    protected static final boolean useGlobalTracer = false;
    protected static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    protected static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    protected TrcDbgTrace dbgTrace = null;

    /**
     * This method is provided by the platform dependent subclass that extends this class. It sets the LED pattern
     * value to the physical REV Blinkin device in a platform dependent way.
     *
     * @param value specifies the color pattern value.
     */
    public abstract void set(double value);

    /**
     * This method is provided by the platform dependent subclass that extends this class. It gets the current set
     * LED pattern value.
     *
     * @return currently set LED pattern value.
     */
    public abstract double get();

    public enum LEDPattern
    {
        FixedRainbowRainBow(-0.99),
        FixedRainbowParty(-0.97),
        FixedRainbowOcean(-0.95),
        FixedRainbowLave(-0.93),
        FixedRainbowForest(-0.91),
        FixedRainbowGlitter(-0.89),
        FixedConfetti(-0.87),
        FixedShotRed(-0.85),
        FixedShotBlue(-0.83),
        FixedShotWhite(-0.81),
        FixedSinelonRainbow(-0.79),
        FixedSinelonParty(-0.77),
        FixedSinelonOcean(-0.75),
        FixedSinelonLava(-0.73),
        FixedSinelonForest(-0.71),
        FixedBeatsPerMinuteRainbow(-0.69),
        FixedBeatsPerMinuteParty(-0.67),
        FixedBeatsPerMinuteOcean(-0.65),
        FixedBeatsPerMinuteLave(-0.63),
        FixedBeatsPerMinuteForest(-0.61),
        FixedFireMedium(-0.59),
        FixedFireLarge(-0.57),
        FixedTwinklesRainbow(-0.55),
        FixedTwinklesParty(-0.53),
        FixedTwinklesOcean(-0.51),
        FixedTwinklesLava(-0.49),
        FixedTwinklesForest(-0.47),
        FixedColorWavesRainbow(-0.45),
        FixedColorWavesParty(-0.43),
        FixedColorWavesOcean(-0.41),
        FixedColorWavesLava(-0.39),
        FixedColorWavesForest(-0.37),
        FixedLarsonScannerRed(-0.35),
        FixedLarsonScannerGray(-0.33),
        FixedLightChaseRed(-0.31),
        FixedLightChaseBlue(-0.29),
        FixedLightChaseGray(-0.27),
        FixedHeartbeatRed(-0.25),
        FixedHeartbeatBlue(-0.23),
        FixedHeartbeatWhite(-0.21),
        FixedHeartbeatGray(-0.19),
        FixedBreathRed(-0.17),
        FixedBreathBlue(-0.15),
        FixedBreathGray(-0.13),
        FixedStrobeRed(-0.11),
        FixedStrobeBlue(-0.09),
        FixedStrobeGold(-0.07),
        FixedStrobeWhite(-0.05),
        Color1EndToEndBlendToBlack(-0.03),
        Color1LarsonScanner(-0.01),
        Color1LightChase(0.01),
        Color1HeartbeatSlow(0.03),
        Color1HeartbeatMedium(0.05),
        Color1HeartbeatFast(0.07),
        Color1BreathSlow(0.09),
        Color1BreathFast(0.11),
        Color1Shot(0.13),
        Color1Strobe(0.15),
        Color2EndToEndBlendToBlack(0.17),
        Color2LarsonScanner(0.19),
        Color2LightChase(0.21),
        Color2HeartbeatSlow(0.23),
        Color2HeartbeatMedium(0.25),
        Color2HeartbeatFast(0.27),
        Color2BreathSlow(0.29),
        Color2BreathFast(0.31),
        Color2Shot(0.33),
        Color2Strobe(0.35),
        SparkleColor1On2(0.37),
        SparkleColor2On1(0.39),
        GradientColor1And2(0.41),
        BeatsPerMinuteColor1And2(0.43),
        EndToEndBlendColor1To2(0.45),
        EndToEndBlendColor1And2(0.47),
        Color1And2NoBlending(0.49),
        TwinklesColor1And2(0.51),
        ColorWavesColor1And2(0.53),
        SinelonColor1And2(0.55),
        SolidHotPink(0.57),
        SolidDarkRed(0.59),
        SolidRed(0.61),
        SolidRedOrange(0.63),
        SolidOrange(0.65),
        SolidGold(0.67),
        SolidYellow(0.69),
        SolidLawnGreen(0.71),
        SolidLime(0.73),
        SolidDarkGreen(0.75),
        SolidGreen(0.77),
        SolidBlueGreen(0.79),
        SolidAqua(0.81),
        SolidSkyBlue(0.83),
        SolidDarkBlue(0.85),
        SolidBlue(0.87),
        SolidBlueViolet(0.89),
        SolidViolet(0.91),
        SolidWhite(0.93),
        SolidGray(0.95),
        SolidDarkGray(0.97),
        SolidBlack(0.99);

        public double value;

        /**
         * Constructor: Creates an enum member.
         *
         * @param value specifies the value of the new member.
         */
        LEDPattern(double value)
        {
            this.value = value;
        }   //LEDPattern

        /**
         * This method looks up the enum member that matches the given value.
         *
         * @param value specifies the enum member value.
         * @return enum member with a matching value.
         */
        public static LEDPattern getPattern(double value)
        {
            for (LEDPattern p: LEDPattern.values())
            {
                if (value == p.value)
                {
                    return p;
                }
            }

            return null;
        }   //getPattern

    }   //enum LEDPattern

    /**
     * This class implements the LED pattern state.
     */
    private class PatternState
    {
        final LEDPattern pattern;
        boolean enabled;

        /**
         * Constructor: Create an instance of the object.
         *
         * @param pattern specifies the LED pattern.
         * @param enabled specifies the initial state of the pattern.
         */
        PatternState(LEDPattern pattern, boolean enabled)
        {
            this.pattern = pattern;
            this.enabled = enabled;
        }   //PatternState

        /**
         * Constructor: Create an instance of the object.
         *
         * @param pattern specifies the LED pattern.
         */
        PatternState(LEDPattern pattern)
        {
            this(pattern, false);
        }   //PatternState

    }   //class PatternState

    private final String instanceName;
    private PatternState[] patternPriorities = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public TrcRevBlinkin(final String instanceName)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
    }   //TrcRevBlinkin

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
     * This method sets the LED pattern priority list for operations that need it.
     *
     * @param ledPriorities specifies the pattern priority list or null to disregard the previously set list.
     */
    public void setPatternPriorities(LEDPattern[] ledPriorities)
    {
        final String funcName = "setPatternPriorities";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                "priorityList=%s", ledPriorities == null? "null": Arrays.toString(ledPriorities));
        }

        if (ledPriorities != null)
        {
            PatternState[] oldPriorities = patternPriorities;
            patternPriorities = new PatternState[ledPriorities.length];

            for (int i = 0; i < patternPriorities.length; i++)
            {
                patternPriorities[i] = new PatternState(ledPriorities[i]);
            }

            // If we had a previous priority list, make sure patterns persist
            if (oldPriorities != null)
            {
                for (PatternState patternState : oldPriorities)
                {
                    if (patternState.enabled)
                    {
                        // This will silently fail if this pattern is not in the priority list
                        setPatternState(patternState.pattern, true);
                    }
                }
            }
            updateLED();
        }
        else
        {
            patternPriorities = null;
            setPattern(LEDPattern.SolidBlack);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setPatternPriorities

    /**
     * This method returns the currently set LED pattern
     *
     * @return currently set LED pattern.
     */
    public LEDPattern getPattern()
    {
        final String funcName = "getPattern";
        LEDPattern pattern = LEDPattern.getPattern(get());

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", pattern);
        }

        return pattern;
    }   //getPattern

    /**
     * This method sets the color pattern of the LED strip regardless on what is already showing. It means it
     * does not respect the priority list.
     *
     * @param pattern specifies the color pattern to be set.
     */
    public void setPattern(LEDPattern pattern)
    {
        final String funcName = "setPattern";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "pattern=%s", pattern);
        }

        set(pattern.value);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setPattern

    /**
     * This method enables/disables the LED pattern in the priority list.
     *
     * @param pattern specifies the LED pattern in the priority list.
     * @param enabled specifies true to turn the pattern ON, false to turn it OFF.
     */
    public void setPatternState(LEDPattern pattern, boolean enabled)
    {
        final String funcName = "setPatternState";
        int index = getPatternPriority(pattern);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "pattern=%s,state=%s,index=%d",
                pattern, enabled, index);
        }

        if (index != -1)
        {
            patternPriorities[index].enabled = enabled;
            updateLED();
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setPatternState

    /**
     * This method returns the LED pattern state if it is in the priority list. If the pattern is not in the list,
     * it returns false.
     *
     * @param pattern specifies the LED pattern in the priority list.
     * @return true if the LED pattern is ON, false if it is OFF.
     */
    public boolean getPatternState(LEDPattern pattern)
    {
        final String funcName = "getPatternState";
        boolean state = false;
        int index = getPatternPriority(pattern);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "pattern=%s,index=%d",
                pattern, index);
        }

        if (index != -1)
        {
            state = patternPriorities[index].enabled;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", state);
        }

        return state;
    }   //getPatternState

    /**
     * This method resets all pattern states in the pattern priority list and turns off the LED strip.
     */
    public void resetAllPatternStates()
    {
        final String funcName = "resetAllPatternStates";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (patternPriorities != null)
        {
            for (PatternState state: patternPriorities)
            {
                state.enabled = false;
            }

            setPattern(LEDPattern.SolidBlack);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //resetAllPatternStates

    /**
     * This method searches the given pattern priorities array for the given pattern. If found, its index is
     * the priority and will be returned. If the pattern is not found in the array, -1 will be return which also
     * means the lowest priority.
     *
     * @param pattern specifies the LED pattern to be searched in the pattern priorities array.
     * @return the pattern priority if found, -1 if not found.
     */
    public int getPatternPriority(LEDPattern pattern)
    {
        final String funcName = "getPatternPriority";
        int priority = -1;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "pattern=%s", pattern);
        }

        if (patternPriorities != null)
        {
            for (int i = 0; i < patternPriorities.length; i++)
            {
                if (pattern == patternPriorities[i].pattern)
                {
                    priority = i;
                    break;
                }
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%d", priority);
        }

        return priority;
    }   //getPatternPriority

    /**
     * This method is called to update the LED pattern according to the patternPriorities list. It will turn on the
     * highest priority pattern if enabled. If none of the patterns in the priority list is enabled, it will turn
     * off the LED strip.
     */
    private void updateLED()
    {
        final String funcName = "updateLED";
        LEDPattern pattern = LEDPattern.SolidBlack;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC);
        }

        for (int i = patternPriorities.length - 1; i >= 0; i--)
        {
            if (patternPriorities[i].enabled)
            {
                pattern = patternPriorities[i].pattern;
                break;
            }
        }

        setPattern(pattern);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC, "! (pattern=%s)", pattern);
        }
    }   //updateLED

}   //class TrcRevBlinkin
