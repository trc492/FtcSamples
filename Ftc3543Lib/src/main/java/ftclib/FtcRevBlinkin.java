/*
 * Copyright (c) 2019 Titan Robotics Club (http://www.titanrobotics.com)
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

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import trclib.TrcDbgTrace;
import trclib.TrcHashMap;
import trclib.TrcRevBlinkin;

/**
 * This class implements a platform dependent REV Blinkin device. It provides platform dependent methods that
 * gets/sets the color pattern from/to the device.
 */
public class FtcRevBlinkin extends TrcRevBlinkin
{
    private static final TrcHashMap<LEDPattern, RevBlinkinLedDriver.BlinkinPattern> patternMap =
            new TrcHashMap<LEDPattern, RevBlinkinLedDriver.BlinkinPattern>()
            /*
             * Fixed Palette Pattern
             */
            .add(LEDPattern.FixedRainbowRainBow, RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE)
            .add(LEDPattern.FixedRainbowParty, RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE)
            .add(LEDPattern.FixedRainbowOcean, RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE)
            .add(LEDPattern.FixedRainbowLave, RevBlinkinLedDriver.BlinkinPattern.RAINBOW_LAVA_PALETTE)
            .add(LEDPattern.FixedRainbowForest, RevBlinkinLedDriver.BlinkinPattern.RAINBOW_FOREST_PALETTE)
            .add(LEDPattern.FixedRainbowGlitter, RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER)
            .add(LEDPattern.FixedConfetti, RevBlinkinLedDriver.BlinkinPattern.CONFETTI)
            .add(LEDPattern.FixedShotRed, RevBlinkinLedDriver.BlinkinPattern.SHOT_RED)
            .add(LEDPattern.FixedShotBlue, RevBlinkinLedDriver.BlinkinPattern.SHOT_BLUE)
            .add(LEDPattern.FixedShotWhite, RevBlinkinLedDriver.BlinkinPattern.SHOT_WHITE)
            .add(LEDPattern.FixedSinelonRainbow, RevBlinkinLedDriver.BlinkinPattern.SINELON_RAINBOW_PALETTE)
            .add(LEDPattern.FixedSinelonParty, RevBlinkinLedDriver.BlinkinPattern.SINELON_PARTY_PALETTE)
            .add(LEDPattern.FixedSinelonOcean, RevBlinkinLedDriver.BlinkinPattern.SINELON_OCEAN_PALETTE)
            .add(LEDPattern.FixedSinelonLava, RevBlinkinLedDriver.BlinkinPattern.SINELON_LAVA_PALETTE)
            .add(LEDPattern.FixedSinelonForest, RevBlinkinLedDriver.BlinkinPattern.SINELON_FOREST_PALETTE)
            .add(LEDPattern.FixedBeatsPerMinuteRainbow, RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE)
            .add(LEDPattern.FixedBeatsPerMinuteParty, RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_PARTY_PALETTE)
            .add(LEDPattern.FixedBeatsPerMinuteOcean, RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_OCEAN_PALETTE)
            .add(LEDPattern.FixedBeatsPerMinuteLave, RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_LAVA_PALETTE)
            .add(LEDPattern.FixedBeatsPerMinuteForest, RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_FOREST_PALETTE)
            .add(LEDPattern.FixedFireMedium, RevBlinkinLedDriver.BlinkinPattern.FIRE_MEDIUM)
            .add(LEDPattern.FixedFireLarge, RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE)
            .add(LEDPattern.FixedTwinklesRainbow, RevBlinkinLedDriver.BlinkinPattern.TWINKLES_RAINBOW_PALETTE)
            .add(LEDPattern.FixedTwinklesParty, RevBlinkinLedDriver.BlinkinPattern.TWINKLES_PARTY_PALETTE)
            .add(LEDPattern.FixedTwinklesOcean, RevBlinkinLedDriver.BlinkinPattern.TWINKLES_OCEAN_PALETTE)
            .add(LEDPattern.FixedTwinklesLava, RevBlinkinLedDriver.BlinkinPattern.TWINKLES_LAVA_PALETTE)
            .add(LEDPattern.FixedTwinklesForest, RevBlinkinLedDriver.BlinkinPattern.TWINKLES_FOREST_PALETTE)
            .add(LEDPattern.FixedColorWavesRainbow, RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE)
            .add(LEDPattern.FixedColorWavesParty, RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_PARTY_PALETTE)
            .add(LEDPattern.FixedColorWavesOcean, RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE)
            .add(LEDPattern.FixedColorWavesLava, RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE)
            .add(LEDPattern.FixedColorWavesForest, RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE)
            .add(LEDPattern.FixedLarsonScannerRed, RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_RED)
            .add(LEDPattern.FixedLarsonScannerGray, RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_GRAY)
            .add(LEDPattern.FixedLightChaseRed, RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED)
            .add(LEDPattern.FixedLightChaseBlue, RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE)
            .add(LEDPattern.FixedLightChaseGray, RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_GRAY)
            .add(LEDPattern.FixedHeartbeatRed, RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED)
            .add(LEDPattern.FixedHeartbeatBlue, RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE)
            .add(LEDPattern.FixedHeartbeatWhite, RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE)
            .add(LEDPattern.FixedHeartbeatGray, RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_GRAY)
            .add(LEDPattern.FixedBreathRed, RevBlinkinLedDriver.BlinkinPattern.BREATH_RED)
            .add(LEDPattern.FixedBreathBlue, RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE)
            .add(LEDPattern.FixedBreathGray, RevBlinkinLedDriver.BlinkinPattern.BREATH_GRAY)
            .add(LEDPattern.FixedStrobeRed, RevBlinkinLedDriver.BlinkinPattern.STROBE_RED)
            .add(LEDPattern.FixedStrobeBlue, RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE)
            .add(LEDPattern.FixedStrobeGold, RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD)
            .add(LEDPattern.FixedStrobeWhite, RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE)
            /*
             * CP1: Color 1 Pattern
             */
            .add(LEDPattern.Color1EndToEndBlendToBlack, RevBlinkinLedDriver.BlinkinPattern.CP1_END_TO_END_BLEND_TO_BLACK)
            .add(LEDPattern.Color1LarsonScanner, RevBlinkinLedDriver.BlinkinPattern.CP1_LARSON_SCANNER)
            .add(LEDPattern.Color1LightChase, RevBlinkinLedDriver.BlinkinPattern.CP1_LIGHT_CHASE)
            .add(LEDPattern.Color1HeartbeatSlow, RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_SLOW)
            .add(LEDPattern.Color1HeartbeatMedium, RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_MEDIUM)
            .add(LEDPattern.Color1HeartbeatFast, RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_FAST)
            .add(LEDPattern.Color1BreathSlow, RevBlinkinLedDriver.BlinkinPattern.CP1_BREATH_SLOW)
            .add(LEDPattern.Color1BreathFast, RevBlinkinLedDriver.BlinkinPattern.CP1_BREATH_FAST)
            .add(LEDPattern.Color1Shot, RevBlinkinLedDriver.BlinkinPattern.CP1_SHOT)
            .add(LEDPattern.Color1Strobe, RevBlinkinLedDriver.BlinkinPattern.CP1_STROBE)
            /*
             * CP2: Color 2 Pattern
             */
            .add(LEDPattern.Color2EndToEndBlendToBlack, RevBlinkinLedDriver.BlinkinPattern.CP2_END_TO_END_BLEND_TO_BLACK)
            .add(LEDPattern.Color2LarsonScanner, RevBlinkinLedDriver.BlinkinPattern.CP2_LARSON_SCANNER)
            .add(LEDPattern.Color2LightChase, RevBlinkinLedDriver.BlinkinPattern.CP2_LIGHT_CHASE)
            .add(LEDPattern.Color2HeartbeatSlow, RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_SLOW)
            .add(LEDPattern.Color2HeartbeatMedium, RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_MEDIUM)
            .add(LEDPattern.Color2HeartbeatFast, RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_FAST)
            .add(LEDPattern.Color2BreathSlow, RevBlinkinLedDriver.BlinkinPattern.CP2_BREATH_SLOW)
            .add(LEDPattern.Color2BreathFast, RevBlinkinLedDriver.BlinkinPattern.CP2_BREATH_FAST)
            .add(LEDPattern.Color2Shot, RevBlinkinLedDriver.BlinkinPattern.CP2_SHOT)
            .add(LEDPattern.Color2Strobe, RevBlinkinLedDriver.BlinkinPattern.CP2_STROBE)
            /*
             * CP1_2: Color 1 and 2 Pattern
             */
            .add(LEDPattern.SparkleColor1On2, RevBlinkinLedDriver.BlinkinPattern.CP1_2_SPARKLE_1_ON_2)
            .add(LEDPattern.SparkleColor2On1, RevBlinkinLedDriver.BlinkinPattern.CP1_2_SPARKLE_2_ON_1)
            .add(LEDPattern.GradientColor1And2, RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_GRADIENT)
            .add(LEDPattern.BeatsPerMinuteColor1And2, RevBlinkinLedDriver.BlinkinPattern.CP1_2_BEATS_PER_MINUTE)
            .add(LEDPattern.EndToEndBlendColor1To2, RevBlinkinLedDriver.BlinkinPattern.CP1_2_END_TO_END_BLEND_1_TO_2)
            .add(LEDPattern.EndToEndBlendColor1And2, RevBlinkinLedDriver.BlinkinPattern.CP1_2_END_TO_END_BLEND)
            .add(LEDPattern.Color1And2NoBlending, RevBlinkinLedDriver.BlinkinPattern.CP1_2_NO_BLENDING)
            .add(LEDPattern.TwinklesColor1And2, RevBlinkinLedDriver.BlinkinPattern.CP1_2_TWINKLES)
            .add(LEDPattern.ColorWavesColor1And2, RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_WAVES)
            .add(LEDPattern.SinelonColor1And2, RevBlinkinLedDriver.BlinkinPattern.CP1_2_SINELON)
            /*
             * Solid color
             */
            .add(LEDPattern.SolidHotPink, RevBlinkinLedDriver.BlinkinPattern.HOT_PINK)
            .add(LEDPattern.SolidDarkRed, RevBlinkinLedDriver.BlinkinPattern.DARK_RED)
            .add(LEDPattern.SolidRed, RevBlinkinLedDriver.BlinkinPattern.RED)
            .add(LEDPattern.SolidRedOrange, RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE)
            .add(LEDPattern.SolidOrange, RevBlinkinLedDriver.BlinkinPattern.ORANGE)
            .add(LEDPattern.SolidGold, RevBlinkinLedDriver.BlinkinPattern.GOLD)
            .add(LEDPattern.SolidYellow, RevBlinkinLedDriver.BlinkinPattern.YELLOW)
            .add(LEDPattern.SolidLawnGreen, RevBlinkinLedDriver.BlinkinPattern.LAWN_GREEN)
            .add(LEDPattern.SolidLime, RevBlinkinLedDriver.BlinkinPattern.LIME)
            .add(LEDPattern.SolidDarkGreen, RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN)
            .add(LEDPattern.SolidGreen, RevBlinkinLedDriver.BlinkinPattern.GREEN)
            .add(LEDPattern.SolidBlueGreen, RevBlinkinLedDriver.BlinkinPattern.BLUE_GREEN)
            .add(LEDPattern.SolidAqua, RevBlinkinLedDriver.BlinkinPattern.AQUA)
            .add(LEDPattern.SolidSkyBlue, RevBlinkinLedDriver.BlinkinPattern.SKY_BLUE)
            .add(LEDPattern.SolidDarkBlue, RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE)
            .add(LEDPattern.SolidBlue, RevBlinkinLedDriver.BlinkinPattern.BLUE)
            .add(LEDPattern.SolidBlueViolet, RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET)
            .add(LEDPattern.SolidViolet, RevBlinkinLedDriver.BlinkinPattern.VIOLET)
            .add(LEDPattern.SolidWhite, RevBlinkinLedDriver.BlinkinPattern.WHITE)
            .add(LEDPattern.SolidGray, RevBlinkinLedDriver.BlinkinPattern.GRAY)
            .add(LEDPattern.SolidDarkGray, RevBlinkinLedDriver.BlinkinPattern.DARK_GRAY)
            .add(LEDPattern.SolidBlack, RevBlinkinLedDriver.BlinkinPattern.BLACK);

    private final RevBlinkinLedDriver blinkinLedDriver;
    private LEDPattern currPattern = LEDPattern.SolidBlack;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     */
    public FtcRevBlinkin(HardwareMap hardwareMap, String instanceName)
    {
        super(instanceName);

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, instanceName);
        setPattern(currPattern);
    }   //FtcRevBlinkin

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FtcRevBlinkin(String instanceName)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName);
    }   //FtcRevBlinkin

    //
    // Implements TrcRevBlinkin abstract methods.
    //

    /**
     * This method gets the current set LED pattern.
     *
     * @return currently set LED pattern.
     */
    @Override
    public LEDPattern getPattern()
    {
        final String funcName = "getPattern";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", currPattern);
        }

        return currPattern;
    }   //getPattern

    /**
     * This method sets the LED pattern to the physical REV Blinkin device.
     *
     * @param pattern specifies the color pattern.
     */
    @Override
    public void setPattern(LEDPattern pattern)
    {
        final String funcName = "setPattern";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "pattern=%s", pattern);
        }

        currPattern = pattern;
        blinkinLedDriver.setPattern(patternMap.get(pattern));

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setPattern

}   //class FtcRevBlinkin
