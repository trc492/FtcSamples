/*
 * Copyright (c) 2020 Titan Robotics Club (http://www.titanrobotics.com)
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

package org.firstinspires.ftc.teamcode;

import ftclib.FtcRevBlinkin;
import trclib.TrcHashMap;
import trclib.TrcRevBlinkin.LEDPattern;

/**
 * This class implements the LED indicator using a REV Blinkin LED Driver driving an RGB LED strip.
 */
public class LEDIndicator
{
    private static final LEDPattern offPattern = LEDPattern.SolidBlack;
    private FtcRevBlinkin blinkin;

    /**
     * Constructor: Create an instance of the object.
     */
    public LEDIndicator()
    {
        blinkin = new FtcRevBlinkin("blinkin");
        reset();
    }   //LEDIndicator

    /**
     * This method turns the LED strip off.
     */
    public void reset()
    {
        blinkin.setPattern(offPattern);
    }   //reset

    /**
     * This method sets the LED pattern associated with a detected vision target.
     *
     * @param targetName specifies the name of the detected vision target.
     */
    public void setDetectedTarget(String targetName)
    {
        blinkin.setPattern(VuforiaVision.getTargetLEDPattern(targetName));
    }   //setDetectedTarget

    /**
     * This method turns the LED strip ON (white) or OFF (black) and use it as the flash light for the vision
     * subsystem.
     *
     * @param lightOn specifies true to turn on the LED strip, false to turn off.
     */
    public void setFlashLightOn(boolean lightOn)
    {
        blinkin.setPattern(lightOn? LEDPattern.SolidWhite: LEDPattern.SolidBlack);
    }   //setFlashLightOn

}   //class LEDIndicator
