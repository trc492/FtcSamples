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

import java.util.Arrays;

/**
 * This class implements a platform independent I2C LED panel. This class is intended to be extended by a platform
 * dependent I2C LED panel which provides the abstract methods required by this class. This class provides the APIs
 * to assemble command requests and send them over to the panel asynchronously.
 */
public abstract class TrcI2cLEDPanel
{
    protected static final String moduleName = "TrcI2cLEDPanel";
    protected static final boolean debugEnabled = false;
    protected static final boolean tracingEnabled = false;
    protected static final boolean useGlobalTracer = false;
    protected static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    protected static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    protected TrcDbgTrace dbgTrace = null;

    private static final int I2C_BUFF_LEN = 32;

    /**
     * This method writes the data buffer to the device asynchronously.
     *
     * @param data specifies the data buffer.
     */
    public abstract void asyncWriteData(byte[] data);

    private final String instanceName;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public TrcI2cLEDPanel(final String instanceName)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
    }   //TrcI2cLEDPanel

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
     * This method sets the specified line in the LED panel with all the text info for displaying text on the panel.
     * Note that the (x, y) coordinates is rotation sensitive. If rotation is 0, the text orientation is normal
     * horizontal and (0, 0) corresponds to the upper left corner of the physical panel. If rotation is 2, the
     * text orientation is inverted horizontal and (0, 0) corresponds to the lower right corner of the physica
     * panel.
     *
     * @param index specifies the line index of the array.
     * @param x specifies the x coordinate of the upper left corner of the text rectangle.
     * @param y specifies the y coordinate of the upper left corner of the text rectangle.
     * @param fontColor specifies the font color for displaying the text.
     * @param orientation specifies the text orientation (0: normal horizontal, 1: clockwise vertical, 
     *                   2: inverted horizontal, 3: anti-clockwise vertical).
     * @param fontSize specifies the size of the font (1: 6x8, 2: 12x16, 3: 18x24, 4: 24x32).
     * @param scrollInc specifies the scroll increment (0: no scroll, 1: scroll to the right, -1: scroll to the left).
     * @param text specifies the text string to be displayed.
     */
    public void setTextLine(
        int index, int x, int y, int fontColor, int orientation, int fontSize, int scrollInc, String text)
    {
        sendCommand("setTextLine " + index + " " + x + " " + y + " " + fontColor + " " + orientation + " " +
                    fontSize + " " + scrollInc + " " + text);
    }   //setTextLine

    /**
     * This method clears the specified line in the lines array.
     *
     * @param index specifies the line index of the array.
     */
    public void clearTextLine(int index)
    {
        sendCommand("clearTextLine " + index);
    } //clearTextLine

    /**
     * This method clears all text lines in the lines array.
     */
    public void clearAllTextLines()
    {
        sendCommand("clearAllTextLines");
    } //clearAllTextLines

    /**
     * This method sets the Arduino loop delay. This effectively controls how fast the text will scroll.
     *
     * @param delay specifies the delay in msec.
     */
    public void setDelay(int delay)
    {
        sendCommand("setDelay " + delay);
    }   //setDelay

    /**
     * This method converts the specified RGB values into a 16-bit color value in 565 format (5-bit R, 6-bit G and
     * 5-bit B: RRRRRGGGGGGBBBBB).
     *
     * @param red specifies the red value.
     * @param green specifies the green value.
     * @param blue specifies the blue value.
     * @return 16-bit color value in 565 format.
     */
    public int color(int red, int green, int blue)
    {
        final String funcName = "color";
        int colorValue = ((red & 0xf8) << 8) | ((green & 0xfc) << 3) | (blue >> 3);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "red=%d,green=%d,blue=%d", red, green, blue);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=0x%x", colorValue);
        }

        return colorValue;
    }   //color

    /**
     * This method sends the command string to the I2C device. If the command string is longer than 32 bytes,
     * it will break down the command string into multiple I2C requests so that they can be reassembled on the
     * device side.
     *
     * @param command specifies the command string to be sent to the I2C device.
     */
    private void sendCommand(String command)
    {
        final String funcName = "sendCommand";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC, "command=%s", command);
        }

        command += "~";
        int cmdLen = command.length();

        if (debugEnabled)
        {
            dbgTrace.traceInfo(funcName, "sendCommand(%s)=%d", command, command.length());
        }
        for (int i = 0; i < cmdLen; )
        {
            int len = Math.min(cmdLen - i, I2C_BUFF_LEN);
            if (len > 0)
            {
                byte[] data = command.substring(i, i + len).getBytes();
                if (debugEnabled)
                {
                    dbgTrace.traceInfo(funcName, "asyncWrite%s=%d", Arrays.toString(data), data.length);
                }
                asyncWriteData(data);
                i += len;
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC);
        }
    }   //sendCommand

}   //class TrcI2cLEDPanel
