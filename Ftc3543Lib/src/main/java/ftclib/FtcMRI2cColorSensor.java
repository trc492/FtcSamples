/*
 * Copyright (c) 2016 Titan Robotics Club (http://www.titanrobotics.com)
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

import com.qualcomm.robotcore.hardware.HardwareMap;

import trclib.TrcDbgTrace;
import trclib.TrcI2cDevice;
import trclib.TrcSensor;
import trclib.TrcSensorDataSource;
import trclib.TrcUtil;

/**
 * This class implements the Modern Robotics Color Sensor extending FtcI2cDevice.
 * It provides the TrcI2cDevice.CompletionHandler interface to read the received data.
 */
public class FtcMRI2cColorSensor extends FtcMRI2cDevice implements TrcI2cDevice.CompletionHandler,
                                                                   TrcSensorDataSource
{
    private static final String moduleName = "FtcMRI2cColorSensor";
    private static final boolean debugEnabled = false;
    private TrcDbgTrace dbgTrace = null;

    public static final int DEF_I2CADDRESS          = 0x3c;

    //
    // I2C registers.
    //
    private static final int REG_COLOR_NUMBER       = 0x04;
    private static final int REG_RED                = 0x05;
    private static final int REG_GREEN              = 0x06;
    private static final int REG_BLUE               = 0x07;
    private static final int REG_WHITE              = 0x08;

    private static final int READ_START             = REG_COLOR_NUMBER;
    private static final int READ_END               = REG_WHITE;
    private static final int READ_LENGTH            = (READ_END - READ_START + 1);

    //
    // Commands.
    //
    private static final byte CMD_ENABLE_LED        = 0x00;
    private static final byte CMD_DISABLE_LED       = 0x01;
    private static final byte CMD_SET_50HZ_MODE     = 0x35;
    private static final byte CMD_SET_60HZ_MODE     = 0x36;
    private static final byte CMD_CAL_BLACKLEVEL    = 0x42;
    private static final byte CMD_CAL_WHITEBAL      = 0x43;

    //
    // Color Numbers.
    //
    private static final byte COLORNUM_BLACK        = 0;
    private static final byte COLORNUM_PURPLE       = 1;
    private static final byte COLORNUM_PURPLE_BLUE  = 2;
    private static final byte COLORNUM_BLUE         = 3;
    private static final byte COLORNUM_BLUE_GREEN   = 4;
    private static final byte COLORNUM_GREEN        = 5;
    private static final byte COLORNUM_GREEN_YELLOW = 6;
    private static final byte COLORNUM_YELLOW       = 7;
    private static final byte COLORNUM_YELLOW_ORANGE= 8;
    private static final byte COLORNUM_ORANGE       = 9;
    private static final byte COLORNUM_ORANGE_RED   = 10;
    private static final byte COLORNUM_RED          = 11;
    private static final byte COLORNUM_PINK         = 12;
    private static final byte COLORNUM_LIGHT_PINK   = 13;
    private static final byte COLORNUM_LIGHT_YELLOW = 14;
    private static final byte COLORNUM_LIGHT_BLUE   = 15;
    private static final byte COLORNUM_WHITE        = 16;

    private TrcSensor.SensorData colorNumber = new TrcSensor.SensorData(0.0, null);
    private TrcSensor.SensorData redValue = new TrcSensor.SensorData(0.0, null);
    private TrcSensor.SensorData greenValue = new TrcSensor.SensorData(0.0, null);
    private TrcSensor.SensorData blueValue = new TrcSensor.SensorData(0.0, null);
    private TrcSensor.SensorData whiteValue = new TrcSensor.SensorData(0.0, null);

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     * @param i2cAddress specifies the I2C address of the device.
     */
    public FtcMRI2cColorSensor(HardwareMap hardwareMap, String instanceName, int i2cAddress)
    {
        super(hardwareMap, instanceName, i2cAddress);

        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(
                    moduleName + "." + instanceName,
                    false,
                    TrcDbgTrace.TraceLevel.API,
                    TrcDbgTrace.MsgLevel.INFO);
        }

        read(READ_START, READ_LENGTH, this);
    }   //FtcMRI2cColorSensor

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param i2cAddress specifies the I2C address of the device.
     */
    public FtcMRI2cColorSensor(String instanceName, int i2cAddress)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, i2cAddress);
    }   //FtcMRI2cColorSensor

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FtcMRI2cColorSensor(String instanceName)
    {
        this(instanceName, DEF_I2CADDRESS);
    }   //FtcMRI2cColorSensor

    /**
     * This method turns on the internal LED to illuminate the target surface or turns off
     * the internal LED and reads from external light sources.
     *
     * @param enabled specifies true to turn on internal LED, false otherwise.
     */
    public void setLEDEnabled(boolean enabled)
    {
        sendByteCommand(REG_COMMAND, enabled? CMD_ENABLE_LED: CMD_DISABLE_LED);
    }   //setLEDEnabled

    /**
     * This method sets the operating frequency to 50Hz. This setting is saved in EEPROM.
     * This function is provided to enable the sampling to coincide with the normal flickering
     * associated with mains electrical A/C artificial lighting, and helps minimize signal
     * noise issues. Call this method when used in countries with 50Hz A/C electric current
     * frequency. When Frequency Mode set is complete, the LED will blink off briefly and then
     * the previous measurement mode will resume.
     */
    public void set50HzMode()
    {
        sendByteCommand(REG_COMMAND, CMD_SET_50HZ_MODE);
    }   //set50HzMode

    /**
     * This method sets the operating frequency to 60Hz. This setting is saved in EEPROM.
     * This function is provided to enable the sampling to coincide with the normal flickering
     * associated with mains electrical A/C artificial lighting, and helps minimize signal
     * noise issues. Call this method when used in countries with 60Hz A/C electric current
     * frequency. When Frequency Mode set is complete, the LED will blink off briefly and then
     * the previous measurement mode will resume.
     */
    public void set60HzMode()
    {
        sendByteCommand(REG_COMMAND, CMD_SET_60HZ_MODE);
    }   //set60HzMode

    /**
     * This method calibrates the black level. It runs 64 measurement cycles to obtain an
     * average value for each of the 3 color channels. The three values obtained are stored
     * in EEPROM and will subsequently be subtracted from all future measurements. When the
     * black level calibration is complete, the LED will blink off briefly and then the previous
     * measurement mode will resume with the command byte being set to 00H or 01H. During the
     * black level calibration, the sensor should be placed such that no surface is within 5 feet
     * (1.5m) of the sensor element.
     */
    public void calibrateBlackLevel()
    {
        sendByteCommand(REG_COMMAND, CMD_CAL_BLACKLEVEL);
    }   //calibrateBlackLevel

    /**
     * This method calibrates the white balance. It runs 64 measurement cycles to obtain an
     * average value for each of the 3 color channels. The values obtained are adjusted according
     * to the stored black level calibration values and stored in EEPROM. When the white balance
     * calibration is complete, the LED will blink off briefly and then previous measurement mode
     * will resume. During white balance calibration, the sensor must be placed approximately 2
     * inches (5cm) from a white target. This target must be as white as possible. At least 3
     * layers of high quality copy paper make a good white target.
     */
    public void calibrateWhiteBalance()
    {
        sendByteCommand(REG_COMMAND, CMD_CAL_WHITEBAL);
    }   //calibrateWhiteBalance

    /**
     * This method returns the color number.
     *
     * @return color number.
     */
    public TrcSensor.SensorData getColorNumber()
    {
        final String funcName = "getColorNumber";
        TrcSensor.SensorData data =
                new TrcSensor.SensorData(colorNumber.timestamp, colorNumber.value);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%d)", data.timestamp, (Integer)data.value);
        }

        return data;
    }   //getColorNumber

    /**
     * This method returns the red value.
     *
     * @return red value.
     */
    public TrcSensor.SensorData getRedValue()
    {
        final String funcName = "getRedValue";
        TrcSensor.SensorData data =
                new TrcSensor.SensorData(redValue.timestamp, redValue.value);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%d)", data.timestamp, (Integer)data.value);
        }

        return data;
    }   //getRedValue

    /**
     * This method returns the green value.
     *
     * @return green value.
     */
    public TrcSensor.SensorData getGreenValue()
    {
        final String funcName = "getGreenValue";
        TrcSensor.SensorData data =
                new TrcSensor.SensorData(greenValue.timestamp, greenValue.value);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%d)", data.timestamp, (Integer)data.value);
        }

        return data;
    }   //getGreenValue

    /**
     * This method returns the blue value.
     *
     * @return blue value.
     */
    public TrcSensor.SensorData getBlueValue()
    {
        final String funcName = "getBlueValue";
        TrcSensor.SensorData data =
                new TrcSensor.SensorData(blueValue.timestamp, blueValue.value);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%d)", data.timestamp, (Integer)data.value);
        }

        return data;
    }   //getBlueValue

    /**
     * This method returns the white value.
     *
     * @return white value.
     */
    public TrcSensor.SensorData getWhiteValue()
    {
        final String funcName = "getWhiteValue";
        TrcSensor.SensorData data =
                new TrcSensor.SensorData(whiteValue.timestamp, whiteValue.value);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%d)", data.timestamp, (Integer)data.value);
        }

        return data;
    }   //getWhiteValue

    //
    // Implements TrcI2cDevice.CompletionHandler interface.
    //

    /**
     * This method is called to notify the completion of the read operation.
     *
     * @param regAddress specifies the starting register address.
     * @param length specifies the number of bytes read.
     * @param timestamp specified the timestamp of the data retrieved.
     * @param data specifies the data byte array.
     * @param timedout specifies true if the operation was timed out, false otherwise.
     * @return true to repeat the operation, false otherwise.
     */
    @Override
    public boolean readCompletion(
            int regAddress, int length, double timestamp, byte[] data, boolean timedout)
    {
        final String funcName = "readCompletion";
        boolean repeat = false;

        if (regAddress == READ_START && length == READ_LENGTH)
        {
            if (!timedout)
            {
                //
                // Read these repeatedly.
                //
                colorNumber.timestamp = timestamp;
                colorNumber.value = TrcUtil.bytesToInt(data[REG_COLOR_NUMBER - READ_START]);

                redValue.timestamp = timestamp;
                redValue.value = TrcUtil.bytesToInt(data[REG_RED - READ_START]);

                greenValue.timestamp = timestamp;
                greenValue.value = TrcUtil.bytesToInt(data[REG_GREEN - READ_START]);

                blueValue.timestamp = timestamp;
                blueValue.value = TrcUtil.bytesToInt(data[REG_BLUE - READ_START]);

                whiteValue.timestamp = timestamp;
                whiteValue.value = TrcUtil.bytesToInt(data[REG_WHITE - READ_START]);
            }
            repeat = true;
        }
        else
        {
            repeat = super.readCompletion(regAddress, length, timestamp, data, timedout);
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.CALLBK,
                                "regAddr=%x,len=%d,timestamp=%.3f,timedout=%s",
                                regAddress, length, timestamp, Boolean.toString(timedout));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.CALLBK,
                               "=%s", Boolean.toString(repeat));
            dbgTrace.traceInfo(funcName, "%s(addr=%x,len=%d,time=%.3f,size=%d,timedout=%s)=%s",
                               funcName, regAddress, length, timestamp, data.length,
                               Boolean.toString(timedout), Boolean.toString(repeat));
        }

        return repeat;
    }   //readCompletion

    /**
     * This method is called to notify the completion of the write operation.
     *
     * @param regAddress specifies the starting register address.
     * @param length specifies the number of bytes read.
     * @param timedout specifies true if the operation was timed out, false otherwise.
     */
    @Override
    public void writeCompletion(int regAddress, int length, boolean timedout)
    {
    }   //writeCompletion

    //
    // Implements TrcSensorDataSource interface.
    //

    /**
     * This method returns the sensor data of the specified index.
     *
     * @param index specifies the data index.
     * @return sensor data of the specified index.
     */
    @Override
    public TrcSensor.SensorData getSensorData(int index)
    {
        final String funcName = "getSensorData";
        TrcSensor.SensorData data = null;

        switch (index)
        {
            case 0:
                data = getColorNumber();
                break;

            case 1:
                data = getRedValue();
                break;

            case 2:
                data = getGreenValue();
                break;

            case 3:
                data = getBlueValue();
                break;

            case 4:
                data = getWhiteValue();
                break;
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "index=%d", index);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(time=%.3f,value=%d)", data.timestamp, data.value);
        }

        return data;
    }   //getSensorData

}   //class FtcMRI2cColorSensor
