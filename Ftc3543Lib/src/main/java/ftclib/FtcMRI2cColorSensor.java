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

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import trclib.TrcDbgTrace;
import trclib.TrcSensor;
import trclib.TrcUtil;

/**
 * This class implements the Modern Robotics Color Sensor extending FtcMRI2cDevice that implements the common
 * features of all Modern Robotics I2C devices.
 */
public class FtcMRI2cColorSensor extends FtcMRI2cDevice implements TrcSensor.DataSource<FtcMRI2cColorSensor.DataType>
{
    private static final String moduleName = "FtcMRI2cColorSensor";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    public enum DataType
    {
        COLOR_NUMBER,
        RED,
        GREEN,
        BLUE,
        WHITE
    }   //enum DataType

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

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     * @param i2cAddress specifies the I2C address of the device.
     * @param addressIs7Bit specifies true if the I2C address is a 7-bit address, false if it is 8-bit.
     */
    public FtcMRI2cColorSensor(HardwareMap hardwareMap, String instanceName, int i2cAddress, boolean addressIs7Bit)
    {
        super(hardwareMap, instanceName, i2cAddress, addressIs7Bit);

        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        deviceSynch.setDeviceInfo(HardwareDevice.Manufacturer.ModernRobotics, "MR Color Sensor");
        deviceSynch.setBufferedReadWindow(READ_START, READ_LENGTH, I2cDeviceSynch.ReadMode.REPEAT, READ_LENGTH);
    }   //FtcMRI2cColorSensor

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param i2cAddress specifies the I2C address of the device.
     * @param addressIs7Bit specifies true if the I2C address is a 7-bit address, false if it is 8-bit.
     */
    public FtcMRI2cColorSensor(String instanceName, int i2cAddress, boolean addressIs7Bit)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, i2cAddress, addressIs7Bit);
    }   //FtcMRI2cColorSensor

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FtcMRI2cColorSensor(String instanceName)
    {
        this(instanceName, DEF_I2CADDRESS, false);
    }   //FtcMRI2cColorSensor

    /**
     * This method turns on the internal LED to illuminate the target surface or turns off the internal LED and reads
     * from external light sources.
     *
     * @param enabled specifies true to turn on internal LED, false otherwise.
     */
    public void setLEDEnabled(boolean enabled)
    {
        sendByteCommand(REG_COMMAND, enabled? CMD_ENABLE_LED: CMD_DISABLE_LED, false);
    }   //setLEDEnabled

    /**
     * This method sets the operating frequency to 50Hz. This setting is saved in EEPROM. This function is provided
     * to enable the sampling to coincide with the normal flickering associated with mains electrical A/C artificial
     * lighting, and helps minimize signal noise issues. Call this method when used in countries with 50Hz A/C
     * electric current frequency. When Frequency Mode set is complete, the LED will blink off briefly and then
     * the previous measurement mode will resume.
     */
    public void set50HzMode()
    {
        sendByteCommand(REG_COMMAND, CMD_SET_50HZ_MODE, false);
    }   //set50HzMode

    /**
     * This method sets the operating frequency to 60Hz. This setting is saved in EEPROM. This function is provided
     * to enable the sampling to coincide with the normal flickering associated with mains electrical A/C artificial
     * lighting, and helps minimize signal noise issues. Call this method when used in countries with 60Hz A/C
     * electric current frequency. When Frequency Mode set is complete, the LED will blink off briefly and then
     * the previous measurement mode will resume.
     */
    public void set60HzMode()
    {
        sendByteCommand(REG_COMMAND, CMD_SET_60HZ_MODE, false);
    }   //set60HzMode

    /**
     * This method calibrates the black level. It runs 64 measurement cycles to obtain an average value for each of
     * the 3 color channels. The three values obtained are stored in EEPROM and will subsequently be subtracted from
     * all future measurements. When the black level calibration is complete, the LED will blink off briefly and then
     * the previous measurement mode will resume with the command byte being set to 00H or 01H. During the black level
     * calibration, the sensor should be placed such that no surface is within 5 feet (1.5m) of the sensor element.
     */
    public void calibrateBlackLevel()
    {
        sendByteCommand(REG_COMMAND, CMD_CAL_BLACKLEVEL, false);
    }   //calibrateBlackLevel

    /**
     * This method calibrates the white balance. It runs 64 measurement cycles to obtain an average value for each
     * of the 3 color channels. The values obtained are adjusted according to the stored black level calibration
     * values and stored in EEPROM. When the white balance calibration is complete, the LED will blink off briefly
     * and then previous measurement mode will resume. During white balance calibration, the sensor must be placed
     * approximately 2 inches (5cm) from a white target. This target must be as white as possible. At least 3 layers
     * of high quality copy paper make a good white target.
     */
    public void calibrateWhiteBalance()
    {
        sendByteCommand(REG_COMMAND, CMD_CAL_WHITEBAL, false);
    }   //calibrateWhiteBalance

    /**
     * This method returns the color number.
     *
     * @return color number.
     */
    public TrcSensor.SensorData<Double> getColorNumber()
    {
        final String funcName = "getColorNumber";
        byte[] regData = readData(READ_START, READ_LENGTH);
        TrcSensor.SensorData<Double> data = new TrcSensor.SensorData<>(
                TrcUtil.getCurrentTime(), (double)TrcUtil.bytesToInt(regData[REG_COLOR_NUMBER - READ_START]));

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%.0f)", data.timestamp, data.value);
        }

        return data;
    }   //getColorNumber

    /**
     * This method returns the red value.
     *
     * @return red value.
     */
    public TrcSensor.SensorData<Double> getRedValue()
    {
        final String funcName = "getRedValue";
        byte[] regData = readData(READ_START, READ_LENGTH);
        TrcSensor.SensorData<Double> data = new TrcSensor.SensorData<>(
                TrcUtil.getCurrentTime(), (double)TrcUtil.bytesToInt(regData[REG_RED - READ_START]));

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%.0f)", data.timestamp, data.value);
        }

        return data;
    }   //getRedValue

    /**
     * This method returns the green value.
     *
     * @return green value.
     */
    public TrcSensor.SensorData<Double> getGreenValue()
    {
        final String funcName = "getGreenValue";
        byte[] regData = readData(READ_START, READ_LENGTH);
        TrcSensor.SensorData<Double> data = new TrcSensor.SensorData<>(
                TrcUtil.getCurrentTime(), (double)TrcUtil.bytesToInt(regData[REG_GREEN - READ_START]));

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%.0f)", data.timestamp, data.value);
        }

        return data;
    }   //getGreenValue

    /**
     * This method returns the blue value.
     *
     * @return blue value.
     */
    public TrcSensor.SensorData<Double> getBlueValue()
    {
        final String funcName = "getBlueValue";
        byte[] regData = readData(READ_START, READ_LENGTH);
        TrcSensor.SensorData<Double> data = new TrcSensor.SensorData<>(
                TrcUtil.getCurrentTime(), (double)TrcUtil.bytesToInt(regData[REG_BLUE - READ_START]));

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%.0f)", data.timestamp, data.value);
        }

        return data;
    }   //getBlueValue

    /**
     * This method returns the white value.
     *
     * @return white value.
     */
    public TrcSensor.SensorData<Double> getWhiteValue()
    {
        final String funcName = "getWhiteValue";
        byte[] regData = readData(READ_START, READ_LENGTH);
        TrcSensor.SensorData<Double> data = new TrcSensor.SensorData<>(
                TrcUtil.getCurrentTime(), (double)TrcUtil.bytesToInt(regData[REG_WHITE - READ_START]));

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%.0f)", data.timestamp, data.value);
        }

        return data;
    }   //getWhiteValue

    //
    // Implements TrcSensor.DataSource<DataType> interface.
    //

    /**
     * This method returns the sensor data of the specified index.
     *
     * @param index specifies the data index.
     * @param dataType specifies the data type.
     * @return sensor data of the specified index and type.
     */
    @Override
    public TrcSensor.SensorData<Double> getRawData(int index, DataType dataType)
    {
        final String funcName = "getRawData";
        TrcSensor.SensorData<Double> data = null;

        switch (dataType)
        {
            case COLOR_NUMBER:
                data = getColorNumber();
                break;

            case RED:
                data = getRedValue();
                break;

            case GREEN:
                data = getGreenValue();
                break;

            case BLUE:
                data = getBlueValue();
                break;

            case WHITE:
                data = getWhiteValue();
                break;
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "index=%d", index);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(time=%.3f,value=%.0f)", data.timestamp, data.value);
        }

        return data;
    }   //getRawData

}   //class FtcMRI2cColorSensor
