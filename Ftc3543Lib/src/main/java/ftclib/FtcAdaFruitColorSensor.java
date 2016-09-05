/*
 * Copyright (c) 2016 Titan Robotics Club (http://www.titanrobotics.com)
 * Contributed by FTC team 1001
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
 * This class implements the AdaFruit Color Sensor extending FtcI2cDevice.
 * It provides the TrcI2cDevice.CompletionHandler interface to read the received data.
 */
public class FtcAdaFruitColorSensor extends FtcI2cDevice implements TrcI2cDevice.CompletionHandler,
                                                                    TrcSensorDataSource
{
    private static final String moduleName = "FtcAdaFruitColorSensor";
    private static final boolean debugEnabled = false;
    private TrcDbgTrace dbgTrace = null;

    public static final int DEF_I2CADDRESS          = (0x29 << 1);
    public static final int ALTERNATE_I2CADDRESS    = (0x39 << 1);

    //
    // AdaFruit RGB Color Sensor Registers.
    //
    private static final int REG_COMMAND_BIT= 0x80;                     //Must OR into the reg address.
    private static final int REG_ENABLE     = (0x00 | REG_COMMAND_BIT); //Enables states and interrupts (R/W).
    private static final int REG_ATIME      = (0x01 | REG_COMMAND_BIT); //RGBC time (R/W).
    private static final int REG_WTIME      = (0x03 | REG_COMMAND_BIT); //Wait time (R/W).
    private static final int REG_AILTL      = (0x04 | REG_COMMAND_BIT); //Clear interrupt low threshold low byte (R/W).
    private static final int REG_AILTH      = (0x05 | REG_COMMAND_BIT); //Clear interrupt low threshold high byte (R/W).
    private static final int REG_AIHTL      = (0x06 | REG_COMMAND_BIT); //Clear interrupt high threshold low byte (R/W).
    private static final int REG_AIHTH      = (0x07 | REG_COMMAND_BIT); //Clear interrupt high threshold high byte (R/W).
    private static final int REG_PERS       = (0x0c | REG_COMMAND_BIT); //Interrupt persistence filter (R/W).
    private static final int REG_CONFIG     = (0x0d | REG_COMMAND_BIT); //Configuration (R/W).
    private static final int REG_CONTROL    = (0x0f | REG_COMMAND_BIT); //Control (R/W).
    private static final int REG_ID         = (0x12 | REG_COMMAND_BIT); //Device ID (R).
    private static final int REG_STATUS     = (0x13 | REG_COMMAND_BIT); //Device status (R).
    private static final int REG_CDATAL     = (0x14 | REG_COMMAND_BIT); //Clear data low byte (R).
    private static final int REG_CDATAH     = (0x15 | REG_COMMAND_BIT); //Clear data high byte (R).
    private static final int REG_RDATAL     = (0x16 | REG_COMMAND_BIT); //Red data low byte (R).
    private static final int REG_RDATAH     = (0x17 | REG_COMMAND_BIT); //Red data high byte (R).
    private static final int REG_GDATAL     = (0x18 | REG_COMMAND_BIT); //Green data low byte (R).
    private static final int REG_GDATAH     = (0x19 | REG_COMMAND_BIT); //Green data high byte (R).
    private static final int REG_BDATAL     = (0x1a | REG_COMMAND_BIT); //Blue data low byte (R).
    private static final int REG_BDATAH     = (0x1b | REG_COMMAND_BIT); //Blue data high byte (R).

    private static final int READ_START     = REG_STATUS;
    private static final int READ_END       = REG_BDATAH;
    private static final int READ_LENGTH    = (READ_END - READ_START + 1);

    private static final byte ENABLE_PON    = ((byte)(1 << 0)); //Power ON.
    private static final byte ENABLE_AEN    = ((byte)(1 << 1)); //RGBC enable.
    private static final byte ENABLE_WEN    = ((byte)(1 << 3)); //Wait enable.
    private static final byte ENABLE_AIEN   = ((byte)(1 << 4)); //RGBC interrupt enable.

    private static final byte ATIME_1_CYCLE = ((byte)0xff);     //2.4 ms
    private static final byte ATIME_10_CYCLE= ((byte)0xf6);     //24 ms
    private static final byte ATIME_42_CYCLE= ((byte)0xd5);     //101 ms
    private static final byte ATIME_64_CYCLE= ((byte)0xc0);     //154 ms
    private static final byte ATIME_256_CYCLE=((byte)0x00);     //700 ms

    private static final byte WTIME_1_CYCLE = ((byte)0xff);     //2.4 ms
    private static final byte WTIME_85_CYCLE= ((byte)0xab);     //204 ms
    private static final byte WTIME_256_CYCLE=((byte)0x00);     //614 ms

    private static final byte CONFIG_WLONG  = ((byte)(1 << 1)); //Wait Long.

    private static final byte CONTROL_AGAIN_1X  = 0x00;         //1X gain.
    private static final byte CONTROL_AGAIN_4X  = 0x01;         //4X gain.
    private static final byte CONTROL_AGAIN_16X = 0x02;         //16X gain.
    private static final byte CONTROL_AGAIN_60X = 0x03;         //60X gain.

    private static final byte STATUS_AVALID = ((byte)(1 << 0)); //RGBC Valid.
    private static final byte STATUS_AINT   = ((byte)(1 << 4)); //RGBC clear channel Interrupt.

    private int deviceID = 0;
    private int deviceStatus = 0;
    private TrcSensor.SensorData clearValue = new TrcSensor.SensorData(0.0, null);
    private TrcSensor.SensorData redValue = new TrcSensor.SensorData(0.0, null);
    private TrcSensor.SensorData greenValue = new TrcSensor.SensorData(0.0, null);
    private TrcSensor.SensorData blueValue = new TrcSensor.SensorData(0.0, null);

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     * @param i2cAddress specifies the I2C address of the device.
     */
    public FtcAdaFruitColorSensor(HardwareMap hardwareMap, String instanceName, int i2cAddress)
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

        setSensorEnabled(true);
        read(REG_ID, 1, this);
        read(READ_START, READ_LENGTH, this);
    }   //FtcAdaFruitColorSensor

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param i2cAddress specifies the I2C address of the device.
     */
    public FtcAdaFruitColorSensor(String instanceName, int i2cAddress)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, i2cAddress);
    }   //FtcAdaFruitColorSensor

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FtcAdaFruitColorSensor(String instanceName)
    {
        this(instanceName, DEF_I2CADDRESS);
    }   //FtcAdaFruitColorSensor

    /**
     * This method enables/disables the sensor.
     *
     * @param enabled specifies true to enable sensor, false otherwise.
     */
    public void setSensorEnabled(boolean enabled)
    {
        sendByteCommand(REG_ENABLE, enabled? (byte)(ENABLE_PON | ENABLE_AEN): 0);
    }   //setSensorEnabled

    /**
     * This method sets the RGBC time.
     *
     * @param aTime specifies the RGBC time cycles (see ATIME_*).
     */
    public void setATime(byte aTime)
    {
        sendByteCommand(REG_ATIME, aTime);
    }   //setATime

    /**
     * This method sets the wait time.
     *
     * @param wTime specifies the wait time cycles (see WTIME_*).
     */
    public void setWTime(byte wTime)
    {
        sendByteCommand(REG_WTIME, wTime);
    }   //setWTime

    /**
     * This method sets the low threshold for the Clear Interrupt.
     *
     * @param threshold specifies the low threshold value.
     */
    public void setClearInterruptLowThreshold(short threshold)
    {
        sendWordCommand(REG_AILTL, threshold);
    }   //setClearInterruptLowThreshold

    /**
     * This method sets the high threshold for the Clear Interrupt.
     *
     * @param threshold specifies the high threshold value.
     */
    public void setClearInterruptHighThreshold(short threshold)
    {
        sendWordCommand(REG_AIHTL, threshold);
    }   //setClearInterruptHighThreshold

    /**
     * This method sets the RGBC gain control.
     *
     * @param aGain specifies the RGBC gain value.
     */
    public void setAGain(byte aGain)
    {
        sendByteCommand(REG_CONTROL, aGain);
    }   //setAGain

    /**
     * This method returns the device ID.
     *
     * @return device ID.
     */
    public int getID()
    {
        return deviceID;
    }   //getID

    /**
     * This method returns the device status.
     *
     * @return device status.
     */
    public int getStatus()
    {
        return deviceStatus;
    }   //getStatus

    /**
     * This method returns the clear value.
     *
     * @return clear value.
     */
    public TrcSensor.SensorData getClearValue()
    {
        final String funcName = "getClearValue";
        TrcSensor.SensorData data =
                new TrcSensor.SensorData(clearValue.timestamp, clearValue.value);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%d)", data.timestamp, (Integer)data.value);
        }

        return data;
    } //getClearValue

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
    } //getRedValue

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
    } //getGreenValue

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
    } //getBlueValue

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

        if (regAddress == REG_ID && length == 1)
        {
            if (!timedout)
            {
                deviceID = TrcUtil.bytesToInt(data[0]);
            }
            else
            {
                repeat = true;
            }
        }
        else if (regAddress == READ_START && length == READ_LENGTH)
        {
            if (!timedout)
            {
                //
                // Read these repeatedly.
                //
                deviceStatus = TrcUtil.bytesToInt(data[REG_STATUS - READ_START]);
                if ((deviceStatus & STATUS_AVALID) != 0)
                {
                    clearValue.timestamp = timestamp;
                    clearValue.value = TrcUtil.bytesToInt(data[REG_CDATAL - READ_START],
                                                          data[REG_CDATAH - READ_START]);
                    redValue.timestamp = timestamp;
                    redValue.value = TrcUtil.bytesToInt(data[REG_RDATAL - READ_START],
                                                        data[REG_RDATAH - READ_START]);
                    greenValue.timestamp = timestamp;
                    greenValue.value = TrcUtil.bytesToInt(data[REG_GDATAL - READ_START],
                                                          data[REG_GDATAH - READ_START]);
                    blueValue.timestamp = timestamp;
                    blueValue.value = TrcUtil.bytesToInt(data[REG_BDATAL - READ_START],
                                                         data[REG_BDATAH - READ_START]);
                }
            }
            repeat = true;
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.CALLBK,
                                "regAddr=%x,len=%d,timestamp=%.3f,timedout=%s" ,
                                regAddress, length, timestamp, Boolean.toString(timedout));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.CALLBK,
                               "=%s", Boolean.toString(repeat));
            dbgTrace.traceInfo(funcName, "%s(addr=%x,len=%d,time=%.3f,size=%d,timedout=%s)=%s",
                               funcName, regAddress, length, timestamp, data.length,
                               Boolean.toString(timedout), Boolean.toString(repeat));
        }

        return repeat;
    } //readCompletion

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
    } //writeCompletion

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
                data = getClearValue();
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
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "index=%d", index);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(time=%.3f,value=%d)", data.timestamp, data.value);
        }

        return data;
    } //getSensorData

}   //class FtcAdaFruitColorSensor
