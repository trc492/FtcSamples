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

package ftclib;

import com.qualcomm.robotcore.hardware.HardwareMap;

import trclib.TrcDbgTrace;
import trclib.TrcSensor;
import trclib.TrcSensorDataSource;
import trclib.TrcUtil;

/**
 * This class implements the Modern Robotics Gyro extending FtcMRI2cDevice that implements
 * the common features of all Modern Robotics I2C devices.
 */
public class FtcMRI2cGyro extends FtcMRI2cDevice implements TrcSensorDataSource
{
    private static final String moduleName = "FtcMRI2cGyro";
    private static final boolean debugEnabled = false;
    private TrcDbgTrace dbgTrace = null;

    public static final int DEF_I2CADDRESS          = 0x20;

    //
    // I2C registers.
    //
    private static final int REG_HEADING_LSB        = 0x04;
    private static final int REG_HEADING_MSB        = 0x05;
    private static final int REG_INTEGRATED_Z_LSB   = 0x06;
    private static final int REG_INTEGRATED_Z_MSB   = 0x07;
    private static final int REG_RAW_X_LSB          = 0x08;
    private static final int REG_RAW_X_MSB          = 0x09;
    private static final int REG_RAW_Y_LSB          = 0x0a;
    private static final int REG_RAW_Y_MSB          = 0x0b;
    private static final int REG_RAW_Z_LSB          = 0x0c;
    private static final int REG_RAW_Z_MSB          = 0x0d;
    private static final int REG_Z_OFFSET_LSB       = 0x0e;
    private static final int REG_Z_OFFSET_MSB       = 0x0f;
    private static final int REG_Z_SCALING_LSB      = 0x10;
    private static final int REG_Z_SCALING_MSB      = 0x11;

    private static final int READ_START             = REG_HEADING_LSB;
    private static final int READ_END               = REG_Z_SCALING_MSB;
    private static final int READ_LENGTH            = (READ_END - READ_START + 1);

    private static final byte CMD_MEASUREMENT_MODE  = 0x00;
    private static final byte CMD_RESET_OFFSET_CAL  = 0x4e;
    private static final byte CMD_RESET_Z_INTEGRATOR= 0x52;
    private static final byte CMD_WRITE_EEPROM_DATA = 0x57;

    private int readerId = -1;
    private boolean calibrating = false;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     * @param i2cAddress specifies the I2C address of the device.
     * @param addressIs7Bit specifies true if the I2C address is a 7-bit address, false if it is 8-bit.
     */
    public FtcMRI2cGyro(HardwareMap hardwareMap, String instanceName, int i2cAddress, boolean addressIs7Bit)
    {
        super(hardwareMap, instanceName, i2cAddress, addressIs7Bit);

        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(
                    moduleName + "." + instanceName,
                    false,
                    TrcDbgTrace.TraceLevel.API,
                    TrcDbgTrace.MsgLevel.INFO);
        }

        resetZIntegrator();
        readerId = addReader(instanceName, READ_START, READ_LENGTH);
    }   //FtcMRI2cGyro

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param i2cAddress specifies the I2C address of the device.
     * @param addressIs7Bit specifies true if the I2C address is a 7-bit address, false if it is 8-bit.
     */
    public FtcMRI2cGyro(String instanceName, int i2cAddress, boolean addressIs7Bit)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, i2cAddress, addressIs7Bit);
    }   //FtcMRI2cGyro

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FtcMRI2cGyro(String instanceName)
    {
        this(instanceName, DEF_I2CADDRESS, false);
    }   //FtcMRI2cGyro

    /**
     * This method initiates the gyro calibration. The process may take a little
     * time to complete.
     */
    public void calibrate()
    {
        final String funcName = "calibrate";

        sendByteCommand(REG_COMMAND, CMD_RESET_OFFSET_CAL, false);
        calibrating = true;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //calibrate

    /**
     * This method check if the calibration is still in progress.
     *
     * @return true if calibration is still in progress, false otherwise.
     */
    public boolean isCalibrating()
    {
        final String funcName = "isCalibrating";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=%s", Boolean.toString(calibrating));
        }

        return calibrating;
    }   //isCalibrating

    /**
     * This method resets the Z integrator and the heading to zero.
     */
    public void resetZIntegrator()
    {
        final String funcName = "resetZIntegrator";

        sendByteCommand(REG_COMMAND, CMD_RESET_Z_INTEGRATOR, false);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //resetZIntegrator

    /**
     * This method returns the heading data.
     *
     * @return heading data in the range of 0 and 359 inclusive.
     */
    public TrcSensor.SensorData getHeading()
    {
        final String funcName = "getHeading";
        byte[] regData = getData(readerId);
        int value = TrcUtil.bytesToInt(regData[REG_HEADING_LSB - READ_START], regData[REG_HEADING_MSB - READ_START]);
        //
        // MR gyro heading is decreasing when turning clockwise. This is opposite to convention.
        // So we are reversing it.
        //
        TrcSensor.SensorData data = new TrcSensor.SensorData(getDataTimestamp(readerId), (360 - value)%360);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%d)", data.timestamp, (Integer)data.value);
        }

        return data;
    }   //getHeading

    /**
     * This method returns the integrated Z value.
     *
     * @return integrated Z value.
     */
    public TrcSensor.SensorData getIntegratedZ()
    {
        final String funcName = "getIntegratedZ";
        byte[] regData = getData(readerId);
        //
        // MR gyro IntegratedZ is decreasing when turning clockwise. This is opposite to convention.
        // So we are reversing it.
        //
        short value = TrcUtil.bytesToShort(regData[REG_INTEGRATED_Z_LSB - READ_START],
                                           regData[REG_INTEGRATED_Z_MSB - READ_START]);
        TrcSensor.SensorData data = new TrcSensor.SensorData(getDataTimestamp(readerId), -value);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%d)", data.timestamp, (Integer)data.value);
        }

        return data;
    }   //getIntegratedZ

    /**
     * This method returns the raw turn rate of the X-axis.
     *
     * @return raw X turn rate.
     */
    public TrcSensor.SensorData getRawX()
    {
        final String funcName = "getRawX";
        byte[] regData = getData(readerId);
        TrcSensor.SensorData data = new TrcSensor.SensorData(
                getDataTimestamp(readerId),
                -TrcUtil.bytesToInt(regData[REG_RAW_X_LSB - READ_START], regData[REG_RAW_X_MSB - READ_START]));

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%d)", data.timestamp, (Integer)data.value);
        }

        return data;
    }   //getRawX

    /**
     * This method returns the raw turn rate of the Y-axis.
     *
     * @return raw Y turn rate.
     */
    public TrcSensor.SensorData getRawY()
    {
        final String funcName = "getRawY";
        byte[] regData = getData(readerId);
        TrcSensor.SensorData data = new TrcSensor.SensorData(
                getDataTimestamp(readerId),
                -TrcUtil.bytesToInt(regData[REG_RAW_Y_LSB - READ_START],
                                    regData[REG_RAW_Y_MSB - READ_START]));

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%d)", data.timestamp, (Integer)data.value);
        }

        return data;
    }   //getRawY

    /**
     * This method returns the raw turn rate of the Z-axis.
     *
     * @return raw Z turn rate.
     */
    public TrcSensor.SensorData getRawZ()
    {
        final String funcName = "getRawZ";
        byte[] regData = getData(readerId);
        TrcSensor.SensorData data = new TrcSensor.SensorData(
                getDataTimestamp(readerId),
                -TrcUtil.bytesToInt(regData[REG_RAW_Z_LSB - READ_START],
                                    regData[REG_RAW_Z_MSB - READ_START]));

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%d)", data.timestamp, (Integer)data.value);
        }

        return data;
    }   //getRawZ

    /**
     * This method returns the offset of the Z-axis.
     *
     * @return Z offset.
     */
    public TrcSensor.SensorData getZOffset()
    {
        final String funcName = "getZOffset";
        byte[] regData = getData(readerId);
        TrcSensor.SensorData data = new TrcSensor.SensorData(
                getDataTimestamp(readerId),
                TrcUtil.bytesToInt(regData[REG_Z_OFFSET_LSB - READ_START],
                                   regData[REG_Z_OFFSET_MSB - READ_START]));

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%d)", data.timestamp, (Integer)data.value);
        }

        return data;
    }   //getZOffset

    /**
     * This method returns the scaling coefficient of the Z-axis.
     *
     * @return Z scaling coefficient.
     */
    public TrcSensor.SensorData getZScaling()
    {
        final String funcName = "getZScaling";
        byte[] regData = getData(readerId);
        TrcSensor.SensorData data = new TrcSensor.SensorData(
                getDataTimestamp(readerId),
                TrcUtil.bytesToInt(regData[REG_Z_SCALING_LSB - READ_START],
                                   regData[REG_Z_SCALING_MSB - READ_START]));

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%d)", data.timestamp, (Integer)data.value);
        }

        return data;
    }   //getZScaling

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
                data = getHeading();
                break;

            case 1:
                data = getIntegratedZ();
                break;

            case 2:
                data = getRawX();
                break;

            case 3:
                data = getRawY();
                break;

            case 4:
                data = getRawZ();
                break;

            case 5:
                data = getZOffset();
                break;

            case 6:
                data = getZScaling();
                break;
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "index=%d", index);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(time=%.3f,value=%f)", data.timestamp, data.value);
        }

        return data;
    }   //getSensorData

}   //class FtcMRI2cGyro
