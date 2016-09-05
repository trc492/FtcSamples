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
import trclib.TrcI2cDevice;
import trclib.TrcSensor;
import trclib.TrcSensorDataSource;
import trclib.TrcUtil;

/**
 * This class implements the Modern Robotics Gyro extending FtcI2cDevice.
 * It provides the TrcI2cDevice.CompletionHandler interface to read the
 * received data.
 */
public class FtcMRI2cGyro extends FtcMRI2cDevice implements TrcI2cDevice.CompletionHandler,
                                                            TrcSensorDataSource
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

    private TrcSensor.SensorData heading = new TrcSensor.SensorData(0.0, null);
    private TrcSensor.SensorData integratedZ = new TrcSensor.SensorData(0.0, null);
    private TrcSensor.SensorData rawX = new TrcSensor.SensorData(0.0, null);
    private TrcSensor.SensorData rawY = new TrcSensor.SensorData(0.0, null);
    private TrcSensor.SensorData rawZ = new TrcSensor.SensorData(0.0, null);
    private TrcSensor.SensorData zOffset = new TrcSensor.SensorData(0.0, null);
    private TrcSensor.SensorData zScaling = new TrcSensor.SensorData(0.0, null);
    private boolean calibrating = false;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     * @param i2cAddress specifies the I2C address of the device.
     */
    public FtcMRI2cGyro(HardwareMap hardwareMap, String instanceName, int i2cAddress)
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

        resetZIntegrator();
        read(READ_START, READ_LENGTH, this);
    }   //FtcMRI2cGyro

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param i2cAddress specifies the I2C address of the device.
     */
    public FtcMRI2cGyro(String instanceName, int i2cAddress)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, i2cAddress);
    }   //FtcMRI2cGyro

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FtcMRI2cGyro(String instanceName)
    {
        this(instanceName, DEF_I2CADDRESS);
    }   //FtcMRI2cGyro

    /**
     * This method initiates the gyro calibration. The process may take a little
     * time to complete.
     */
    public void calibrate()
    {
        final String funcName = "calibrate";

        sendByteCommand(REG_COMMAND, CMD_RESET_OFFSET_CAL);
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

        sendByteCommand(REG_COMMAND, CMD_RESET_Z_INTEGRATOR);

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
        TrcSensor.SensorData data =
                new TrcSensor.SensorData(heading.timestamp, heading.value);

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
        TrcSensor.SensorData data =
                new TrcSensor.SensorData(integratedZ.timestamp, integratedZ.value);

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
        TrcSensor.SensorData data =
                new TrcSensor.SensorData(rawX.timestamp, rawX.value);

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
        TrcSensor.SensorData data =
                new TrcSensor.SensorData(rawY.timestamp, rawY.value);

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
        TrcSensor.SensorData data =
                new TrcSensor.SensorData(rawZ.timestamp, rawZ.value);

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
        TrcSensor.SensorData data =
                new TrcSensor.SensorData(zOffset.timestamp, zOffset.value);

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
        TrcSensor.SensorData data =
                new TrcSensor.SensorData(zScaling.timestamp, zScaling.value);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%d)", data.timestamp, (Integer)data.value);
        }

        return data;
    }   //getZScaling

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
                int value = TrcUtil.bytesToInt(data[REG_HEADING_LSB - READ_START]);
                heading.timestamp = timestamp;
                heading.value = (360 - value)%360;

                integratedZ.timestamp = timestamp;
                integratedZ.value =
                        -TrcUtil.bytesToInt(data[REG_INTEGRATED_Z_LSB - READ_START],
                                            data[REG_INTEGRATED_Z_MSB - READ_START]);

                rawX.timestamp = timestamp;
                rawX.value =
                        -TrcUtil.bytesToInt(data[REG_RAW_X_LSB - READ_START],
                                            data[REG_RAW_X_MSB - READ_START]);

                rawY.timestamp = timestamp;
                rawY.value =
                        -TrcUtil.bytesToInt(data[REG_RAW_Y_LSB - READ_START],
                                            data[REG_RAW_Y_MSB - READ_START]);

                rawZ.timestamp = timestamp;
                rawZ.value =
                        -TrcUtil.bytesToInt(data[REG_RAW_Z_LSB - READ_START],
                                            data[REG_RAW_Z_MSB - READ_START]);

                zOffset.timestamp = timestamp;
                zOffset.value =
                        TrcUtil.bytesToInt(data[REG_Z_OFFSET_LSB - READ_START],
                                           data[REG_Z_OFFSET_MSB - READ_START]);

                zScaling.timestamp = timestamp;
                zScaling.value =
                        TrcUtil.bytesToInt(data[REG_Z_SCALING_LSB - READ_START],
                                           data[REG_Z_SCALING_MSB - READ_START]);
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
        final String funcName = "writeCompletion";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.CALLBK,
                                "regAddr=%x,len=%d,timedout=%s",
                                regAddress, length, Boolean.toString(timedout));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.CALLBK);
            dbgTrace.traceInfo(funcName, "%s(addr=%x,len=%d,timedout=%s)",
                               funcName, regAddress, length, Boolean.toString(timedout));
        }

        if (regAddress == REG_COMMAND && length == 1)
        {
            if (calibrating)
            {
                //
                // This was the calibrate command, mark it done.
                //
                calibrating = false;
            }
        }
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
