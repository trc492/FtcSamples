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
import trclib.TrcGyro;
import trclib.TrcSensor;
import trclib.TrcUtil;

/**
 * This class implements the Modern Robotics Gyro extending FtcMRI2cDevice that implements the common features of
 * all Modern Robotics I2C devices.
 */
public class FtcMRI2cGyro extends FtcMRI2cDevice
                          implements TrcGyro.GyroData, TrcSensor.DataSource<FtcMRI2cGyro.DataType>
{
    private static final String moduleName = "FtcMRI2cGyro";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    public enum DataType
    {
        HEADING,
        INTEGRATED_Z,
        RAW_X,
        RAW_Y,
        RAW_Z,
        Z_OFFSET,
        Z_SCALING
    }   //enum DataType

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
    private int xSign = 1;
    private int ySign = 1;
    private int zSign = 1;

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
            dbgTrace = new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
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
     * This method initiates the gyro calibration. The process may take a little time to complete.
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
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Boolean.toString(calibrating));
        }

        return calibrating;
    }   //isCalibrating

    /**
     * This method returns the heading data.
     *
     * @return heading data in the range of 0 and 359 inclusive.
     */
    public TrcSensor.SensorData<Double> getHeading()
    {
        final String funcName = "getHeading";
        byte[] regData = getData(readerId);
        int value = zSign*TrcUtil.bytesToInt(
                regData[REG_HEADING_LSB - READ_START], regData[REG_HEADING_MSB - READ_START]);
        //
        // MR gyro heading is decreasing when turning clockwise. This is opposite to convention.
        // So we are reversing it.
        //
        TrcSensor.SensorData<Double> data = new TrcSensor.SensorData<>(
                getDataTimestamp(readerId), (double)((360 - value)%360));

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%.0f)", data.timestamp, data.value);
        }

        return data;
    }   //getHeading

    /**
     * This method returns the integrated Z value.
     *
     * @return integrated Z value.
     */
    public TrcSensor.SensorData<Double> getIntegratedZ()
    {
        final String funcName = "getIntegratedZ";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        TrcSensor.SensorData<Double> data;
        byte[] regData = getData(readerId);
        //
        // MR gyro IntegratedZ is decreasing when turning clockwise. This is opposite to convention.
        // So we are reversing it.
        //
        if (regData != null)
        {
            int value = zSign * TrcUtil.bytesToInt(
                    regData[REG_INTEGRATED_Z_LSB - READ_START], regData[REG_INTEGRATED_Z_MSB - READ_START]);
            data = new TrcSensor.SensorData<>(getDataTimestamp(readerId), (double)-value);
        }
        else
        {
            data = new TrcSensor.SensorData<>(0, 0.0);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%.0f)", data.timestamp, data.value);
        }

        return data;
    }   //getIntegratedZ

    /**
     * This method returns the raw turn rate of the X-axis.
     *
     * @return raw X turn rate.
     */
    public TrcSensor.SensorData<Double> getRawX()
    {
        final String funcName = "getRawX";
        byte[] regData = getData(readerId);
        TrcSensor.SensorData<Double> data = new TrcSensor.SensorData<>(
                getDataTimestamp(readerId),
                -xSign*(double)TrcUtil.bytesToInt(
                        regData[REG_RAW_X_LSB - READ_START], regData[REG_RAW_X_MSB - READ_START]));

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%.0f)", data.timestamp, data.value);
        }

        return data;
    }   //getRawX

    /**
     * This method returns the raw turn rate of the Y-axis.
     *
     * @return raw Y turn rate.
     */
    public TrcSensor.SensorData<Double> getRawY()
    {
        final String funcName = "getRawY";
        byte[] regData = getData(readerId);
        TrcSensor.SensorData<Double> data = new TrcSensor.SensorData<>(
                getDataTimestamp(readerId),
                -ySign*(double)TrcUtil.bytesToInt(
                        regData[REG_RAW_Y_LSB - READ_START], regData[REG_RAW_Y_MSB - READ_START]));

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%.0f)", data.timestamp, data.value);
        }

        return data;
    }   //getRawY

    /**
     * This method returns the raw turn rate of the Z-axis.
     *
     * @return raw Z turn rate.
     */
    public TrcSensor.SensorData<Double> getRawZ()
    {
        final String funcName = "getRawZ";
        byte[] regData = getData(readerId);
        TrcSensor.SensorData<Double> data = new TrcSensor.SensorData<>(
                getDataTimestamp(readerId),
                -zSign*(double)TrcUtil.bytesToInt(
                        regData[REG_RAW_Z_LSB - READ_START], regData[REG_RAW_Z_MSB - READ_START]));

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%.0f)", data.timestamp, data.value);
        }

        return data;
    }   //getRawZ

    /**
     * This method returns the offset of the Z-axis.
     *
     * @return Z offset.
     */
    public TrcSensor.SensorData<Double> getZOffset()
    {
        final String funcName = "getZOffset";
        byte[] regData = getData(readerId);
        TrcSensor.SensorData<Double> data = new TrcSensor.SensorData<>(
                getDataTimestamp(readerId),
                (double)TrcUtil.bytesToInt(
                        regData[REG_Z_OFFSET_LSB - READ_START], regData[REG_Z_OFFSET_MSB - READ_START]));

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%.0f)", data.timestamp, data.value);
        }

        return data;
    }   //getZOffset

    /**
     * This method returns the scaling coefficient of the Z-axis.
     *
     * @return Z scaling coefficient.
     */
    public TrcSensor.SensorData<Double> getZScaling()
    {
        final String funcName = "getZScaling";
        byte[] regData = getData(readerId);
        TrcSensor.SensorData<Double> data = new TrcSensor.SensorData<>(
                getDataTimestamp(readerId),
                (double)TrcUtil.bytesToInt(
                        regData[REG_Z_SCALING_LSB - READ_START], regData[REG_Z_SCALING_MSB - READ_START]));

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%.0f)", data.timestamp, data.value);
        }

        return data;
    }   //getZScaling

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
            case HEADING:
                data = getHeading();
                break;

            case INTEGRATED_Z:
                data = getIntegratedZ();
                break;

            case RAW_X:
                data = getRawX();
                break;

            case RAW_Y:
                data = getRawY();
                break;

            case RAW_Z:
                data = getRawZ();
                break;

            case Z_OFFSET:
                data = getZOffset();
                break;

            case Z_SCALING:
                data = getZScaling();
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

    //
    // Implements HalGyro interface.
    //

    /**
     * This method inverts the x-axis. This is useful if the orientation of the gyro x-axis is such that the data
     * goes the wrong direction.
     *
     * @param inverted specifies true to invert x-axis, false otherwise.
     */
    @Override
    public void setXInverted(boolean inverted)
    {
        xSign = inverted? -1: 1;
    }   //setXInverted

    /**
     * This method inverts the y-axis. This is useful if the orientation of the gyro y-axis is such that the data
     * goes the wrong direction.
     *
     * @param inverted specifies true to invert y-axis, false otherwise.
     */
    @Override
    public void setYInverted(boolean inverted)
    {
        ySign = inverted? -1: 1;
    }   //setYInverted

    /**
     * This method inverts the z-axis. This is useful if the orientation of the gyro z-axis is such that the data
     * goes the wrong direction.
     *
     * @param inverted specifies true to invert z-axis, false otherwise.
     */
    @Override
    public void setZInverted(boolean inverted)
    {
        zSign = inverted? -1: 1;
    }   //setZInverted

    /**
     * This method returns the rotation rate on the x-axis.
     *
     * @return X rotation rate.
     */
    @Override
    public TrcSensor.SensorData<Double> getXRotationRate()
    {
        return getRawX();
    }   //getXRotationRate

    /**
     * This method returns the rotation rate on the y-axis.
     *
     * @return Y rotation rate.
     */
    @Override
    public TrcSensor.SensorData<Double> getYRotationRate()
    {
        return getRawY();
    }   //getYRotationRate

    /**
     * This method returns the rotation rate on the z-axis.
     *
     * @return Z rotation rate.
     */
    @Override
    public TrcSensor.SensorData<Double> getZRotationRate()
    {
        return getRawZ();
    }   //getZRotationRate

    /**
     * This method returns the heading of the x-axis. If there is an integrator, we call the integrator to get
     * the heading. Else if we have an unwrapper, we call the unwrapper to get the heading else we call the
     * platform dependent gyro to get the raw heading value.
     *
     * @return X heading.
     */
    @Override
    public TrcSensor.SensorData<Double> getXHeading()
    {
        throw new UnsupportedOperationException("Modern Robotics Gyro does not support X heading.");
    }   //getXHeading

    /**
     * This method returns the heading of the y-axis. If there is an integrator, we call the integrator to get
     * the heading. Else if we have an unwrapper, we call the unwrapper to get the heading else we call the
     * platform dependent gyro to get the raw heading value.
     *
     * @return Y heading.
     */
    public TrcSensor.SensorData<Double> getYHeading()
    {
        throw new UnsupportedOperationException("Modern Robotics Gyro does not support Y heading.");
    }   //getYHeading

    /**
     * This method returns the heading of the z-axis. If there is an integrator, we call the integrator to get
     * the heading. Else if we have an unwrapper, we call the unwrapper to get the heading else we call the
     * platform dependent gyro to get the raw heading value.
     *
     * @return Z heading.
     */
    @Override
    public TrcSensor.SensorData<Double> getZHeading()
    {
        return getHeading();
    }   //getZHeading

    /**
     * This method resets the integrator on the x-axis.
     */
    @Override
    public void resetXIntegrator()
    {
        throw new UnsupportedOperationException("Modern Robotics Gyro does not have X integrator.");
    }   //resetXIntegrator

    /**
     * This method resets the integrator on the y-axis.
     */
    @Override
    public void resetYIntegrator()
    {
        throw new UnsupportedOperationException("Modern Robotics Gyro does not have Y integrator.");
    }   //resetYIntegrator

    /**
     * This method resets the integrator on the z-axis.
     */
    @Override
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

}   //class FtcMRI2cGyro
