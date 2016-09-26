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
import trclib.TrcSensor;
import trclib.TrcSensorDataSource;
import trclib.TrcUtil;

/**
 * This class implements the Modern Robotics Range Sensor extending FtcMRI2cDevice that implements
 * the common features of all Modern Robotics I2C devices.
 */
public class FtcMRRangeSensor extends FtcMRI2cDevice implements TrcSensorDataSource
{
    private static final String moduleName = "FtcMRRangeSensor";
    private static final boolean debugEnabled = false;
    private TrcDbgTrace dbgTrace = null;

    public static final int DEF_I2CADDRESS          = 0x28;     //8-bit address.

    //
    // I2C registers.
    //
    private static final int REG_ULTRSONIC_DISTANCE = 0x04;
    private static final int REG_OPTICAL_DISTANCE   = 0x05;

    private static final int READ_START             = REG_ULTRSONIC_DISTANCE;
    private static final int READ_END               = REG_OPTICAL_DISTANCE;
    private static final int READ_LENGTH            = (READ_END - READ_START + 1);

    private int readerId = -1;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     * @param i2cAddress specifies the I2C address of the device.
     * @param addressIs7Bit specifies true if the I2C address is a 7-bit address, false if it is 8-bit.
     */
    public FtcMRRangeSensor(HardwareMap hardwareMap, String instanceName, int i2cAddress, boolean addressIs7Bit)
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

        readerId = addReader(instanceName, READ_START, READ_LENGTH);
    }   //FtcMRRangeSensor

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param i2cAddress specifies the I2C address of the device.
     * @param addressIs7Bit specifies true if the I2C address is a 7-bit address, false if it is 8-bit.
     */
    public FtcMRRangeSensor(String instanceName, int i2cAddress, boolean addressIs7Bit)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, i2cAddress, addressIs7Bit);
    }   //FtcMRRangeSensor

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FtcMRRangeSensor(String instanceName)
    {
        this(instanceName, DEF_I2CADDRESS, false);
    }   //FtcMRRangeSensor

    /**
     * This method returns the ultrasonic distance.
     *
     * @return ultrasonic distance.
     */
    public TrcSensor.SensorData getUltrasonicDistance()
    {
        final String funcName = "getUltrasonicDistance";
        byte[] regData = getData(readerId);
        TrcSensor.SensorData data = new TrcSensor.SensorData(
                getDataTimestamp(readerId), TrcUtil.bytesToInt(regData[REG_ULTRSONIC_DISTANCE - READ_START]));

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%d)", data.timestamp, (Integer)data.value);
        }

        return data;
    }   //getUltrasonicDistance

    /**
     * This method returns the optical distance.
     *
     * @return optical distance.
     */
    public TrcSensor.SensorData getOpticalDistance()
    {
        final String funcName = "getOpticalDistance";
        byte[] regData = getData(readerId);
        TrcSensor.SensorData data = new TrcSensor.SensorData(
                getDataTimestamp(readerId), TrcUtil.bytesToInt(regData[REG_OPTICAL_DISTANCE - READ_START]));

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp=%.3f,value=%d)", data.timestamp, (Integer)data.value);
        }

        return data;
    }   //getOpticalDistance

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
                data = getUltrasonicDistance();
                break;

            case 1:
                data = getOpticalDistance();
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

}   //class FtcMRRangeSensor
