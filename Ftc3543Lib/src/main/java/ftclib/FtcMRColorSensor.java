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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;

import hallib.HalUtil;
import trclib.TrcAnalogInput;
import trclib.TrcDbgTrace;

/**
 * This class implements the Modern Color sensor extending TrcAnalogInput. It provides implementation of the
 * abstract methods in TrcAnalogInput.
 */
public class FtcMRColorSensor extends TrcAnalogInput
{
    private static final String moduleName = "FtcMRColorSensor";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private TrcDbgTrace dbgTrace = null;

    public ModernRoboticsI2cColorSensor sensor;
    private FtcI2cDeviceState sensorState;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     */
    public FtcMRColorSensor(HardwareMap hardwareMap, String instanceName)
    {
        super(instanceName, 1, 0, null);

        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled,
                                       TrcDbgTrace.TraceLevel.API, TrcDbgTrace.MsgLevel.INFO);
        }

        sensor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, instanceName);
        sensorState = new FtcI2cDeviceState(sensor);
    }   //FtcMRColorSensor

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FtcMRColorSensor(String instanceName)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName);
    }   //FtcMRColorSensor

    /**
     * This method calibrates the sensor.
     */
    public void calibrate()
    {
        calibrate(DataType.INPUT_DATA);
    }   //calibrate

    /**
     * This method check if the color sensor is enabled.
     *
     * @return true if the device state indicates it is enabled, false otherwise.
     */
    public boolean isDeviceEnabled()
    {
        final String funcName = "isDeviceEnabled";
        boolean enabled = sensorState.isEnabled();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Boolean.toString(enabled));
        }

        return enabled;
    }   //isDeviceEnabled

    /**
     * This method is called to enable/disable the sensor so it is not hogging I2c bus bandwidth when not in use.
     *
     * @param enabled specifies true if enabling, false otherwise.
     */
    public void setDeviceEnabled(boolean enabled)
    {
        final String funcName = "setDeviceEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enabled=%s", Boolean.toString(enabled));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        sensorState.setEnabled(enabled);
    }   //setDeviceEnabled

    //
    // Implements TrcAnalogInput abstract methods.
    //

    /**
     * This method returns the raw sensor data of the specified type.
     *
     * @param index specifies the data index.
     * @return raw sensor data of the specified type.
     */
    @Override
    public SensorData getRawData(int index, DataType dataType)
    {
        final String funcName = "getRawData";
        SensorData data = null;

        //
        // Color sensor supports only INPUT_DATA type.
        //
        if (dataType == DataType.INPUT_DATA)
        {
            if (index == 0)
            {
                data = new SensorData(
                        HalUtil.getCurrentTime(), sensor.argb());
            }
            else if (index == 1)
            {
                data = new SensorData(
                        HalUtil.getCurrentTime(), sensor.red());
            }
            else if (index == 2)
            {
                data = new SensorData(
                        HalUtil.getCurrentTime(), sensor.green());
            }
            else if (index == 3)
            {
                data = new SensorData(
                        HalUtil.getCurrentTime(), sensor.blue());
            }
            else if (index == 4)
            {
                data = new SensorData(
                        HalUtil.getCurrentTime(), sensor.alpha());
            }
            else
            {
                throw new IllegalArgumentException("Invalid data index.");
            }
        }
        else
        {
            throw new UnsupportedOperationException(
                    "Color sensor only support INPUT_DATA type.");
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp:%.3f,value=%f)", data.timestamp, data.value);
        }

        return data;
    }   //getRawData

}   //class FtcMRColorSensor
