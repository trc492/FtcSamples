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

import trclib.TrcDbgTrace;
import trclib.TrcSensor;
import trclib.TrcUtil;

/**
 * This class implements the Modern Color sensor extending TrcAnalogInput. It provides implementation of the abstract
 * methods in TrcAnalogInput.
 */
public class FtcMRColorSensor extends TrcSensor<FtcMRColorSensor.DataType>
{
    private static final String moduleName = "FtcMRColorSensor";
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

    public ModernRoboticsI2cColorSensor sensor;
    private int argbData = 0;
    private long argbTagId = -1;
    private int redData = 0;
    private long redTagId = -1;
    private int greenData = 0;
    private long greenTagId = -1;
    private int blueData = 0;
    private long blueTagId = -1;
    private int alphaData = 0;
    private long alphaTagId = -1;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     */
    public FtcMRColorSensor(HardwareMap hardwareMap, String instanceName)
    {
        super(instanceName, 1);

        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        sensor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, instanceName);
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

    //
    // Implements TrcAnalogInput abstract methods.
    //

    /**
     * This method returns the raw sensor data of the specified type.
     *
     * @param index specifies the data index.
     * @param dataType specifies the data type.
     * @return raw sensor data of the specified index and type.
     */
    @Override
    public SensorData<Integer> getRawData(int index, DataType dataType)
    {
        final String funcName = "getRawData";
        SensorData<Integer> data = null;
        long currTagId = FtcOpMode.getLoopCounter();

        switch (dataType)
        {
            case COLOR_NUMBER:
                if (currTagId != argbTagId)
                {
                    argbData = sensor.argb();
                    argbTagId = currTagId;
                }
                data = new SensorData<>(TrcUtil.getCurrentTime(), argbData);
                break;

            case RED:
                if (currTagId != redTagId)
                {
                    redData = sensor.red();
                    redTagId = currTagId;
                }
                data = new SensorData<>(TrcUtil.getCurrentTime(), redData);
                break;

            case GREEN:
                if (currTagId != greenTagId)
                {
                    greenData = sensor.green();
                    greenTagId = currTagId;
                }
                data = new SensorData<>(TrcUtil.getCurrentTime(), greenData);
                break;

            case BLUE:
                if (currTagId != blueTagId)
                {
                    blueData = sensor.blue();
                    blueTagId = currTagId;
                }
                data = new SensorData<>(TrcUtil.getCurrentTime(), blueData);
                break;

            case WHITE:
                if (currTagId != alphaTagId)
                {
                    alphaData = sensor.alpha();
                    alphaTagId = currTagId;
                }
                data = new SensorData<>(TrcUtil.getCurrentTime(), alphaData);
                break;
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp:%.3f,value=%d)", data.timestamp, data.value);
        }

        return data;
    }   //getRawData

}   //class FtcMRColorSensor
