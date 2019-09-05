/*
 * Copyright (c) 2018 Titan Robotics Club (http://www.titanrobotics.com)
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
 * This class implements the Lidar Lite v3 Laser Ranging sensor.
 */
public class TrcLidarLite
{
    private static final String moduleName = "TrcLidarLite";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = true;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    public enum RequestId
    {
        READ_DISTANCE,
        GET_DISTANCE
    }   //enum RequestId

    public static final int DEF_I2C_ADDRESS_7BIT        = 0x62;

    //
    // Garmin Lidar Lite v3 Laser Ranging Module
    //
    private static final int REG_ACQ_COMMAND            = 0x00;     //Device command (W)
    private static final int REG_STATUS                 = 0x01;     //System status (R)
//    private static final int REG_SIG_COUNT_VAL          = 0x02;     //Maximum acquisition count (R/W)
//    private static final int REG_ACQ_CONFIG             = 0x04;     //Acquisition mode control (R/W)
//    private static final int REG_VELOCITY               = 0x09;     //Velocity measurement output (R)
//    private static final int REG_PEAK_CORR              = 0x0c;     //Peak value in correlation record (R)
//    private static final int REG_NOISE_PEAK             = 0x0d;     //Correlation record noise floor (R)
//    private static final int REG_SIGNAL_STRENGTH        = 0x0e;     //Received signal strength (R)
    private static final int REG_FULL_DELAY_HIGH        = 0x0f;     //Distance measurement high byte (R)
//    private static final int REG_FULL_DELAY_LOW         = 0x10;     //Distance measurement low byte (R)
//    private static final int REG_OUTER_LOOP_COUNT       = 0x11;     //Burst measure count control (R/W)
//    private static final int REG_REF_COUNT_VAL          = 0x12;     //Reference acquisition count (R/W)
//    private static final int REG_LAST_DELAY_HIGH        = 0x14;     //Previous distance measurement high byte (R)
//    private static final int REG_LAST_DELAY_LOW         = 0x15;     //Previous distance measurement low byte (R)
//    private static final int REG_UNIT_ID_HIGH           = 0x16;     //Serial number high byte (R)
//    private static final int REG_UNIT_ID_LOW            = 0x17;     //Serial number low byte (R)
//    private static final int REG_I2C_ID_HIGH            = 0x18;     //Write serial number high byte for I2C address unlock (W)
//    private static final int REG_I2C_ID_LOW             = 0x19;     //Write serial number low byte for I2C address unlock (W)
//    private static final int REG_I2C_SEC_ADDR           = 0x1a;     //Write new I2C address after unlock (R/W)
//    private static final int REG_THRESHOLD_BYPASS       = 0x1c;     //Peak detection threshold bypass (R/W)
//    private static final int REG_I2C_CONFIG             = 0x1e;     //Default address response control (R/W)
//    private static final int REG_COMMAND                = 0x40;     //State command (R/W)
//    private static final int REG_MEASURE_DELAY          = 0x45;     //Delay between automatic measurement (R/W)
//    private static final int REG_PEAK_BCK               = 0x4c;     //Second largest peak value in correlation record (R)
//    private static final int REG_CORR_DATA              = 0x52;     //Correlation record data low byte (R)
//    private static final int REG_CORR_DATA_SIGN         = 0x53;     //Correlation record data high byte (R)
//    private static final int REG_ACQ_SETTINGS           = 0x5d;     //Correlation record memory bank select (R/W)
//    private static final int REG_POWER_CONTROL          = 0x65;     //Power state control (R/W)

    // REG_ACQ_COMMAND: Device command (W)
    private static final byte ACQCMD_RESET              = (byte)0x00;//Reset FPGA, all registers return to default value
    private static final byte ACQCMD_DISTANCE_NO_BIAS   = (byte)0x03;//Take distance measurement without receiver bias correction
    private static final byte ACQCMD_DISTANCE_WITH_BIAS = (byte)0x04; //Take distance measurement with receiver bias correction
    // REG_STATUS: Status (R)
//    private static final byte STATUS_BUSY               = (byte)0x01;//Device is busy taking a measurement.
//    private static final byte STATUS_REFERENCE_OVERFLOW = (byte)0x02;//Reference data in correlation record has reached max before overflow
//    private static final byte STATUS_SIGNAL_OVERFLOW    = (byte)0x04;//Signal data in correlation record has reached max before overflow
//    private static final byte STATUS_INVALID_SIGNAL     = (byte)0x08;//Peak not detected in correlation record, measurement is invalid
//    private static final byte STATUS_SECONDARY_RETURN   = (byte)0x10;//Secondary return detected in correlation record
//    private static final byte STATUS_HEALTH             = (byte)0x20;//Reference and receiver bias are operational
//    private static final byte STATUS_PROCESS_ERROR      = (byte)0x40;//System error detected during measurement
    // REG_SIG_COUNT_VAL: Maximum acquisition count (R/W) [default: 0x80]
    // REG_ACQ_CONFIG: Acquisition mode control (R/W) [default: 0x08]
//    private static final byte ACQCFG_MODE_MASK          = (byte)0x03;
//    private static final byte ACQCFG_PWM                = (byte)0x00;//Default PWM mode
//    private static final byte ACQCFG_STATUS_OUTPUT      = (byte)0x01;//Status output mode
//    private static final byte ACQCFG_FIXED_DELAY_PWM    = (byte)0x02;//Fixed delay PWM mode
//    private static final byte ACQCFG_OSCILLATOR_OUTPUT  = (byte)0x03;//Oscillator output mode
//    private static final byte ACQCFG_USE_REF_COUNT_VAL  = (byte)0x04;//Use REF_COUNT_VAL as reference acquisition count
//    private static final byte ACQCFG_DIS_MEA_QUICK_TERM = (byte)0x08;//Disable measurement quick termination
//    private static final byte ACQCFG_DISABLE_REF_FILTER = (byte)0x10;//Disable reference filter
//    private static final byte ACQCFG_USE_MEASURE_DELAY  = (byte)0x20;//Use MEASURE_DELAY for burst and free running mode
//    private static final byte ACQCFG_DISABLE_REF_PROCESS= (byte)0x40;//Disable reference process during measurement
    // REG_VELOCITY: Velocity measurement output (R)
    // REG_PEAK_CORR: Peak value in correlation record (R)
    // REG_NOISE_PEAK: Correlation record noise floor (R)
    // REG_SIGNAL_STRENGTH: Received signal strength (R)
    // REG_FULL_DELAY_HIGH: Distance measurement high byte (R)
    // REG_FULL_DELAY_LOW: Distance measurement low byte (R)
    // REG_OUTER_LOOP_COUNT: Burst measure count control (R/W) [default: 0x01]
    // REG_REF_COUNT_VAL: Reference acquisition count (R/W) [default: 0x05]
    // REG_LAST_DELAY_HIGH: Previous distance measurement high byte (R)
    // REG_LAST_DELAY_LOW: Previous distance measurement low byte (R)
    // REG_UNIT_ID_HIGH: Serial number high byte (R)
    // REG_UNIT_ID_LOW: Serial number low byte (R)
    // REG_I2C_ID_HIGH: Write serial number high byte for I2C address unlock (W)
    // REG_I2C_ID_LOW: Write serial number low byte for I2C address unlock (W)
    // REG_I2C_SEC_ADDR: Write new I2C address after unlock (R/W)
    // REG_THRESHOLD_BYPASS: Peak detection threshold bypass (R/W) [default: 0x00]
    // REG_I2C_CONFIG: Default address response control (R/W) [default: 0x00]
//    private static final byte I2CCFG_RESPOND_NON_DEF_ADDR= (byte)0x08;//Device will only respbnd to non-default I2C address
    // REG_COMMAND: State command (R/W)
//    private static final byte CMD_TEST_MODE_DISABLE     = (byte)0x00;
//    private static final byte CMD_TEST_MODE_ENABLE      = (byte)0x07;
    // REG_MEASURE_DELAY: Delay between automatic measurement (R/W) [default: 0x14]
    // REG_PEAK_BCK: Second largest peak value in correlation record (R)
    // REG_CORR_DATA: Correlation record data low byte (R)
    // REG_CORR_DATA_SIGN: Correlation record data high byte (R)
    // REG_ACQ_SETTINGS: Correlation record memory bank select (R/W)
//    private static final byte ACQSETTINGS_ACCESS_MEM_BANK= (byte)0xc0;//Access correlation memory bank
    // REG_POWER_CONTROL: Power state control (R/W) [default: 0x80]
//    private static final byte PWRCTRL_DISABLE_RECEIVER  = (byte)0x01;//Disable receiver circuit
//    private static final byte PWRCTRL_DEVICE_SLEEP      = (byte)0x04;//Device Sleep, wakes upon I2C transaction

    private final TrcSerialBusDevice device;
    private boolean started = false;

    private TrcSensor.SensorData<Double> distance = new TrcSensor.SensorData<>(0.0, null);
//    private int status = 0;
//    private int signalCount = 0;
//    private int acqConfig = 0;
//    private int velocity = 0;
//    private int peakCorrelation = 0;
//    private int noisePeak = 0;
//    private int signalStrength = 0;
//    private int distance = 0;
//    private int burstMeasureCountCtrl = 0;
//    private int referenceAcquistionCount = 0;
//    private int prevDistance = 0;
//    private int serialNumber = 0;
//    private int i2cAddress = 0;
//    private int thresholdBypass = 0;
//    private int defAddressResponseControl = 0;
//    private int command = 0;
//    private int autoMeasureDelay = 0;
//    private int secPeakCorrelation = 0;
//    private int correlationData = 0;
//    private int correlationMemoryBankSelect = 0;
//    private int powerControl = 0;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param device specifies the device object.
     */
    public TrcLidarLite(String instanceName, TrcSerialBusDevice device)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.device = device;
    }   //TrcLidarLite

    /**
     * This method starts the reading device data by queuing the initial read request if not already.
     */
    public void start()
    {
        if (!started)
        {
            dbgTrace.traceInfo("Start", "Starting Lidar");
            started = true;
            writeAcquisitionCommand(ACQCMD_DISTANCE_WITH_BIAS);
        }
    }   //start

 /**
  * This method writes an acquisition command to the device.
  *
  * @param command specifies the acquisition command.
  */
 private void writeAcquisitionCommand(byte command)
 {
     final String funcName = "writeAcquisitionCommand";

     if (debugEnabled)
     {
         dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "command=0x%02x", command);
     }

     RequestId requestId;
     byte[] data = new byte[1];
     data[0] = command;

     switch (command)
     {
         case ACQCMD_DISTANCE_NO_BIAS:
         case ACQCMD_DISTANCE_WITH_BIAS:
             requestId = RequestId.READ_DISTANCE;
             break;

         case ACQCMD_RESET:
         default:
             requestId = null;
             break;
     }

     dbgTrace.traceInfo(funcName, "Id=%s,data=%s", requestId, Arrays.toString(data));
     device.asyncWrite(requestId, REG_ACQ_COMMAND, data, data.length, null, this::notify);

     if (debugEnabled)
     {
         dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
     }
 }   //writeAcquistionCommand

    /**
     * This method returns the distance measurement data with bias correction.
     *
     * @return distance measurement data in cm.
     */
    public TrcSensor.SensorData<Double> getDistance()
    {
        final String funcName = "getDistance";
//        if (distance == null) throw new RuntimeException("distance is null");
//        if (distance.value == null) throw new RuntimeException("value is null");
        TrcSensor.SensorData<Double> data = new TrcSensor.SensorData<>(
            distance.timestamp, distance.value == null? 0: distance.value*TrcUtil.INCHES_PER_CM);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                "=(timestamp=%.3f,value=%.0f)", data.timestamp, data.value);
        }

        return data;
    }   //getDistance

//    /**
//     * Constructor: Creates an instance of the object.
//     *
//     * @param instanceName specifies the instance name.
//     * @param i2cAddress specifies the I2C address of the device.
//     * @param addressIs7Bit specifies true if the I2C address is a 7-bit address, false if it is 8-bit.
//     */
//    public TrcLidarLite(String instanceName, int i2cAddress, boolean addressIs7Bit)
//    {
//        this(FtcOpMode.getInstance().hardwareMap, instanceName, i2cAddress, addressIs7Bit);
//    }   //FtcZXDistanceSensor
//
//    /**
//     * Constructor: Creates an instance of the object.
//     *
//     * @param instanceName specifies the instance name.
//     */
//    public TrcLidarLite(String instanceName)
//    {
//        this(instanceName, DEF_I2CADDRESS, false);
//    }   //FtcZXDistanceSensor
//
//    /**
//     * This method returns the data from the Status register.
//     *
//     * @return status register data.
//     */
//    public int getStatus()
//    {
//        final String funcName = "getStatus";
//        long currTagId = FtcOpMode.getLoopCounter();
//
//        if (currTagId != dataTagId)
//        {
//            byte[] data;
//
//            deviceStatus = TrcUtil.bytesToInt(getData(statusReaderId)[0]);
//            if ((deviceStatus & STATUS_GESTURES) != 0)
//            {
//                data = getData(gestureReaderId);
//                gesture.timestamp = getDataTimestamp(gestureReaderId);
//                gesture.value = Gesture.getGesture(TrcUtil.bytesToInt(data[0]));
//
//                data = getData(gspeedReaderId);
//                gestureSpeed.timestamp = getDataTimestamp(gspeedReaderId);
//                gestureSpeed.value = (double)TrcUtil.bytesToInt(data[0]);
//            }
//
//            if ((deviceStatus & STATUS_DAV) != 0)
//            {
//                data = getData(xposReaderId);
//                xPos.timestamp = getDataTimestamp(xposReaderId);
//                xPos.value = (double)TrcUtil.bytesToInt(data[0]);
//
//                data = getData(zposReaderId);
//                zPos.timestamp = getDataTimestamp(zposReaderId);
//                zPos.value = (double)TrcUtil.bytesToInt(data[0]);
//
//                data = getData(lrngReaderId);
//                leftRangingData.timestamp = getDataTimestamp(lrngReaderId);
//                leftRangingData.value = (double)TrcUtil.bytesToInt(data[0]);
//
//                data = getData(rrngReaderId);
//                rightRangingData.timestamp = getDataTimestamp(rrngReaderId);
//                rightRangingData.value = (double)TrcUtil.bytesToInt(data[0]);
//            }
//
//            if (debugEnabled)
//            {
//                dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
//                dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%x", deviceStatus);
//            }
//
//            dataTagId = currTagId;
//        }
//
//        return deviceStatus;
//    }   //getStatus
//
//    /**
//     * This method returns the detected gesture type.
//     *
//     * @return detected gesture type.
//     */
//    public TrcSensor.SensorData<Gesture> getGesture()
//    {
//        final String funcName = "getGesture";
//        getStatus();
//        TrcSensor.SensorData<Gesture> data = new TrcSensor.SensorData<>(gesture.timestamp, gesture.value);
//
//        if (debugEnabled)
//        {
//            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
//            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
//                               "=(timestamp=%.3f,value=%s)", data.timestamp, data.value.toString());
//        }
//
//        return data;
//    }   //getGesture
//
//    /**
//     * This method returns the data from the Gesture Speed register.
//     *
//     * @return gesture speed.
//     */
//    public TrcSensor.SensorData<Double> getGestureSpeed()
//    {
//        final String funcName = "getGestureSpeed";
//        getStatus();
//        TrcSensor.SensorData<Double> data = new TrcSensor.SensorData<>(gestureSpeed.timestamp, gestureSpeed.value);
//
//        if (debugEnabled)
//        {
//            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
//            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
//                               "=(timestamp=%.3f,value=%.0f)", data.timestamp, data.value);
//        }
//
//        return data;
//    }   //getGestureSpeed
//
//    /**
//     * This method returns the data from teh X Position register.
//     *
//     * @return X position.
//     */
//    public TrcSensor.SensorData<Double> getX()
//    {
//        final String funcName = "getX";
//        getStatus();
//        TrcSensor.SensorData<Double> data = new TrcSensor.SensorData<>(xPos.timestamp, xPos.value);
//
//        if (debugEnabled)
//        {
//            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
//            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
//                               "=(timestamp=%.3f,value=%.0f)", data.timestamp, data.value);
//        }
//
//        return data;
//    }   //getX
//
//    /**
//     * This method returns the data from teh Z Position register.
//     *
//     * @return Z position.
//     */
//    public TrcSensor.SensorData<Double> getZ()
//    {
//        final String funcName = "getZ";
//        getStatus();
//        TrcSensor.SensorData<Double> data = new TrcSensor.SensorData<>(zPos.timestamp, zPos.value);
//
//        if (debugEnabled)
//        {
//            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
//            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
//                               "=(timestamp=%.3f,value=%.0f)", data.timestamp, data.value);
//        }
//
//        return data;
//    }   //getZ
//
//    /**
//     * This method returns the data from the Left Ranging Data register.
//     *
//     * @return left ranging data.
//     */
//    public TrcSensor.SensorData<Double> getLeftRangingData()
//    {
//        final String funcName = "getLeftRangingData";
//        getStatus();
//        TrcSensor.SensorData<Double> data = new TrcSensor.SensorData<>(
//                leftRangingData.timestamp, leftRangingData.value);
//
//        if (debugEnabled)
//        {
//            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
//            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
//                               "=(timestamp=%.3f,value=%.0f)", data.timestamp, data.value);
//        }
//
//        return data;
//    }   //getLeftRangingData
//
//    /**
//     * This method returns the data from the Right Ranging Data register.
//     *
//     * @return right ranging data.
//     */
//    public TrcSensor.SensorData<Double> getRightRangingData()
//    {
//        final String funcName = "getRightRangingData";
//        getStatus();
//        TrcSensor.SensorData<Double> data = new TrcSensor.SensorData<>(
//                rightRangingData.timestamp, rightRangingData.value);
//
//        if (debugEnabled)
//        {
//            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
//            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
//                               "=(timestamp=%.3f,value=%.0f)", data.timestamp, data.value);
//        }
//
//        return data;
//    }   //getRightRangingData
//
//    /**
//     * This method returns the data from the Register Map Version register.
//     *
//     * @return register map version.
//     */
//    public int getRegMapVersion()
//    {
//        final String funcName = "getRegMapVersion";
//
//        if (debugEnabled)
//        {
//            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
//            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%x", regMapVersion);
//        }
//
//        return regMapVersion;
//    }   //getRegMapVersion
//
//    /**
//     * This method returns the data from the Model Version register.
//     *
//     * @return model version.
//     */
//    public int getModelVersion()
//    {
//        final String funcName = "getModelVersion";
//
//        if (debugEnabled)
//        {
//            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
//            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%x", modelVersion);
//        }
//
//        return modelVersion;
//    }   //getModelVersion
//
//    //
//    // Implements TrcSensor.DataSource interface.
//    //
//
//    /**
//     * This method returns the sensor data of the specified index.
//     *
//     * @param index specifies the data index.
//     * @param dataType specifies the data type.
//     * @return sensor data of the specified index and type.
//     */
//    @Override
//    public TrcSensor.SensorData getRawData(int index, DataType dataType)
//    {
//        final String funcName = "getRawData";
//        TrcSensor.SensorData data = null;
//
//        switch (dataType)
//        {
//            case GESTURE:
//                data = getGesture();
//                break;
//
//            case GESTURE_SPEED:
//                data = getGestureSpeed();
//                break;
//
//            case X:
//                data = getX();
//                break;
//
//            case Z:
//                data = getZ();
//                break;
//
//            case LEFT_RANGING_DATA:
//                data = getLeftRangingData();
//                break;
//
//            case RIGHT_RANGING_DATA:
//                data = getRightRangingData();
//                break;
//        }
//
//        if (debugEnabled)
//        {
//            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "index=%d", index);
//            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
//                               "=(time=%.3f,value=%s)", data.timestamp, data.value.toString());
//        }
//
//        return data;
//    }   //getRawData
//    /**
//     * This method processes the data from the read completion handler.
//     *
//     * @param requestId specifies the ID to identify the request. Can be null if none was provided.
//     * @param data specifies the data read.
//     * @param length specifies the number of bytes read.
//     */
//    private void processData(RequestId requestId, byte[] data, int length)
//    {
//        final String funcName = "processData";
//        int word;
//
//        if (debugEnabled)
//        {
//            dbgTrace.traceVerbose(funcName, "Id=%s,data=%s,len=%d", requestId, Arrays.toString(data), length);
//        }
//
//        switch (requestId)
//        {
//            case STATUS:
//                break;
//
//            default:
//                //
//                // We should never come here. Let's throw an exception to catch this unlikely scenario.
//                //
//                throw new IllegalStateException(String.format("Unexpected request ID %s.", requestId));
//        }
//    }   //processData

    /**
     * This method is called when the read request is completed.
     *
     * @param context specifies the read request.
     */
    public void notify(Object context)
    {
        final String funcName = "notify";
        TrcSerialBusDevice.Request request = (TrcSerialBusDevice.Request) context;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.CALLBK, "request=%s", request);
        }

        if (request.readRequest)
        {
            if (request.buffer != null)
            {
                switch ((RequestId)request.requestId)
                {
                    case READ_DISTANCE:
                        if ((request.buffer[0] & 0x1) == 0x1)
                        {
                            // Not ready yet, read status again.
                            device.asyncRead(request.requestId, REG_STATUS, 1, null, this::notify);
                        }
                        else
                        {
                            device.asyncRead(RequestId.GET_DISTANCE, REG_FULL_DELAY_HIGH, 2, null, this::notify);
                        }
                        break;

                    case GET_DISTANCE:
                        distance.timestamp = TrcUtil.getCurrentTime();
                        distance.value = (double)TrcUtil.bytesToInt(request.buffer[1], request.buffer[0]);
                        break;

                    default:
                        break;
                }
            }
        }
        else
        {
            switch ((RequestId)request.requestId)
            {
                case READ_DISTANCE:
                    device.asyncRead(request.requestId, REG_STATUS, 1, null, this::notify);
                    break;

                default:
                    break;
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.CALLBK);
        }
    }   //notify

}   //class TrcLidarLite
