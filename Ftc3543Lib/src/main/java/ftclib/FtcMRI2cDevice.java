/*
 * Titan Robotics Framework Library
 * Copyright (c) 2016 Titan Robotics Club (http://www.titanrobotics.net)
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
import trclib.TrcUtil;

/**
 * This class implements the common features of all Modern Robotics I2C devices. Typically,
 * this class will be extended by specific Modern Robotics I2C devices.
 * It provides the TrcI2cDevice.CompletionHandler interface to read the common data.
 */
public class FtcMRI2cDevice extends FtcI2cDevice implements TrcI2cDevice.CompletionHandler
{
    private static final String moduleName = "FtcMRI2cDevice";
    private static final boolean debugEnabled = false;
    private TrcDbgTrace dbgTrace = null;

    //
    // I2C registers.
    //
    protected static final int REG_FIRMWARE_REVISION    = 0x00;
    protected static final int REG_MANUFACTURER_CODE    = 0x01;
    protected static final int REG_ID_CODE              = 0x02;
    protected static final int REG_COMMAND              = 0x03;
    protected static final int REG_SET_I2C_ADDRESS      = 0x70;

    private static final int HEADER_START               = REG_FIRMWARE_REVISION;
    private static final int HEADER_END                 = REG_ID_CODE;
    private static final int HEADER_LENGTH              = (HEADER_END - HEADER_START + 1);

    protected static final byte MANUFACTURER_CODE       = 0x4d;

    //
    // Set I2C Address.
    //
    private static final byte I2CADDR_TRIGGER_BYTE_1    = 0x55;
    private static final byte I2CADDR_TRIGGER_BYTE_2    = ((byte)0xaa);

    private int firmwareRev = 0;
    private int manufacturerCode = 0;
    private int idCode = 0;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     * @param i2cAddress specifies the I2C address of the device.
     */
    public FtcMRI2cDevice(HardwareMap hardwareMap, String instanceName, int i2cAddress)
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

        read(HEADER_START, HEADER_LENGTH, this);
    }   //FtcMRI2cColorSensor

    /**
     * This method sets a new I2C address to the Modern Robotics device.
     *
     * @param newAddress specifies the new I2C address.
     */
    public void setI2cAddress(byte newAddress)
    {
        final String funcName = "setI2cAddress";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "addr=%x", newAddress);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        byte[] data = {newAddress, I2CADDR_TRIGGER_BYTE_1, I2CADDR_TRIGGER_BYTE_2};
        write(REG_SET_I2C_ADDRESS, data.length, data);
        updateI2cAddress(newAddress);
    }   //setI2cAddress

    /**
     * This method returns the firmware revision.
     *
     * @return firmware revision number.
     */
    public int getFirmwareRevision()
    {
        final String funcName = "getFirmwareRevision";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%x", firmwareRev);
        }

        return firmwareRev;
    }   //getFirmwareRevision

    /**
     * This method returns the manufacturer code of the sensor.
     *
     * @return manufacturer code.
     */
    public int getManufacturerCode()
    {
        final String funcName = "getManufacturerCode";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%x", manufacturerCode);
        }

        return manufacturerCode;
    }   //getManufacturerCode

    /**
     * This method returns the ID code of the sensor.
     *
     * @return ID code.
     */
    public int getIdCode()
    {
        final String funcName = "getIdCode";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%x", idCode);
        }

        return idCode;
    }   //getManufacturerCode

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

        if (regAddress == HEADER_START && length == HEADER_LENGTH)
        {
            if (timedout)
            {
                repeat = true;
            }
            else
            {
                //
                // These only need to be read once, so no repeat.
                //
                firmwareRev = TrcUtil.bytesToInt(data[REG_FIRMWARE_REVISION - HEADER_START]);
                manufacturerCode =
                        TrcUtil.bytesToInt(data[REG_MANUFACTURER_CODE - HEADER_START]);
                idCode = TrcUtil.bytesToInt(data[REG_ID_CODE - HEADER_START]);
            }
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

}   //class FtcMRI2cDevice
