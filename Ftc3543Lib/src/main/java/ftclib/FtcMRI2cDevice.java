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
import trclib.TrcUtil;

/**
 * This class implements the common features of all Modern Robotics I2C devices. Typically, this class will be
 * extended by specific Modern Robotics I2C devices.
 */
public class FtcMRI2cDevice extends FtcI2cDevice
{
    private static final String moduleName = "FtcMRI2cDevice";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
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
     * @param addressIs7Bit specifies true if the I2C address is a 7-bit address, false if it is 8-bit.
     */
    public FtcMRI2cDevice(HardwareMap hardwareMap, String instanceName, int i2cAddress, boolean addressIs7Bit)
    {
        super(hardwareMap, instanceName, i2cAddress, addressIs7Bit);

        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        byte[] data = syncRead(HEADER_START, HEADER_LENGTH);
        firmwareRev = TrcUtil.bytesToInt(data[REG_FIRMWARE_REVISION - HEADER_START]);
        manufacturerCode = TrcUtil.bytesToInt(data[REG_MANUFACTURER_CODE - HEADER_START]);
        idCode = TrcUtil.bytesToInt(data[REG_ID_CODE - HEADER_START]);
    }   //FtcMRI2cDevice

    /**
     * This method changes the I2C address of the Modern Robotics device.
     *
     * @param newAddress specifies the new I2C address.
     * @param addressIs7Bit specifies true if the I2C address is a 7-bit address, false if it is 8-bit.
     */
    public void changeI2cAddress(int newAddress, boolean addressIs7Bit)
    {
        final String funcName = "changeI2cAddress";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "addr=%x", newAddress);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        byte[] data = {(byte)(newAddress & 0xff), I2CADDR_TRIGGER_BYTE_1, I2CADDR_TRIGGER_BYTE_2};
        asyncWrite(null, REG_SET_I2C_ADDRESS, data, data.length, null, null);
        deviceSynch.setI2cAddress(newAddress, addressIs7Bit);
    }   //changeI2cAddress

    /**
     * This method changes the I2C address of the Modern Robotics device.
     *
     * @param newAddress specifies the new I2C address.
     */
    public void changeI2cAddress(int newAddress)
    {
        changeI2cAddress(newAddress, false);
    }   //changeI2cAddress

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

}   //class FtcMRI2cDevice
