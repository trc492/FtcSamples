/*
 * Titan Robotics Framework Library
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.net)
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
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;

import java.util.Arrays;

import trclib.TrcDbgTrace;
import trclib.TrcI2cDevice;

/**
 * This class implements a platform dependent I2C device.
 * extending TrcI2cDevice. It provides implementation of the
 * abstract methods in TrcI2cDevice.
 */
public class FtcI2cDevice extends TrcI2cDevice
{
    private static final String moduleName = "FtcI2cDevice";
    private static final boolean debugEnabled = false;
    private TrcDbgTrace dbgTrace = null;

    private I2cAddr i2cAddr;
    private I2cDevice device;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     * @param i2cAddress specifies the I2C address of the device.
     * @param addressIs7Bit specifies true if the I2C address is a 7-bit address, false if it is 8-bit.
     */
    public FtcI2cDevice(HardwareMap hardwareMap, String instanceName, int i2cAddress, boolean addressIs7Bit)
    {
        super(instanceName);

        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(
                    moduleName + "." + instanceName,
                    false,
                    TrcDbgTrace.TraceLevel.API,
                    TrcDbgTrace.MsgLevel.INFO);
        }

        updateI2cAddress(i2cAddress, addressIs7Bit);
        device = hardwareMap.i2cDevice.get(instanceName);
    }   //FtcI2cDevice

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     * @param i2cAddress specifies the I2C address of the device.
     */
    public FtcI2cDevice(HardwareMap hardwareMap, String instanceName, int i2cAddress)
    {
        this(hardwareMap, instanceName, i2cAddress, false);
    }   //FtcI2cDevice

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param i2cAddress specifies the I2C address of the device.
     */
    public FtcI2cDevice(String instanceName, int i2cAddress)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, i2cAddress, false);
    }   //FtcI2cDevice

    /**
     * This method updates the I2C address of the device to a new address. This is typically
     * called by the child class to update the I2C address after changing it.
     *
     * @param newAddress specifies the new I2C address.
     * @param addressIs7Bit specifies true if the I2C address is a 7-bit address, false if it is 8-bit.
     */
    protected void updateI2cAddress(int newAddress, boolean addressIs7Bit)
    {
        final String funcName = "updateI2cAddress";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "address=%x", newAddress);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (addressIs7Bit)
        {
            this.i2cAddr = I2cAddr.create7bit(newAddress);
        }
        else
        {
            this.i2cAddr = I2cAddr.create8bit(newAddress);
        }
    }   //updateI2cAddress

    //
    // Implements TrcI2cDevice abstract methods.
    //

    /**
     * This method checks if the I2C port is ready for bus transaction.
     *
     * @return true if port is ready, false otherwise.
     */
    @Override
    public boolean isPortReady()
    {
        final String funcName = "isPortReady";
        boolean ready = device.isI2cPortReady();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=%s", Boolean.toString(ready));
        }

        return ready;
    }   //isPortReady

    /**
     * This method checks if the I2C port is in write mode.
     *
     * @return true if port is in write mode, false otherwise.
     */
    @Override
    public boolean isPortInWriteMode()
    {
        final String funcName = "isPortInWriteMode";
        boolean writeMode = device.isI2cPortInWriteMode();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=%s", Boolean.toString(writeMode));
        }

        return writeMode;
    }   //isPortInWriteMode

    /**
     * This method sends the read command to the device.
     *
     * @param regAddress specifies the register address.
     * @param length specifies the number of bytes to read.
     */
    @Override
    public void sendReadCommand(int regAddress, int length)
    {
        final String funcName = "sendReadCommand";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                                "addr=%x,len=%d", regAddress, length);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        device.enableI2cReadMode(i2cAddr, regAddress, length);
        device.setI2cPortActionFlag();
        device.writeI2cCacheToController();
    }   //sendReadCommand

    /**
     * This method sends the write command to the device.
     *
     * @param regAddress specifies the register address.
     * @param length specifies the number of bytes to write.
     * @param data specifies the data buffer containing the data to write to the device.
     */
    @Override
    public void sendWriteCommand(int regAddress, int length, byte[] data)
    {
        final String funcName = "sendWriteCommand";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                                "addr=%x,len=%d", regAddress, length);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        device.enableI2cWriteMode(i2cAddr, regAddress, length);
        device.copyBufferIntoWriteBuffer(data);
        device.setI2cPortActionFlag();
        device.writeI2cCacheToController();
    }   //sendWriteCommand

    /**
     * This method retrieves the data read from the device.
     *
     * @return byte array containing the data read.
     */
    @Override
    public byte[] getData()
    {
        final String funcName = "getData";

        device.readI2cCacheFromController();
        byte[] data = device.getCopyOfReadBuffer();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Arrays.toString(data));
        }

        return data;
    }   //getData

}   //class FtcI2cDevice
