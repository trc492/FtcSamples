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
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

import java.util.ArrayList;
import java.util.Arrays;

import trclib.TrcDbgTrace;

/**
 * This class implements a platform dependent I2C device.
 */
public class FtcI2cDevice
{
    private static final String moduleName = "FtcI2cDevice";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private I2cDevice device;
    private I2cAddr i2cAddr;
    private I2cDeviceSynchImpl syncDevice;
    private ArrayList<FtcI2cDeviceReader> readers = new ArrayList<>();
    private FtcI2cDeviceState deviceState;

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
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        device = hardwareMap.i2cDevice.get(instanceName);
        deviceState = new FtcI2cDeviceState(instanceName, device);
        setI2cAddress(i2cAddress, addressIs7Bit);
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
     * This method sets the I2C address of the device. This is typically called by the subclass to update the I2C
     * address after changing it.
     *
     * @param newAddress specifies the new I2C address.
     * @param addressIs7Bit specifies true if the I2C address is a 7-bit address, false if it is 8-bit.
     */
    protected void setI2cAddress(int newAddress, boolean addressIs7Bit)
    {
        final String funcName = "setI2cAddress";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "address=%x", newAddress);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        i2cAddr = addressIs7Bit? I2cAddr.create7bit(newAddress): I2cAddr.create8bit(newAddress);
        //
        // I2C address has changed, discard the old sync device and create a new one with updated address.
        //
        syncDevice = new I2cDeviceSynchImpl(device, i2cAddr, false);
        syncDevice.engage();
        //
        // Recreate all readers with the new I2C address if any.
        //
        for (int i = 0; i < readers.size(); i++)
        {
            FtcI2cDeviceReader reader = readers.get(i);
            readers.set(i, new FtcI2cDeviceReader(
                    reader.toString(), device, i2cAddr, reader.getMemStart(), reader.getMemLength()));
        }
    }   //setI2cAddress

    /**
     * This method check if the I2C device is enabled.
     *
     * @return true if the device state indicates it is enabled, false otherwise.
     */
    public boolean isDeviceEnabled()
    {
        final String funcName = "isDeviceEnabled";
        boolean enabled = deviceState.isEnabled();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Boolean.toString(enabled));
        }

        return enabled;
    }   //isDeviceEnabled

    /**
     * This method is called to enable/disable the I2C device so that it will not unnecessarily bog down the I2C bus
     * bandwidth if it is not needed.
     *
     * @param enabled specifies true to enable device, false otherwise.
     */
    public void setDeviceEnabled(boolean enabled)
    {
        final String funcName = "setDeviceEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enabled=%s", Boolean.toString(enabled));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        deviceState.setEnabled(enabled);
    }   //setDeviceEnabled

    /**
     * This method adds a device reader to read the specified block of memory.
     *
     * @param readerName specifies the instqance name of the reader.
     * @param memStart specifies the starting memory address to read from.
     * @param memLength specifies the length of the memory block to read.
     * @return ID of the new reader created.
     */
    public int addReader(String readerName, int memStart, int memLength)
    {
        final String funcName = "addReader";
        int readerId = readers.size();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                    "name=%s,start=0x%02x,len=%d", readerName, memStart, memLength);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        readers.add(readerId, new FtcI2cDeviceReader(readerName, device, i2cAddr, memStart, memLength));

        return readerId;
    }   //addReader

    /**
     * This method is doing a synchronous read from the device with the specified starting address and length of the
     * register block.
     *
     * @param startAddress specifies the starting register to read from.
     * @param length specifies the length of the register block to read.
     * @return data read.
     */
    public byte[] syncRead(int startAddress, int length)
    {
        final String funcName = "syncRead";
        byte[] data = syncDevice.read(startAddress, length);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "start=0x%02x,len=%d", startAddress, length);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Arrays.toString(data));
        }

        return data;
    }   //syncRead

    /**
     * This method is doing a synchronous write to the device with the specified starting address and data of the
     * register block.
     *
     * @param startAddress specifies the starting register to read from.
     * @param data specifies the data to write to the device.
     */
    public void syncWrite(int startAddress, byte[] data)
    {
        final String funcName = "syncWrite";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                                "start=0x%02x,data=%s", startAddress, Arrays.toString(data));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        syncDevice.write(startAddress, data, true);
    }   //syncWrite

    /**
     * This method is doing an asynchronous write to the device with the specified starting address and data of the
     * register block.
     *
     * @param startAddress specifies the starting register to read from.
     * @param data specifies the data to write to the device.
     */
    public void asyncWrite(int startAddress, byte[] data)
    {
        final String funcName = "asyncWrite";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                                "start=0x%02x,data=%s", startAddress, Arrays.toString(data));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        syncDevice.write(startAddress, data, false);
    }   //asyncWrite

    /**
     * This method sends a byte command to the device.
     *
     * @param regAddress specifies the register address to write to.
     * @param command specifies the command byte.
     * @param waitForCompletion specifies true to wait for write completion.
     */
    public void sendByteCommand(int regAddress, byte command, boolean waitForCompletion)
    {
        final String funcName = "sendByteCommand";
        byte[] data = new byte[1];

        data[0] = command;
        syncDevice.write(regAddress, data, waitForCompletion);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "command=%x", command);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //sendByteCommand

    /**
     * This method sends a 16-bit command to the device.
     *
     * @param regAddress specifies the register address to write to.
     * @param command specifies the 16-bit command.
     * @param waitForCompletion specifies true to wait for write completion.
     */
    public void sendWordCommand(int regAddress, short command, boolean waitForCompletion)
    {
        final String funcName = "sendWordCommand";
        byte[] data = new byte[2];

        data[0] = (byte)(command & 0xff);
        data[1] = (byte)(command >> 8);
        syncDevice.write(regAddress, data, waitForCompletion);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "command=%x", command);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //sendWordCommand

    /**
     * This method retrieves the data read from the specified reader.
     *
     * @param readerId specifies the reader ID for the reader to get the data from.
     * @return device data.
     */
    public byte[] getData(int readerId)
    {
        final String funcName = "getData";
        byte[] data = readers.get(readerId).getData();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Arrays.toString(data));
        }

        return data;
    }   //getData

    /**
     * This method retrieves the timestamp of the cached data from the specified reader.
     *
     * @param readerId specifies the reader ID for the reader to get the data from.
     * @return data timestamp.
     */
    public double getDataTimestamp(int readerId)
    {
        final String funcName = "getDataTimestamp";
        double timestamp = readers.get(readerId).getDataTimestamp();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%.3f", timestamp);
        }

        return timestamp;
    }   //getDataTimestamp

}   //class FtcI2cDevice
