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

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceReader;

/**
 * This class extends I2cDeviceReader so we can add methods to retrieve different parameters of the reader.
 */
public class FtcI2cDeviceReader extends I2cDeviceReader
{
    private String instanceName;
    private I2cDevice device;
    private I2cAddr i2cAddr;
    private int memStart;
    private int memLen;
    private byte[] data = null;
    private double timestamp = 0.0;

    public FtcI2cDeviceReader(String instanceName, I2cDevice device, I2cAddr i2cAddr, int memStart, int memLen)
    {
        super(device, i2cAddr, memStart, memLen);

        this.instanceName = instanceName;
        this.device = device;
        this.i2cAddr = i2cAddr;
        this.memStart = memStart;
        this.memLen = memLen;
    }   //FtcI2cDeviceReader

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method returns the I2cDevice object associated with the reader.
     *
     * @return associated I2C device object.
     */
    public I2cDevice getDevice()
    {
        return device;
    }   //getDevice

    /**
     * This method returns the I2C address of the device.
     *
     * @return I2C address of the device.
     */
    public I2cAddr getI2cAddress()
    {
        return i2cAddr;
    }   //getI2cAddr

    /**
     * This method returns the starting memory address of the reader.
     *
     * @return starting memory address.
     */
    public int getMemStart()
    {
        return memStart;
    }   //getMemStart

    /**
     * This method returns the memory block length to be read by the reader.
     *
     * @return memory block length.
     */
    public int getMemLength()
    {
        return memLen;
    }   //getMemLength

    /**
     * This method checks if it is in the same time slice loop as before. If so, it will just returned the
     * data cached from last time. Otherwise, it will retrieve a new data buffer from the device and update
     * the cache with it.
     *
     * @return device data.
     */
    public byte[] getData()
    {
        double loopStartTime = FtcOpMode.getLoopStartTime();

        if (loopStartTime > timestamp)
        {
            data = getReadBuffer();
            timestamp = loopStartTime;
        }

        return data;
    }   //getData

    /**
     * This method returns the timestamp of the data cache.
     *
     * @return data cache timestamp.
     */
    public double getDataTimestamp()
    {
        return timestamp;
    }   //getDataTimestamp

}   //class FtcI2cDeviceReader
