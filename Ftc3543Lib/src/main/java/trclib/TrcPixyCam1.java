/*
 * Copyright (c) 2017 Titan Robotics Club (http://www.titanrobotics.com)
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

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Locale;

/**
 * This class implements a platform independent pixy camera 1. This class is intended to be extended by a platform
 * dependent pixy class which provides the abstract methods required by this class. This class provides the parser
 * to read and parse the object block from the pixy camera 1. It also provides access to the last detected objects
 * reported by the pixy camera asynchronously.
 */
public abstract class TrcPixyCam1
{
    protected static final String moduleName = "TrcPixyCam1";
    protected static final boolean debugEnabled = false;
    protected static final boolean tracingEnabled = false;
    protected static final boolean useGlobalTracer = false;
    protected static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    protected static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    protected TrcDbgTrace dbgTrace = null;

    private static final byte PIXY_SYNC_HIGH                    = (byte)0xaa;
    private static final int PIXY_START_WORD                    = 0xaa55;
    private static final int PIXY_START_WORD_CC                 = 0xaa56;
    private static final int PIXY_START_WORDX                   = 0x55aa;

    private static final byte PIXY_CMD_SET_LED                  = (byte)0xfd;
    private static final byte PIXY_CMD_SET_BRIGHTNESS           = (byte)0xfe;
    private static final byte PIXY_CMD_SET_PAN_TILT             = (byte)0xff;

    /**
     * This method issues an asynchronous read of the specified number of bytes from the device.
     *
     * @param requestId specifies the ID to identify the request for the request handler. Can be null if none was
     *                  provided.
     * @param length specifies the number of bytes to read.
     */
    public abstract void asyncReadData(RequestId requestId, int length);

    /**
     * This method writes the data buffer to the device asynchronously.
     *
     * @param requestId specifies the ID to identify the request for the request handler. Can be null if none was
     *                  provided.
     * @param data specifies the data buffer.
     */
    public abstract void asyncWriteData(RequestId requestId, byte[] data);

    /**
     * This class implements the pixy camera object block communication protocol. 
     */
    public class ObjectBlock
    {
        public int sync;
        public int checksum;
        public int signature;
        public int centerX;
        public int centerY;
        public int width;
        public int height;
        public int angle;

        @Override
        public String toString()
        {
            return String.format(
                Locale.US, "sync=0x%04x, chksum=0x%04x, sig=%d, centerX=%3d, centerY=%3d, width=%3d, height=%3d, " +
                            "angle=%3d",
                sync, checksum, signature, centerX, centerY, width, height, angle);
        }
    }   //class ObjectBlock

    /**
     * This is used identify the request type.
     */
    public enum RequestId
    {
        SYNC,
        ALIGN,
        CHECKSUM,
        NORMAL_BLOCK,
        COLOR_CODE_BLOCK
    }   //enum RequestId

    private final String instanceName;
    private final boolean msbFirst;
    private ArrayList<ObjectBlock> objects = new ArrayList<>();
    private ObjectBlock[] detectedObjects = null;
    private ObjectBlock currBlock = null;
    private boolean started = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param msbFirst specifies true if a word has MSB first.
     */
    public TrcPixyCam1(final String instanceName, boolean msbFirst)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
        this.msbFirst = msbFirst;
    }   //TrcPixyCam1

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @Override
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method starts the pixy camera by queuing the initial read request if not already.
     */
    public void start()
    {
        if (!started)
        {
            started = true;
            asyncReadData(RequestId.SYNC, 2);
        }
    }   //start

    /**
     * This method sets the pixy camera to end state so it will start properly next time.
     */
    public void end()
    {
        started = false;
    }   //end

    /**
     * This method writes the data to the device one byte at a time.
     *
     * @param data specifies the buffer containing the data to be written to the device.
     */
    public void asyncWriteBytes(byte[] data)
    {
        byte[] byteData = new byte[1];

        for (int i = 0; i < data.length; i++)
        {
            byteData[0] = data[i];
            asyncWriteData(null, byteData);
        }
    }   //asyncWriteBytes

    /**
     * This method sets the LED to the specified color.
     *
     * @param red specifies the red value.
     * @param green specifies the green value.
     * @param blue specifies the blue value.
     */
    public void setLED(byte red, byte green, byte blue)
    {
        final String funcName = "setLED";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "red=%d,green=%d,blue=%d", red, green, blue);
        }

        byte[] data = new byte[5];
        data[0] = 0x00;
        data[1] = PIXY_CMD_SET_LED;
        data[2] = red;
        data[3] = green;
        data[4] = blue;

        asyncWriteData(null, data);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setLED

    /**
     * This method sets the camera brightness.
     *
     * @param brightness specifies the brightness value.
     */
    public void setBrightness(byte brightness)
    {
        final String funcName = "setBrightness";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "brightness=%d", brightness);
        }

        byte[] data = new byte[3];
        data[0] = 0x00;
        data[1] = PIXY_CMD_SET_BRIGHTNESS;
        data[2] = brightness;

        asyncWriteData(null, data);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setBrightness

    /**
     * This method sets the pan and tilt servo positions.
     * @param pan specifies the pan position between 0 and 1000.
     * @param tilt specifies the tilt position between 0 and 1000.
     */
    public void setPanTilt(int pan, int tilt)
    {
        final String funcName = "setPanTilt";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "pan=%d,tilt=%d", pan, tilt);
        }

        if (pan < 0 || pan > 1000 || tilt < 0 || tilt > 1000)
        {
            throw new IllegalArgumentException("Invalid pan/tilt range.");
        }

        byte[] data = new byte[6];
        data[0] = 0x00;
        data[1] = PIXY_CMD_SET_PAN_TILT;
        data[2] = (byte)(pan & 0xff);
        data[3] = (byte)(pan >> 8);
        data[4] = (byte)(tilt & 0xff);
        data[5] = (byte)(tilt >> 8);

        asyncWriteData(null, data);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setPanTilt

    /**
     * This method returns an array of detected object blocks.
     *
     * @return array of detected object blocks, can be null if no object detected or result of the next frame
     *         not yet available.
     */
    public ObjectBlock[] getDetectedObjects()
    {
        final String funcName = "getDetectedObjects";
        ObjectBlock[] objectBlocks;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        synchronized (this)
        {
            objectBlocks = detectedObjects;
            detectedObjects = null;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        return objectBlocks;
    }   //getDetectedObjects

    /**
     * This method processes the data from the read completion handler.
     *
     * @param requestId specifies the ID to identify the request. Can be null if none was provided.
     * @param data specifies the data read.
     * @param length specifies the number of bytes read.
     */
    private void processData(RequestId requestId, byte[] data, int length)
    {
        final String funcName = "processData";
        int word;

        if (debugEnabled)
        {
            dbgTrace.traceVerbose(funcName, "Id=%s,data=%s,len=%d", requestId, Arrays.toString(data), length);
        }

        switch (requestId)
        {
            case SYNC:
                //
                // If we don't already have an object block allocated, allocate it now.
                //
                if (currBlock == null)
                {
                    currBlock = new ObjectBlock();
                }

                if (length != 2)
                {
                    //
                    // We should never get here. But if we do, probably due to device read failure, we will initiate
                    // another read for SYNC.
                    //
                    asyncReadData(RequestId.SYNC, 2);
                    if (debugEnabled)
                    {
                        dbgTrace.traceWarn(funcName, "Unexpected data length %d in %s", length, requestId);
                    }
                }
                else
                {
                    word = getWord(data[0], data[1], msbFirst);
                    if (word == PIXY_START_WORD || word == PIXY_START_WORD_CC)
                    {
                        //
                        // Found a sync word, initiate the read for CHECKSUM.
                        //
                        currBlock.sync = word;
                        asyncReadData(RequestId.CHECKSUM, 2);
                    }
                    else if (word == PIXY_START_WORDX)
                    {
                        //
                        // We are word misaligned. Realign it by reading one byte and expecting it to be the high
                        // sync byte.
                        //
                        currBlock.sync = PIXY_START_WORD;
                        asyncReadData(RequestId.ALIGN, 1);
                        if (debugEnabled)
                        {
                            dbgTrace.traceInfo(funcName, "Word misaligned, realigning...");
                        }
                    }
                    else
                    {
                        //
                        // We don't find the sync word, throw it away and initiate another read for SYNC.
                        //
                        asyncReadData(RequestId.SYNC, 2);
                        if (debugEnabled)
                        {
                            if (word != 0)
                            {
                                dbgTrace.traceWarn(funcName, "Unexpected word 0x%04x read in %s", word, requestId);
                            }
                        }
                    }
                }
                break;

            case ALIGN:
                if (length != 1)
                {
                    //
                    // We should never come here. Let's throw an exception to catch this unlikely scenario.
                    //
                    throw new IllegalStateException(String.format("Unexpected data length %d in %s.",
                        length, requestId));
                }
                else if (data[0] == PIXY_SYNC_HIGH)
                {
                    //
                    // Found the expected upper sync byte, so initiate the read for CHECKSUM.
                    //
                    asyncReadData(RequestId.CHECKSUM, 2);
                }
                else
                {
                    //
                    // Don't see the expected upper sync byte, let's initiate another read for SYNC assuming we are
                    // now word aligned again.
                    //
                    asyncReadData(RequestId.SYNC, 2);
                    if (debugEnabled)
                    {
                        dbgTrace.traceWarn(funcName, "Unexpected data byte 0x%02x in %s", data[0], requestId);
                    }
                }
                break;

            case CHECKSUM:
                if (length != 2)
                {
                    //
                    // We should never come here. Let's throw an exception to catch this unlikely scenario.
                    //
                    throw new IllegalStateException(String.format("Unexpected data length %d in %s.",
                        length, requestId));
                }
                else
                {
                    word = getWord(data[0], data[1], msbFirst);
                    if (word == PIXY_START_WORD || word == PIXY_START_WORD_CC)
                    {
                        //
                        // We were expecting a checksum but found a sync word. It means that's the end-of-frame.
                        // Save away the sync word for the next frame and initiate the next read for CHECKSUM.
                        //
                        currBlock.sync = word;
                        asyncReadData(RequestId.CHECKSUM, 2);
                        //
                        // Detected end-of-frame, convert the array list of objects into detected object array.
                        //
                        if (objects.size() > 0)
                        {
                            synchronized (this)
                            {
                                ObjectBlock[] array = new ObjectBlock[objects.size()];
                                detectedObjects = objects.toArray(array);
                                objects.clear();
                                if (debugEnabled)
                                {
                                    for (int i = 0; i < detectedObjects.length; i++)
                                    {
                                        dbgTrace.traceInfo(funcName, "[%02d] %s", i, detectedObjects[i].toString());
                                    }
                                }
                            }
                        }
                    }
                    else
                    {
                        //
                        // Looks like we have a checksum, save it away and initiate the read for the rest of the
                        // block. If the sync word was PIXY_START_WORD, then it is a 10-byte NORMAL_BLOCK, else it
                        // is a 12-byte COLOR_CODE_BLOCK.
                        //
                        currBlock.checksum = word;
                        if (currBlock.sync == PIXY_START_WORD)
                        {
                            asyncReadData(RequestId.NORMAL_BLOCK, 10);
                        }
                        else if (currBlock.sync == PIXY_START_WORD_CC)
                        {
                            asyncReadData(RequestId.COLOR_CODE_BLOCK, 12);
                        }
                        else
                        {
                            //
                            // We should never come here. Let's throw an exception to catch this unlikely scenario.
                            //
                            throw new IllegalStateException(String.format("Unexpected sync word 0x%04x in %s.",
                                currBlock.sync, requestId));
                        }
                    }
                }
                break;

            case NORMAL_BLOCK:
            case COLOR_CODE_BLOCK:
                if (requestId == RequestId.NORMAL_BLOCK && length != 10 ||
                    requestId == RequestId.COLOR_CODE_BLOCK && length != 12)
                {
                    //
                    // We should never come here. Let's throw an exception to catch this unlikely scenario.
                    //
                    throw new IllegalStateException(String.format("Unexpected data length %d in %s.",
                        length, requestId));
                }
                else
                {
                    int index;
                    int runningChecksum = 0;
                    //
                    // Save away the signature and accumulate checksum.
                    //
                    index = 0;
                    word = getWord(data[index], data[index + 1], msbFirst);
                    runningChecksum += word;
                    currBlock.signature = word;
                    //
                    // Save away the object center X and accumulate checksum.
                    //
                    index += 2;
                    word = getWord(data[index], data[index + 1], msbFirst);
                    runningChecksum += word;
                    currBlock.centerX = word;
                    //
                    // Save away the object center Y and accumulate checksum.
                    //
                    index += 2;
                    word = getWord(data[index], data[index + 1], msbFirst);
                    runningChecksum += word;
                    currBlock.centerY = word;
                    //
                    // Save away the object width and accumulate checksum.
                    //
                    index += 2;
                    word = getWord(data[index], data[index + 1], msbFirst);
                    runningChecksum += word;
                    currBlock.width = word;
                    //
                    // Save away the object height and accumulate checksum.
                    //
                    index += 2;
                    word = getWord(data[index], data[index + 1], msbFirst);
                    runningChecksum += word;
                    currBlock.height = word;
                    //
                    // If it is a COLOR_CODE_BLOCK, save away the object angle and accumulate checksum.
                    //
                    if (requestId == RequestId.COLOR_CODE_BLOCK)
                    {
                        index += 2;
                        word = getWord(data[index], data[index + 1], msbFirst);
                        runningChecksum += word;
                        currBlock.angle = word;
                    }

                    if (runningChecksum == currBlock.checksum)
                    {
                        //
                        // Checksum is correct, add the object block.
                        //
                        if (debugEnabled)
                        {
                            dbgTrace.traceInfo(funcName, "Object(%s)", currBlock);
                        }
                        objects.add(currBlock);
                        currBlock = null;
                    }
                    else if (debugEnabled)
                    {
                        dbgTrace.traceWarn(funcName, "Incorrect checksum %d (expecting %d).",
                            runningChecksum, currBlock.checksum);
                    }
                    //
                    // Initiate the read for the SYNC word of the next block.
                    //
                    asyncReadData(RequestId.SYNC, 2);
                }
                break;

            default:
                //
                // We should never come here. Let's throw an exception to catch this unlikely scenario.
                //
                throw new IllegalStateException(String.format("Unexpected request ID %s.", requestId));
        }
    }   //processData

    /**
     * This method combines the two byte into a 16-bit word according to whether the MSB is first.
     *
     * @param firstByte specifies the first byte.
     * @param secondByte specifies the second byte.
     * @param msbFirst specifies true if first byte is the MSB.
     * @return combined 16-bit word.
     */
    private int getWord(byte firstByte, byte secondByte, boolean msbFirst)
    {
        return msbFirst? TrcUtil.bytesToInt(secondByte, firstByte): TrcUtil.bytesToInt(firstByte, secondByte);
    }   //getWord

    /**
     * This method is called when the read request is completed.
     *
     * @param context specifies the read request.
     */
    public void requestHandler(Object context)
    {
        final String funcName = "requestHandler";
        TrcSerialBusDevice.Request request = (TrcSerialBusDevice.Request) context;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.CALLBK, "request=%s", request);
        }

        if (request.readRequest && request.address == -1 && request.buffer != null)
        {
            if (!request.canceled)
            {
                processData((RequestId)request.requestId, request.buffer, request.buffer.length);
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.CALLBK);
        }
    }   //requestHanlder

}   //class TrcPixyCam1
