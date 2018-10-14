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

/**
 * This class implements a platform independent pixy camera. This class is intended to be extended by a platform
 * dependent pixy class which provides the abstract methods required by this class. This class provides the parser
 * to read and parse the object block from the pixy camera. It also provides access to the last detected objects
 * reported by the pixy camera asynchronously.
 */
public abstract class TrcPixyCam
{
    protected static final String moduleName = "TrcPixyCam";
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
     * This method issues an synchronous read of the specified number of bytes from the device.
     *
     * @param length specifies the number of bytes to read.
     * @return data read.
     */
    public abstract byte[] readData(int length);

    /**
     * This method writes the data buffer to the device.
     *
     * @param data specifies the data buffer.
     * @param waitForCompletion specifies true to wait for write completion, false otherwise.
     */
    public abstract void writeData(byte[] data, boolean waitForCompletion);

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

        public String toString()
        {
            return String.format(
                "sync=0x%04x, chksum=0x%04x, sig=%d, centerX=%3d, centerY=%3d, width=%3d, height=%3d, angle=%3d",
                sync, checksum, signature, centerX, centerY, width, height, angle);
        }
    }   //class ObjectBlock

    /**
     * This is used identify the request type.
     */
    public static enum ReaderState
    {
        SYNC,
        ALIGN,
        CHECKSUM,
        NORMAL_BLOCK,
        COLOR_CODE_BLOCK,
        PROCESS_BLOCK
    }   //enum ReaderState

    private final String instanceName;
    private final boolean msbFirst;
    private TrcTaskMgr.TaskObject readerTaskObj;
    private TrcStateMachine<ReaderState> sm;
    private boolean enabled = false;
    private ArrayList<ObjectBlock> objects = new ArrayList<>();
    private ObjectBlock[] detectedObjects = null;
    private ObjectBlock currBlock = null;
    private Object objectLock = new Object();
    private byte[] data = null;
    private int runningChecksum = 0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param msbFirst specifies true if a word has MSB first.
     */
    public TrcPixyCam(final String instanceName, boolean msbFirst)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
        this.msbFirst = msbFirst;
        readerTaskObj = TrcTaskMgr.getInstance().createTask(instanceName + ".readerTask", this::readerTask);
        sm = new TrcStateMachine<>(instanceName);
    }   //TrcPixyCam

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
     * This method enables/disables the reader task.
     *
     * @param enabled specifies true to enable, false to disable.
     */
    public void setEnabled(boolean enabled)
    {
        final String funcName = "setEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC, "enabled=%b", enabled);
        }

        if (enabled)
        {
            sm.start(ReaderState.SYNC);
            readerTaskObj.registerTask(TrcTaskMgr.TaskType.STANDALONE_TASK);
        }
        else
        {
            readerTaskObj.unregisterTask(TrcTaskMgr.TaskType.STANDALONE_TASK);
            sm.stop();
        }
        this.enabled = enabled;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC);
        }
    }   //setEnabled

    /**
     * This method checks if the reader task is enabled.
     *
     * @return true if enabled, false otherwise.
     */
    public boolean isEnabled()
    {
        final String funcName = "isEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", enabled);
        }

        return enabled;
    }   //isEnabled

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

        writeData(data, false);

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

        writeData(data, false);

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

        writeData(data, false);

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
        ObjectBlock[] objectBlocks = null;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        synchronized (objectLock)
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

    public void readerTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = "readerTask";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "taskType=%s,runMode=%s",
                    taskType, runMode);
        }

        ReaderState state = sm.checkReadyAndGetState();
        if (debugEnabled)
        {
            dbgTrace.traceInfo(funcName, "State: %s", state != null? state: "Disabled");
        }

        if (state != null)
        {
            int word;
            int index;

            switch (state)
            {
                case SYNC:
                    //
                    // If we don't already have an object block allocated, allocate it now.
                    //
                    if (currBlock == null)
                    {
                        currBlock = new ObjectBlock();
                    }

                    data = readData(2);
                    if (data == null || data.length != 2)
                    {
                        //
                        // Read failed, remain in this state.
                        //
                        if (debugEnabled)
                        {
                            dbgTrace.traceWarn(funcName, "%s: Read failed (data=%s)",
                                    state, data == null? null: Arrays.toString(data));
                        }
                    }
                    else
                    {
                        word = getWord(data[0], data[1], msbFirst);
                        if (word == PIXY_START_WORD || word == PIXY_START_WORD_CC)
                        {
                            //
                            // Found a sync word, go to next state.
                            //
                            currBlock.sync = word;
                            sm.setState(ReaderState.CHECKSUM);
                        }
                        else if (word == PIXY_START_WORDX)
                        {
                            //
                            // We are word misaligned. Realign it by reading one byte and expecting it to be the high
                            // sync byte.
                            //
                            currBlock.sync = PIXY_START_WORD;
                            sm.setState(ReaderState.ALIGN);
                            if (debugEnabled)
                            {
                                dbgTrace.traceInfo(funcName, "%s: Word misaligned, realigning...", state);
                            }
                        }
                        else
                        {
                            //
                            // We don't find the sync word, throw it away and remain in this state.
                            //
                            if (debugEnabled)
                            {
                                if (word != 0)
                                {
                                    dbgTrace.traceWarn(funcName, "%s: Unexpected word 0x%04x", state, word);
                                }
                            }
                        }
                    }
                    break;

                case ALIGN:
                    data = readData(1);
                    if (data == null || data.length != 1)
                    {
                        //
                        // We should never come here. Let's throw an exception to catch this unlikely scenario.
                        //
                        throw new IllegalStateException(String.format("%s: Read failed (data=%s)",
                                state, data == null? null: Arrays.toString(data)));
                    }
                    else if (data[0] == PIXY_SYNC_HIGH)
                    {
                        //
                        // Found the expected upper sync byte, go to next state.
                        //
                        sm.setState(ReaderState.CHECKSUM);
                    }
                    else
                    {
                        //
                        // Don't see the expected upper sync byte, let's go back to the SYNC state assuming we are
                        // now word aligned again.
                        //
                        sm.setState(ReaderState.SYNC);
                        if (debugEnabled)
                        {
                            dbgTrace.traceWarn(funcName, "%s: Unexpected data byte 0x%02x.", state, data[0]);
                        }
                    }
                    break;

                case CHECKSUM:
                    data = readData(2);
                    if (data == null || data.length != 2)
                    {
                        //
                        // We should never come here. Let's throw an exception to catch this unlikely scenario.
                        //
                        throw new IllegalStateException(String.format("%s: Read failed (data=%s)",
                                state, data == null? null: Arrays.toString(data)));
                    }
                    else
                    {
                        word = getWord(data[0], data[1], msbFirst);
                        if (word == PIXY_START_WORD || word == PIXY_START_WORD_CC)
                        {
                            //
                            // We were expecting a checksum but found a sync word. It means that's the end-of-frame.
                            // Save away the sync word for the next frame and expecting next state to be CHECKSUM.
                            //
                            currBlock.sync = word;
                            sm.setState(ReaderState.CHECKSUM);
                            //
                            // Detected end-of-frame, convert the array list of objects into detected object array.
                            //
                            if (objects.size() > 0)
                            {
                                synchronized (objectLock)
                                {
                                    ObjectBlock[] array = new ObjectBlock[objects.size()];
                                    detectedObjects = objects.toArray(array);
                                    objects.clear();
                                    if (debugEnabled)
                                    {
                                        for (int i = 0; i < detectedObjects.length; i++)
                                        {
                                            dbgTrace.traceInfo(funcName, "[%02d] %s",
                                                    i, detectedObjects[i].toString());
                                        }
                                    }
                                }
                            }
                        }
                        else
                        {
                            //
                            // Looks like we have a checksum, save it away and go to the next state to read the rest
                            // of the block. If the sync word was PIXY_START_WORD, then it is a 10-byte NORMAL_BLOCK,
                            // else it is a 12-byte COLOR_CODE_BLOCK.
                            //
                            currBlock.checksum = word;
                            if (currBlock.sync == PIXY_START_WORD)
                            {
                                sm.setState(ReaderState.NORMAL_BLOCK);
                            }
                            else if (currBlock.sync == PIXY_START_WORD_CC)
                            {
                                sm.setState(ReaderState.COLOR_CODE_BLOCK);
                            }
                            else
                            {
                                //
                                // We should never come here. Let's throw an exception to catch this unlikely scenario.
                                //
                                throw new IllegalStateException(String.format("%s: Unexpected sync word 0x%04x.",
                                        state, currBlock.sync));
                            }
                        }
                    }
                    break;

                case NORMAL_BLOCK:
                    data = readData(10);
                    if (data == null || data.length != 10)
                    {
                        //
                        // We should never come here. Let's throw an exception to catch this unlikely scenario.
                        //
                        throw new IllegalStateException(String.format("%s: Read failed (data=%s)",
                                state, data == null? null: Arrays.toString(data)));
                    }
                    sm.setState(ReaderState.PROCESS_BLOCK);
                    break;

                case COLOR_CODE_BLOCK:
                    data = readData(12);
                    if (data == null || data.length != 12)
                    {
                        //
                        // We should never come here. Let's throw an exception to catch this unlikely scenario.
                        //
                        throw new IllegalStateException(String.format("%s: Read failed (data=%s)",
                                state, data == null? null: Arrays.toString(data)));
                    }
                    sm.setState(ReaderState.PROCESS_BLOCK);
                    break;

                case PROCESS_BLOCK:
                    runningChecksum = 0;
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
                    if (currBlock.sync == PIXY_START_WORD_CC)
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
                        objects.add(currBlock);
                        currBlock = null;
                    }
                    else if (debugEnabled)
                    {
                        dbgTrace.traceWarn(funcName, "%s: Incorrect checksum %d (expecting %d).",
                                state, runningChecksum, currBlock.checksum);
                    }
                    //
                    // Go back to the SYNC state for the next block.
                    //
                    sm.setState(ReaderState.SYNC);
                    break;

                default:
                    //
                    // We should never come here. Let's throw an exception to catch this unlikely scenario.
                    //
                    throw new IllegalStateException(String.format("Unexpected state %s.", state));
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //runPeriodic

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

}   //class TrcPixyCam
