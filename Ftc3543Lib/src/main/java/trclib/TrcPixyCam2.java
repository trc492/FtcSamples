/*
 * Copyright (c) 2019 Titan Robotics Club (http://www.titanrobotics.com)
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
 * This class implements a platform independent pixy camera 2. This class is intended to be extended by a platform
 * dependent pixy class which provides the abstract methods required by this class. This class provides the parser
 * to read and parse the response packets from the pixy camera 2. It also implements pixy camera APIs for all the
 * pixy requests.
 */
public abstract class TrcPixyCam2
{
    protected static final String moduleName = "TrcPixyCam2";
    protected static final boolean debugEnabled = false;
    protected static final boolean tracingEnabled = false;
    protected static final boolean useGlobalTracer = false;
    protected static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    protected static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    protected TrcDbgTrace dbgTrace = null;

    private static final short PIXY2_SEND_SYNC                  = (short)0xc1ae;
    private static final short PIXY2_RECV_SYNC                  = (short)0xc1af;

    private static final byte PIXY2_REQ_GET_RESOLUTION          = (byte)12;
    private static final byte PIXY2_REQ_GET_VERSION             = (byte)14;
    private static final byte PIXY2_REQ_SET_CAMERA_BRIGHTNESS   = (byte)16;
    private static final byte PIXY2_REQ_SET_SERVOS              = (byte)18;
    private static final byte PIXY2_REQ_SET_LED                 = (byte)20;
    private static final byte PIXY2_REQ_SET_LAMP                = (byte)22;
    private static final byte PIXY2_REQ_GET_FPS                 = (byte)24;

    private static final byte PIXY2_REQ_GET_BLOCKS              = (byte)32;

    private static final byte PIXY2_REQ_GET_MAIN_FEATURES       = (byte)48;
    private static final byte PIXY2_REQ_SET_MODE                = (byte)54;
    private static final byte PIXY2_REQ_SET_VECTOR              = (byte)56;
    private static final byte PIXY2_REQ_SET_NEXT_TURN           = (byte)58;
    private static final byte PIXY2_REQ_SET_DEFAULT_TURN        = (byte)60;
    private static final byte PIXY2_REQ_REVERSE_VECTOR          = (byte)62;

    private static final byte PIXY2_REQ_GET_RGB                 = (byte)112;

    private static final byte PIXY2_RES_RESULT                  = (byte)1;
    private static final byte PIXY2_RES_RESOLUTION              = (byte)13;
    private static final byte PIXY2_RES_VERSION                 = (byte)15;

    private static final byte PIXY2_RES_BLOCKS                  = (byte)33;

    private static final byte PIXY2_RES_MAIN_FEATURES           = (byte)49;

    public static final byte PIXY2_FEATURE_TYPE_MAIN            = (byte)0x00;
    public static final byte PIXY2_FEATURE_TYPE_ALL             = (byte)0x01;

    public static final byte PIXY2_FEATURES_VECTOR              = (byte)(1 << 0);
    public static final byte PIXY2_FEATURES_INTERSECTION        = (byte)(1 << 1);
    public static final byte PIXY2_FEATURES_BARCODE             = (byte)(1 << 2);
    public static final byte PIXY2_FEATURES_ALL                 = (byte)(PIXY2_FEATURES_VECTOR +
                                                                         PIXY2_FEATURES_INTERSECTION +
                                                                         PIXY2_FEATURES_BARCODE);

    public static final byte PIXY2_BLOCKS_SIG_1                 = (byte)(1 << 0);
    public static final byte PIXY2_BLOCKS_SIG_2                 = (byte)(1 << 1);
    public static final byte PIXY2_BLOCKS_SIG_3                 = (byte)(1 << 2);
    public static final byte PIXY2_BLOCKS_SIG_4                 = (byte)(1 << 3);
    public static final byte PIXY2_BLOCKS_SIG_5                 = (byte)(1 << 4);
    public static final byte PIXY2_BLOCKS_SIG_6                 = (byte)(1 << 5);
    public static final byte PIXY2_BLOCKS_SIG_7                 = (byte)(1 << 6);
    public static final byte PIXY2_BLOCKS_SIG_8                 = (byte)(1 << 7);
    public static final byte PIXY2_BLOCKS_ALL_SIG               = (byte)255;
    public static final byte PIXY2_MAX_BLOCKS_ALL               = (byte)255;

    public static final byte PIXY2_LINE_FLAG_INVALID            = (byte)0x02;
    public static final byte PIXY2_LINE_FLAG_INTERSECTION_PRESENT=(byte)0x04;

    public static final byte PIXY2_SAT_FLAG_SATURATE            = (byte)0x01;

    /**
     * This method writes the request data to the device synchronously.
     *
     * @param data specifies the request data.
     */
    public abstract void syncWriteRequest(byte[] data);

    /**
     * This method issues a synchronous read of the request response.
     */
    public abstract byte[] syncReadResponse();

    /**
     * This class implements the detected object block.
     */
    public class Block
    {
        public int signature;
        public int centerX;
        public int centerY;
        public int width;
        public int height;
        public int angle;
        public byte trackingIndex;
        public byte age;

        public Block(byte[] data, int startIndex)
        {
            signature = TrcUtil.bytesToInt(data[startIndex], data[startIndex + 1]);
            centerX = TrcUtil.bytesToInt(data[startIndex + 2], data[startIndex + 3]);
            centerY = TrcUtil.bytesToInt(data[startIndex + 4], data[startIndex + 5]);
            width = TrcUtil.bytesToInt(data[startIndex + 6], data[startIndex + 7]);
            height = TrcUtil.bytesToInt(data[startIndex + 8], data[startIndex + 9]);
            trackingIndex = data[startIndex + 10];
            age = data[startIndex + 11];
        }   //Block

        /**
         * This method formats all fields into a string for printing purpose.
         *
         * @return string containing all the fields.
         */
        @Override
        public String toString()
        {
            return String.format(
                Locale.US, "sig=%d, centerX=%3d, centerY=%3d, width=%3d, height=%3d, angle=%3d, index=%d, age=%d",
                signature, centerX, centerY, width, height, angle, trackingIndex, age);
        }   //toString

    }   //class Block

    public class Vector
    {
        public byte x0;
        public byte y0;
        public byte x1;
        public byte y1;
        public byte index;
        public byte flags;

        public Vector(byte[] data, int startIndex)
        {
            x0 = data[startIndex];
            y0 = data[startIndex + 1];
            x1 = data[startIndex + 2];
            y1 = data[startIndex + 3];
            index = data[startIndex + 4];
            flags = data[startIndex + 5];
        }   //Vector

        @Override
        public String toString()
        {
            return String.format(Locale.US, "x0=%d, y0=%d, x1=%d, y1=%d, index=%d, flags=0x%x",
                x0, y0, x1, y1, index, flags);
        }   //toString

    }   //class Vector

    public class IntersectionLine
    {
        public byte index;
        public byte reserved;
        public short angle;

        public IntersectionLine(byte[] data, int startIndex)
        {
            index = data[startIndex];
            reserved = data[startIndex + 1];
            angle = TrcUtil.bytesToShort(data[startIndex + 2], data[startIndex + 3]);
        }   //IntersectionLine

        @Override
        public String toString()
        {
            return String.format(Locale.US, "index=%d, reserved=%d, angle=%d", index, reserved, angle);
        }   //toString

    }   //class IntersectionLine

    public class Intersection
    {
        public byte x;
        public byte y;
        public byte n;
        public byte reserved;
        public IntersectionLine[] intersectionLines;

        public Intersection(byte[] data, int startIndex)
        {
            x = data[startIndex];
            y = data[startIndex + 1];
            n = data[startIndex + 2];
            reserved = data[startIndex + 3];
            for (int i = 0; i < n; i++)
            {
                intersectionLines[i] = new IntersectionLine(data, startIndex + 4 + i*4);
            }
        }   //Intersection

        @Override
        public String toString()
        {
            String s = String.format(Locale.US, "x=%d, y=%d, n=%d, reserved=%d, lines:", x, y, n, reserved);
            for (int i = 0; i < n; i++)
            {
                s += "\n\t" + intersectionLines[i].toString();
            }
            return s;
        }   //toString

    }   //class Intersection

    public class Barcode
    {
        public byte x;
        public byte y;
        public byte flags;
        public byte code;

        public Barcode(byte[] data, int startIndex)
        {
            x = data[startIndex];
            y = data[startIndex + 1];
            flags = data[startIndex + 2];
            code = data[startIndex + 3];
        }   //Barcode

        @Override
        public String toString()
        {
            return String.format(Locale.US, "x=%d, y=%d, flags=0x%x, code=0x%x", x, y, flags, code);
        }   //toString

    }   //class Barcode

    public class Feature
    {
        public byte type;

        public Feature(byte type)
        {
            this.type = type;
        }   //Feature

    }   //class Feature

    public class FeatureVectors extends Feature
    {
        public Vector[] vectors;

        public FeatureVectors(byte[] data, int startIndex)
        {
            super(data[startIndex]);
            final String funcName = "FeatureVectors";
            vectors = new Vector[data[startIndex + 1]/6];

            if (debugEnabled)
            {
                dbgTrace.traceInfo(funcName, "data=%s, startIndex=%d", Arrays.toString(data), startIndex);
            }

            for (int i = 0, index = startIndex + 2; i < vectors.length; i++, index += 6)
            {
                vectors[i] = new Vector(data, index);
                if (debugEnabled)
                {
                    dbgTrace.traceInfo(funcName, "[%d]: %s", i, vectors[i]);
                }
            }
        }   //FeatureVectors

        @Override
        public String toString()
        {
            String feature = "Vectors: ";

            for (int i = 0; i < vectors.length; i++)
            {
                feature += vectors[i].toString() + "; ";
            }

            return feature;
        }   //toString

    }   //class FeatureVectors

    public class FeatureIntersections extends Feature
    {
        public Intersection[] intersections;

        public FeatureIntersections(byte[] data, int startIndex)
        {
            super(data[startIndex]);
            ArrayList<Intersection> list = new ArrayList<>();
            int endIndex = startIndex + 2 + data[startIndex + 1];

            for (int index = startIndex + 2; index < endIndex;)
            {
                Intersection intersection = new Intersection(data, index);
                list.add(intersection);
                index += 4 + intersection.n*4;
            }

            intersections = list.toArray(intersections);
        }   //FeatureIntersections

        @Override
        public String toString()
        {
            String feature = "Intersections: ";

            for (int i = 0; i < intersections.length; i++)
            {
                feature += intersections[i].toString() + "; ";
            }

            return feature;
        }   //toString

    }   //class FeatureIntersections

    public class FeatureBarcodes extends Feature
    {
        public Barcode[] barcodes;

        public FeatureBarcodes(byte[] data, int startIndex)
        {
            super(data[startIndex]);
            barcodes = new Barcode[data[startIndex + 1]/4];
            for (int i = 0, index = startIndex + 2; i < barcodes.length; i++, index += 4)
            {
                barcodes[i] = new Barcode(data, index);
            }
        }   //FeatureBarcodes

        @Override
        public String toString()
        {
            String feature = "Barcodes: ";

            for (int i = 0; i < barcodes.length; i++)
            {
                feature += barcodes[i].toString() + "; ";
            }

            return feature;
        }   //toString

    }   //class FeatureBarcodes

    private final String instanceName;
    private int hardwareVersion = 0;
    private int firmwareVersion = 0;
    private byte firmwareType = 0;
    private int resolutionWidth = 0;
    private int resolutionHeight = 0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public TrcPixyCam2(final String instanceName)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
    }   //TrcPixyCam2

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
     * This method checks if the checksum of the response packet is correct.
     *
     * @param data specifies the response packet.
     * @return true if the checksum is correct, false otherwise.
     */
    private boolean validateChecksum(byte[] data)
    {
        boolean validChecksum = true;

        if (data != null)
        {
            int checksum = 0;

            for (int i = 6; i < data.length; i++)
            {
                checksum += ((int) data[i]) & 0xff ;
            }
            validChecksum = TrcUtil.bytesToInt(data[4], data[5]) == checksum;
        }

        return validChecksum;
    }   //validateChecksum

    /**
     * This method builds the request packet, writes it to the device, reads the response packet,
     * check the checksum and returns the response packet if it's valid.
     *
     * @param requestType specifies the request type.
     * @param data specifies the request data if any, null if none.
     * @param expectedResponseType specifies the expected response type.
     * @return response packet if it's valid, null if invalid.
     */
    private byte[] sendRequest(byte requestType, byte[] data, byte expectedResponseType)
    {
        final String funcName = "SendRequest";
        int dataLen = data == null ? 0 : data.length;
        byte[] request = new byte[4 + dataLen];

        request[0] = (byte)(PIXY2_SEND_SYNC & 0xff);
        request[1] = (byte)((PIXY2_SEND_SYNC >> 8) & 0xff);
        request[2] = requestType;
        request[3] = (byte)dataLen;

        System.arraycopy(data, 0, request, 4, dataLen);
        syncWriteRequest(request);
        byte[] response = syncReadResponse();

        if (debugEnabled)
        {
            dbgTrace.traceInfo(funcName, "Request%s => Response%s",
                Arrays.toString(request), Arrays.toString(response));
        }

        if ((short) TrcUtil.bytesToInt(response[0], response[1]) == PIXY2_RECV_SYNC &&
            response[2] == expectedResponseType && validateChecksum(response))
        {
            return response;
        }
        else
        {
            return null;
        }
    }   //sendRequest

    /**
     * This method builds a request that will reply with a standard PIXY2_RES_RESULT response packet.
     * It will then returned the result code from the response packet.
     *
     * @param requestType specifies the request type.
     * @param data specifies the request data if any, null if none.
     * @return result code from the response packet.
     */
    private int sendDataRequest(byte requestType, byte[] data)
    {
        int result = -1;
        byte[] response = sendRequest(requestType, data, PIXY2_RES_RESULT);

        if (response != null)
        {
            result = TrcUtil.bytesToInt(response[6], response[7], response[8], response[9]);
        }

        return result;
    }   //sendDataRequest

    /**
     * This method sends a GET_VERSION request to the device if it hasn't already. It updates the device
     * version info.
     */
    private void getVersion()
    {
        if (hardwareVersion == 0)
        {
            byte[] response = sendRequest(PIXY2_REQ_GET_VERSION, null, PIXY2_RES_VERSION);

            if (response != null)
            {
                hardwareVersion = TrcUtil.bytesToInt(response[6], response[7]);
                firmwareVersion = TrcUtil.bytesToInt(response[10], response[11], response[9], response[8]);
                firmwareType = response[12];
            }
        }
    }   //getVersion

    /**
     * This method returns the hardware version info.
     *
     * @return hardware version info.
     */
    public int getHardwareVersion()
    {
        getVersion();
        return hardwareVersion;
    }   //getHardwareVersion

    /**
     * This method returns the firmware version info.
     *
     * @return firmware version info.
     */
    public int getFirmwareVersion()
    {
        getVersion();
        return firmwareVersion;
    }   //getFirmwareVersion

    /**
     * This method returns the firmware type info.
     *
     * @return firmware type info.
     */
    public byte getFirmwareType()
    {
        getVersion();
        return firmwareType;
    }   //getFirmwareType

    /**
     * This method sends a GET_RESOLUTION request if it hasn't already. It updates the device
     * resolution info.
     */
    private void getResolution()
    {
        if (resolutionWidth == 0)
        {
            byte[] data = {0};
            byte[] response = sendRequest(PIXY2_REQ_GET_RESOLUTION, data, PIXY2_RES_RESOLUTION);

            if (response != null)
            {
                resolutionWidth = TrcUtil.bytesToInt(response[6], response[7]);
                resolutionHeight = TrcUtil.bytesToInt(response[8], response[9]);
            }
        }
    }   //getResolution

    /**
     * This method returns the resolution width of the device.
     *
     * @return resolution width.
     */
    public int getResolutionWidth()
    {
        getResolution();
        return resolutionWidth;
    }   //getResolutionWidth

    /**
     * This method returns the resolution height of the device.
     *
     * @return resolution height.
     */
    public int getResolutionHeight()
    {
        getResolution();
        return resolutionHeight;
    }   //getResolutionHeight

    /**
     * This method sends a request to set the camera brightness.
     *
     * @param brightness specifies the brightness value (0-255).
     * @return the result code.
     */
    public int setCameraBrightness(byte brightness)
    {
        byte[] data = {brightness};

        return sendDataRequest(PIXY2_REQ_SET_CAMERA_BRIGHTNESS, data);
    }   //setCameraBrightness

    /**
     * This method sends a request to set the pan and tilt value of the gimbal servos.
     *
     * @param panValue specifies the pan servo value (0-511)
     * @param tiltValue specifies the tilt servo value (0-511).
     * @return the result code.
     */
    public int setServos(int panValue, int tiltValue)
    {
        int result = -1;

        if (panValue >= 0 && panValue <= 511 && tiltValue >= 0 && tiltValue <= 511)
        {
            byte[] data = {TrcUtil.intToByte(panValue, 0), TrcUtil.intToByte(panValue, 1),
                           TrcUtil.intToByte(tiltValue, 0), TrcUtil.intToByte(tiltValue, 1)};

            result = sendDataRequest(PIXY2_REQ_SET_SERVOS, data);
        }

        return result;
    }   //setServos

    /**
     * This method sends a request to set the LED color.
     *
     * @param red specifies the red value.
     * @param green specifies the green value.
     * @param blue specifies the blue value.
     *
     * @return the result code.
     */
    public int setLED(byte red, byte green, byte blue)
    {
        byte[] data = {red, green, blue};

        return sendDataRequest(PIXY2_REQ_SET_LED, data);
    }   //setLED

    /**
     * This method sends a request to turn on/off the lamp LEDs.
     *
     * @param upper specifies true to turn on the two white LEDs on the top edge, false to turn off.
     * @param lower specifies true to turn on the lower RGB LED, false to turn off.
     * @return the result code.
     */
    public int setLamp(boolean upper, boolean lower)
    {
        byte[] data = {(byte) (upper ? 1 : 0), (byte) (lower ? 1 : 0)};

        return sendDataRequest(PIXY2_REQ_SET_LAMP, data);
    }   //setLamp

    /**
     * This method sends a request to get the camera frame rate.
     *
     * @return the camera frame rate in frames per second.
     */
    public int getFPS()
    {
        return sendDataRequest(PIXY2_REQ_GET_FPS, null);
    }   //getFPS

    /**
     * This method sends a request to get the detected object blocks.
     *
     * @param sigMap specifies the bitmap of the signatures (i.e. bit 1 set for signature 1, bit 2 set for signature 2 etc).
     * @param maxBlocks specifies the maximum number of blocks to retrieve.
     * @return an array of detected blocks.
     */
    public Block[] getBlocks(byte sigMap, byte maxBlocks)
    {
        Block[] blocks = null;
        byte[] data = {sigMap, maxBlocks};
        byte[] response = sendRequest(PIXY2_REQ_GET_BLOCKS, data, PIXY2_RES_BLOCKS);

        if (response != null)
        {
            int numBlocks = (response.length - 6)/14;

            blocks = new Block[numBlocks];
            for (int i = 0; i < numBlocks; i++)
            {
                blocks[i] = new Block(response, 6 + i*14);
            }
        }

        return blocks;
    }   //getBlocks

    /**
     * This method sends a request to get the detected features.
     *
     * @param requestType specifies 0 for the main features and 1 for all features.
     * @param featuresMap specifies the features bitmap (i.e. 1 for vectors, 2 for intersections and 4 for barcodes).
     * @return an array of detected features.
     */
    public Feature[] getFeatures(byte requestType, byte featuresMap)
    {
        Feature[] features = null;
        byte[] data = {requestType, featuresMap};
        byte[] response = sendRequest(PIXY2_REQ_GET_MAIN_FEATURES, data, PIXY2_RES_MAIN_FEATURES);

        if (response != null)
        {
            ArrayList<Feature> list = new ArrayList<>();

            for (int i = 6; i < response.length;)
            {
                if (i + 1 < response.length && i + 2 + response[i + 1] <= response.length)
                {
                    Feature feature = null;

                    switch (response[i])
                    {
                        case PIXY2_FEATURES_VECTOR:
                            feature = new FeatureVectors(response, i);
                            break;

                        case PIXY2_FEATURES_INTERSECTION:
                            feature = new FeatureIntersections(response, i);
                            break;

                        case PIXY2_FEATURES_BARCODE:
                            feature = new FeatureBarcodes(response, i);
                            break;
                    }

                    if (feature != null)
                    {
                        list.add(feature);
                    }
                }
                i += 2 + response[i + 1];
            }

            if (list.size() > 0)
            {
                features = new Feature[list.size()];
                features = list.toArray(features);
            }
        }

        return features;
    }   //getFeatures

    public Feature[] getMainFeatures(byte featuresMap)
    {
        return getFeatures(PIXY2_FEATURE_TYPE_MAIN, featuresMap);
    }   //getMainFeatures

    public Feature[] getAllFeatures(byte featuresMap)
    {
        return getFeatures(PIXY2_FEATURE_TYPE_ALL, featuresMap);
    }   // getAllFeatures

    /**
     * This method sends a request to set the camera mode.
     *
     * @param mode specifies the camera mode.
     * @return the result code.
     */
    public int setMode(byte mode)
    {
        byte[] data = {mode};

        return sendDataRequest(PIXY2_REQ_SET_MODE, data);
    }   //setMode

    /**
     * This method sends a request to set the next turn angle.
     *
     * @param angle specifies the turn angle.
     * @return the result code.
     */
    public int setNextTurn(short angle)
    {
        byte[] data = {TrcUtil.intToByte(angle, 0), TrcUtil.intToByte(angle, 1)};

        return sendDataRequest(PIXY2_REQ_SET_NEXT_TURN, data);
    }   //setNextTurn

    /**
     * This method sends a request to set the default turn angle.
     *
     * @param angle specifies the default turn angle.
     * @return the result code.
     */
    public int setDefaultTurn(short angle)
    {
        byte[] data = {TrcUtil.intToByte(angle, 0), TrcUtil.intToByte(angle, 1)};

        return sendDataRequest(PIXY2_REQ_SET_DEFAULT_TURN, data);
    }   //setDefaultTurn

    public int setVector(byte index)
    {
        byte[] data = {index};

        return sendDataRequest(PIXY2_REQ_SET_VECTOR, data);
    }   //setVector

    public int reverseVector()
    {
        return sendDataRequest(PIXY2_REQ_REVERSE_VECTOR, null);
    }   //reverseVector

    public int getRGB(int x, int y, byte satFlag)
    {
        byte[] data = {TrcUtil.intToByte(x, 0), TrcUtil.intToByte(x, 1),
                       TrcUtil.intToByte(y, 0), TrcUtil.intToByte(y, 1), satFlag};

        return sendDataRequest(PIXY2_REQ_GET_RGB, data);
    }   //getRGB

}   //class TrcPixyCam2
