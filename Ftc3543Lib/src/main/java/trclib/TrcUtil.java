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

package trclib;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Date;
import java.util.Locale;

/**
 * This class contains platform independent utility methods. All methods in this class are static. It is not
 * necessary to instantiate this class to call its methods.
 */
public class TrcUtil
{
    public static final double INCHES_PER_CM = 0.393701;
    public static final double MM_PER_INCH = 25.4;
    public static final double EARTH_GRAVITATIONAL_CONSTANT = 9.807;    //in m/s2

    /**
     * This interface provides the method to get data of the specified type. This is to replaced the Supplier
     * interface that Java SDK provides but Android API level 19 does not have.
     */
    public interface DataSupplier<T>
    {
        /**
         * This method returns the data of the designated type.
         *
         * @return data of the designated type.
         */
        T get();

    }   //interface DataSupplier

    /**
     * This method returns the current time in seconds with nano-second precision.
     *
     * @return current time in seconds.
     */
    public static double getCurrentTime()
    {
        return System.nanoTime() / 1000000000.0;
    }   //getCurrentTime

    /**
     * This method returns the current time in msec.
     *
     * @return current time in msec.
     */
    public static long getCurrentTimeMillis()
    {
        return System.currentTimeMillis();
    }   //getCurrentTimeMillis

    /**
     * This method returns the current time in nano second.
     *
     * @return current time in nano second.
     */
    public static long getCurrentTimeNanos()
    {
        return System.nanoTime();
    }   //getCurrentTimeNanos

    /**
     * This method returns the current time stamp with the specified format.
     *
     * @param format specifies the time stamp format.
     * @return current time stamp string with the specified format.
     */
    public static String getTimestamp(String format)
    {
        SimpleDateFormat dateFormat = new SimpleDateFormat(format, Locale.US);
        return dateFormat.format(new Date());
    }   //getTimestamp

    /**
     * This method returns the current time stamp with the default format.
     *
     * @return current time stamp string with the default format.
     */
    public static String getTimestamp()
    {
        return getTimestamp("yyyyMMdd@HHmmss");
    }   //getTimestamp

    /**
     * This method puts the current thread to sleep for the given time in msec. It handles InterruptException where
     * it recalculates the remaining time and calls sleep again repeatedly until the specified sleep time has past.
     *
     * @param milliTime specifies sleep time in msec.
     */
    public static void sleep(long milliTime)
    {
        long wakeupTime = System.currentTimeMillis() + milliTime;

        while (milliTime > 0)
        {
            try
            {
                Thread.sleep(milliTime);
                break;
            }
            catch (InterruptedException e)
            {
                milliTime = wakeupTime - System.currentTimeMillis();
            }
        }
    }   //sleep

    /**
     * This method calculates the modulo of two numbers. Unlike the <code>%</code> operator, this returns a number
     * in the range [0, b). For some reason, in Java, the <code>%</code> operator actually does remainder, which
     * means the result is in the range (-b, b).
     *
     * @param a specifies the dividend.
     * @param b specifies the divisor.
     * @return the modulo in the range [0, b)
     */
    public static double modulo(double a, double b)
    {
        return ((a % b) + b) % b;
    }   //modulo

    /**
     * This method sums an array of numbers.
     *
     * @param nums specifies the array of numbers to be summed.
     * @return sum of the numbers.
     */
    public static double sum(double... nums)
    {
        double total = 0.0;

        for (double num : nums)
        {
            total += num;
        }

        return total;
    }   //sum

    /**
     * This method calculates and returns the median of the numbers in the given array.
     *
     * @param num specifies the number array.
     * @return median of numbers in the array.
     */
    public static double median(double... num)
    {
        double m = 0.0;

        if (num.length > 0)
        {
            double[] nums = num.clone();

            Arrays.sort(nums);
            if (nums.length % 2 == 0)
            {
                m = TrcUtil.average(nums[(nums.length / 2) - 1], nums[nums.length / 2]);
            }
            else
            {
                m = nums[nums.length / 2];
            }
        }

        return m;
    }   //median

    /**
     * This method calculates and returns the average of the numbers in the given array.
     *
     * @param nums specifies the number array.
     * @return average of all numbers in the array. If the array is empty, return 0.
     */
    public static double average(double... nums)
    {
        return nums.length == 0 ? 0.0 : sum(nums) / nums.length;
    }   //average

    /**
     * This method calculates the magnitudes of the given array of numbers.
     *
     * @param nums specifies the number array.
     * @return magnitude of all numbers in the array.
     */
    public static double magnitude(double... nums)
    {
        double total = 0.0;

        for (double num : nums)
        {
            total += num * num;
        }

        return Math.sqrt(total);
        // return Math.sqrt(Arrays.stream(nums).map(e -> e*e).sum());
    }   //magnitude

    /**
     * This method returns the maximum magnitude of numbers in the specified array.
     *
     * @param nums specifies the number array.
     * @return maximum magnitude of the numbers in the array.
     */
    public static double maxMagnitude(double... nums)
    {
        double maxMag = Math.abs(nums[0]);

        for (double num : nums)
        {
            double magnitude = Math.abs(num);
            if (magnitude > maxMag)
            {
                maxMag = magnitude;
            }
        }

        return maxMag;
    }   //maxMagnitude

    /**
     * This method returns a bit mask of the least significant set bit.
     *
     * @param data specifies the data to find the least significant set bit.
     * @return bit mask that has only the least significant set bit.
     */
    public static int leastSignificantSetBit(int data)
    {
        int bitMask = 0;

        if (data != 0)
        {
            bitMask = data & ~(data ^ -data);
        }

        return bitMask;
    }   //leastSignificantSetBit

    /**
     * This method returns the bit position of the least significant set bit of the given data.
     *
     * @param data specifies the data to determine its least significant set bit position.
     * @return 0-based least significant set bit position. -1 if no set bit.
     */
    public static int leastSignificantSetBitPosition(int data)
    {
        int pos = -1;

        if (data != 0)
        {
            for (int i = 0; ; i++)
            {
                if ((data & (1 << i)) != 0)
                {
                    pos = i;
                    break;
                }
            }
        }

        return pos;
    }   //leastSignificantSetBitPosition

    /**
     * This method returns the bit position of the most significant set bit of the given data.
     *
     * @param data specifies the data to determine its most significant set bit position.
     * @return 0-based most significant set bit position. -1 if no set bit.
     */
    public static int mostSignificantSetBitPosition(int data)
    {
        int pos = -1;

        if (data != 0)
        {
            for (int i = 0; ; i++)
            {
                if ((data & (0x80000000 >> i)) != 0)
                {
                    pos = 31 - i;
                    break;
                }
            }
        }

        return pos;
    }   //mostSignificantSetBitPosition

    /**
     * This method sets a bitmask with the given bit positions.
     *
     * @param bitPositions specifies the bit positions to be set to 1. Bit positions are 0-based.
     * @return bit mask.
     */
    public static int setBitMask(int... bitPositions)
    {
        int bitMask = 0;

        for (int pos : bitPositions)
        {
            bitMask |= 1 << pos;
        }

        return bitMask;
    }   //setBitMask

    /**
     * This method normalizes the given array of numbers such that no number exceeds +/- 1.0. If no number exceeds
     * the magnitude of 1.0, nothing will change, otherwise the original nums array will be modified in place.
     *
     * @param nums specifies the number array.
     */
    public static void normalizeInPlace(double[] nums)
    {
        double maxMag = maxMagnitude(nums);

        if (maxMag > 1.0)
        {
            for (int i = 0; i < nums.length; i++)
            {
                nums[i] /= maxMag;
            }
        }
    }   //normalizeInPlace

    /**
     * This method normalizes the given array of numbers such that no number exceeds +/- 1.0.
     *
     * @param nums specifies the number array.
     * @return normalized number array.
     */
    public static double[] normalize(double... nums)
    {
        double[] result = nums.clone();
        normalizeInPlace(result);

        return result;
    }   //normalize

    /**
     * This method rounds a double to the nearest integer.
     *
     * @param num Number to round.
     * @return Rounded to the nearest integer.
     */
    public static int round(double num)
    {
        return (int) Math.floor(num + 0.5);
    }   //round

    /**
     * This method checks if the given value is within the specified range.
     *
     * @param value     The value to be checked.
     * @param low       The low limit of the range.
     * @param high      The high limit of the range.
     * @param inclusive specifies true if the range is inclusive [low, high], otherwise the range is exclusive (low, high).
     * @return true if the value is within range, false otherwise.
     */
    public static boolean inRange(int value, int low, int high, boolean inclusive)
    {
        return inclusive ? value >= low && value <= high : value > low && value < high;
    }   //inRange

    /**
     * This method checks if the given value is within the specified range inclusive.
     *
     * @param value The value to be checked.
     * @param low   The low limit of the range.
     * @param high  The high limit of the range.
     * @return true if the value is within range, false otherwise.
     */
    public static boolean inRange(int value, int low, int high)
    {
        return inRange(value, low, high, true);
    }   //inRange

    /**
     * This method checks if the given value is within the specified range.
     *
     * @param value     The value to be checked.
     * @param low       The low limit of the range.
     * @param high      The high limit of the range.
     * @param inclusive specifies true if the range is inclusive [low, high], otherwise the range is exclusive (low,high).
     * @return true if the value is within range, false otherwise.
     */
    public static boolean inRange(double value, double low, double high, boolean inclusive)
    {
        return inclusive ? value >= low && value <= high : value > low && value < high;
    }   //inRange

    /**
     * This method checks if the given value is within the specified range inclusive.
     *
     * @param value The value to be checked.
     * @param low   The low limit of the range.
     * @param high  The high limit of the range.
     * @return true if the value is within range, false otherwise.
     */
    public static boolean inRange(double value, double low, double high)
    {
        return inRange(value, low, high, true);
    }   //inRange

    /**
     * This method clips the given value to the range limited by the given low and high limits.
     *
     * @param value     specifies the value to be clipped
     * @param lowLimit  specifies the low limit of the range.
     * @param highLimit specifies the high limit of the range.
     * @return the result of the clipped value.
     */
    public static int clipRange(int value, int lowLimit, int highLimit)
    {
        return Math.min(Math.max(value, lowLimit), highLimit);
    }   //clipRange

    /**
     * This method clips the given value to the range limited by the given low and high limits.
     *
     * @param value     specifies the value to be clipped
     * @param lowLimit  specifies the low limit of the range.
     * @param highLimit specifies the high limit of the range.
     * @return the result of the clipped value.
     */
    public static double clipRange(double value, double lowLimit, double highLimit)
    {
        return Math.min(Math.max(value, lowLimit), highLimit);
    }   //clipRange

    /**
     * This method clips the given value to the range between -1.0 and 1.0.
     *
     * @param value specifies the value to be clipped
     * @return the result of the clipped value.
     */
    public static double clipRange(double value)
    {
        return clipRange(value, -1.0, 1.0);
    }   //clipRange

    /**
     * This method scales the given value from the source range to the target range.
     *
     * @param value        specifies the value to be scaled.
     * @param lowSrcRange  specifies the low limit of the source range.
     * @param highSrcRange specifies the high limit of the source range.
     * @param lowDstRange  specifies the low limit of the target range.
     * @param highDstRange specifies the high limit of the target range
     * @return the result of the scaled value.
     */
    public static int scaleRange(int value, int lowSrcRange, int highSrcRange, int lowDstRange, int highDstRange)
    {
        return lowDstRange + (value - lowSrcRange) * (highDstRange - lowDstRange) / (highSrcRange - lowSrcRange);
    }   //scaleRange

    /**
     * This method scales the given value from the source range to the target range.
     *
     * @param value        specifies the value to be scaled.
     * @param lowSrcRange  specifies the low limit of the source range.
     * @param highSrcRange specifies the high limit of the source range.
     * @param lowDstRange  specifies the low limit of the target range.
     * @param highDstRange specifies the high limit of the target range
     * @return the result of the scaled value.
     */
    public static double scaleRange(double value, double lowSrcRange, double highSrcRange, double lowDstRange,
        double highDstRange)
    {
        return lowDstRange + (value - lowSrcRange) * (highDstRange - lowDstRange) / (highSrcRange - lowSrcRange);
    }   //scaleRange

    /**
     * This method checks if the given value is within the deadband range. If so, it returns 0.0 else it returns
     * the unchanged value.
     *
     * @param value    specifies the value to be checked.
     * @param deadband specifies the deadband zone.
     * @return the value 0.0 if within deadband, unaltered otherwise.
     */
    public static double applyDeadband(double value, double deadband)
    {
        return Math.abs(value) >= deadband ? value : 0.0;
    }   //applyDeadband

    /**
     * This method returns the indexed byte of an integer.
     *
     * @param data  specifies the integer value.
     * @param index specifies the byte index.
     * @return indexed byte of the integer.
     */
    public static byte intToByte(int data, int index)
    {
        return (byte) (data >> (8 * index));
    }   //intToByte

    /**
     * This method combines two bytes into an integer.
     *
     * @param data1 specifies the lowest byte.
     * @param data2 specifies the next lowest byte.
     * @param data3 specifies the next byte.
     * @param data4 specifies the highest byte.
     * @return the converted integer.
     */
    public static int bytesToInt(byte data1, byte data2, byte data3, byte data4)
    {
        return (data4 << 24) & 0xff000000 | (data3 << 16) & 0x00ff0000 | (data2 << 8) & 0x0000ff00 | data1 & 0x000000ff;
    }   //bytesToInt

    /**
     * This method combines two bytes into an integer.
     *
     * @param low  specifies the low byte.
     * @param high specifies the high byte.
     * @return the converted integer.
     */
    public static int bytesToInt(byte low, byte high)
    {
        return bytesToInt(low, high, (byte) 0, (byte) 0);
    }   //bytesToInt

    /**
     * This method converts a byte into an integer.
     *
     * @param data specifies the byte data.
     * @return the converted integer.
     */
    public static int bytesToInt(byte data)
    {
        return (int) data;
    }   //bytesToInt

    /**
     * This method combines two bytes into a short.
     *
     * @param low  specifies the low byte.
     * @param high specifies the high byte.
     * @return the converted short.
     */
    public static short bytesToShort(byte low, byte high)
    {
        return (short) bytesToInt(low, high);
    }   //bytesToShort

    /**
     * Convert a point from a polar coordinate system to a cartesian coordinate system.
     *
     * @param r Magnitude of vector
     * @param theta Direction of vector, in degrees clockwise from 0 (+y)
     * @return Vector in a cartesian coordinate system representing the same point.
     */
    public static RealVector polarToCartesian(double r, double theta)
    {
        double thetaRad = Math.toRadians(theta);
        return MatrixUtils.createRealVector(new double[] { r * Math.sin(thetaRad), r * Math.cos(thetaRad) });
    }   //polarToCartesian

    /**
     * Rotate a point counter-clockwise about the origin.
     *
     * @param vector The vector to rotate.
     * @param angle  The angle in degrees to rotate by.
     * @return The vector after the rotation transformation.
     */
    public static RealVector rotateCCW(RealVector vector, double angle)
    {
        return createCCWRotationMatrix(angle).operate(vector);
    }   //rotateCCW

    /**
     * Rotate a point clockwise about the origin.
     *
     * @param vector The vector to rotate.
     * @param angle  The angle in degrees to rotate by.
     * @return The vector after the rotation transformation.
     */
    public static RealVector rotateCW(RealVector vector, double angle)
    {
        return createCWRotationMatrix(angle).operate(vector);
    }   //rotateCW

    /**
     * Create a rotation matrix that will rotate a point counter-clockwise
     * about the origin by a specific number of degrees.
     *
     * @param angle The angle in degrees to rotate by.
     * @return A rotation matrix describing a counter-clockwise rotation by <code>angle</code> degrees.
     */
    public static RealMatrix createCCWRotationMatrix(double angle)
    {
        double angleRad = Math.toRadians(angle);
        return MatrixUtils.createRealMatrix(
            new double[][] { { Math.cos(angleRad), -Math.sin(angleRad) }, { Math.sin(angleRad), Math.cos(angleRad) } });
    }   //createCCWRotationMatrix

    /**
     * Create a rotation matrix that will rotate a point clockwise
     * about the origin by a specific number of degrees.
     *
     * @param angle The angle in degrees to rotate by.
     * @return A rotation matrix describing a clockwise rotation by <code>angle</code> degrees.
     */
    public static RealMatrix createCWRotationMatrix(double angle)
    {
        return createCCWRotationMatrix(angle).transpose();
    }   //createCWRotationMatrix

}   //class TrcUtil
