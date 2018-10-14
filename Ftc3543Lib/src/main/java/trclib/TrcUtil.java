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

import java.text.SimpleDateFormat;
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
        return System.nanoTime()/1000000000.0;
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
     *
     * @return sum of the numbers.
     */
    public static double sum(double... nums)
    {
        double total = 0.0;

        for (double num: nums)
        {
            total += num;
        }

        return total;
    }   //sum

    /**
     * This method calculates and returns the average of the numbers in the given array.
     *
     * @param nums specifies the number array.
     * @return average of all numbers in the array. If the array is empty, return 0.
     */
    public static double average(double... nums)
    {
        return sum(nums)/nums.length;
//        return Arrays.stream(nums).average().orElse(0.0);
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

        for (double num: nums)
        {
            total += num*num;
        }

        return Math.sqrt(total);
//        return Math.sqrt(Arrays.stream(nums).map(e -> e*e).sum());
    }   //magnitude

    /**
     * This method returns the maximum magnitude of numbers in the specified array.
     *
     * @param nums specifies the number array.
     *
     * @return maximum magnitude of the numbers in the array.
     */
    public static double maxMagnitude(double... nums)
    {
        double maxMag = Math.abs(nums[0]);

        for (double num: nums)
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
     * This method normalizes the given array of numbers such that no number exceeds +/- 1.0. If no number exceeds
     * the magnitude of 1.0, nothing will change, otherwise the original nums array will be modified in place.
     *
     * @param nums specifies the number array.
     * @return normalized number array.
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
//        double maxMagnitude = Arrays.stream(nums).map(Math::abs).max().orElse(0.0);
//        return maxMagnitude > 1.0? Arrays.stream(nums).map(x -> x/maxMagnitude).toArray(): nums;
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
     * This method clips the given value to the range limited by the given low and high limits.
     *
     * @param value specifies the value to be clipped
     * @param lowLimit specifies the low limit of the range.
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
     * @param value specifies the value to be clipped
     * @param lowLimit specifies the low limit of the range.
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
     * @param value specifies the value to be scaled.
     * @param lowSrcRange specifies the low limit of the source range.
     * @param highSrcRange specifies the high limit of the source range.
     * @param lowDstRange specifies the low limit of the target range.
     * @param highDstRange specifies the high limit of the target range
     * @return the result of the scaled value.
     */
    public static int scaleRange(int value, int lowSrcRange, int highSrcRange, int lowDstRange, int highDstRange)
    {
        return lowDstRange + (value - lowSrcRange)*(highDstRange - lowDstRange)/(highSrcRange - lowSrcRange);
    }   //scaleRange

    /**
     * This method scales the given value from the source range to the target range.
     *
     * @param value specifies the value to be scaled.
     * @param lowSrcRange specifies the low limit of the source range.
     * @param highSrcRange specifies the high limit of the source range.
     * @param lowDstRange specifies the low limit of the target range.
     * @param highDstRange specifies the high limit of the target range
     * @return the result of the scaled value.
     */
    public static double scaleRange(
            double value, double lowSrcRange, double highSrcRange, double lowDstRange, double highDstRange)
    {
        return lowDstRange + (value - lowSrcRange)*(highDstRange - lowDstRange)/(highSrcRange - lowSrcRange);
    }   //scaleRange

    /**
     * This method checks if the given value is within the deadband range. If so, it returns 0.0 else it returns
     * the unchanged value.
     *
     * @param value specifies the value to be checked.
     * @param deadband specifies the deadband zone.
     * @return the value 0.0 if within deadband, unaltered otherwise.
     */
    public static double applyDeadband(double value, double deadband)
    {
        return Math.abs(value) >= deadband? value: 0.0;
    }   //applyDeadband

    /**
     * This method combines two bytes into an integer.
     *
     * @param low specifies the low byte.
     * @param high specifies the high byte.
     *
     * @return the converted integer.
     */
    public static int bytesToInt(byte low, byte high)
    {
        return ((int)low & 0xff) | (((int)high & 0xff) << 8);
    }   //bytesToInt

    /**
     * This method converts a byte into an integer.
     *
     * @param data specifies the byte data.
     *
     * @return the converted integer.
     */
    public static int bytesToInt(byte data)
    {
        return bytesToInt(data, (byte)0);
    }   //bytesToInt

    /**
     * This method combines two bytes into a short.
     *
     * @param low specifies the low byte.
     * @param high specifies the high byte.
     *
     * @return the converted short.
     */
    public static short bytesToShort(byte low, byte high)
    {
        return (short)bytesToInt(low, high);
    }   //bytesToShort

}   //class TrcUtil
