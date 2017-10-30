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

package ftclib;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import trclib.TrcAccelerometer;
import trclib.TrcDbgTrace;
import trclib.TrcGyro;
import trclib.TrcUtil;

/**
 * This class implements the BNO055 IMU which is actually an Adafruit BNO055. It encapsulates two sub-classes:
 * a 3-axis gyro and a 3-axis accelerometer.
 */
public class FtcBNO055Imu
{
    private static final String moduleName = "FtcBNO055Imu";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private static final boolean USE_QUATERNION = true;

    /**
     * This class implements the gyro part of hte BNO055 IMU. It extends TrcGyro so that it implements the standard
     * gyro interface.
     */
    private class Gyro extends TrcGyro
    {
        private AngularVelocity turnRateData = null;
        private long turnRateTagId = -1;
        private double[] eulerAngles = new double[3];
        private Orientation headingData = null;
        private long headingTagId = -1;

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param instanceName specifies the instance name.
         */
        public Gyro(String instanceName)
        {
            //
            // BNO055 IMU has a 3-axis gyro. The angular orientation data it returns is in Ordinal system.
            // So we need to convert it to Cartesian system.
            //
            super(instanceName, 3,
                    GYRO_HAS_X_AXIS | GYRO_HAS_Y_AXIS | GYRO_HAS_Z_AXIS | GYRO_CONVERT_TO_CARTESIAN, null);
        }   //Gyro

        /**
         * This method returns the Euler angles of all 3 axes from quaternion orientation.
         *
         * @param angles specifies the array to hold the angles of the 3 axes.
         */
        private void getEulerAngles(double[] angles)
        {
            Quaternion q = imu.getQuaternionOrientation();
            //
            // 0: roll (x-axis rotation)
            // 1: pitch (y-axis rotation)
            // 2: yaw (z-axis rotation)
            //
            angles[0] = Math.toDegrees(Math.atan2(2.0*(q.w*q.x + q.y*q.z), 1.0 - 2.0*(q.x*q.x + q.y*q.y)));
            double sinp = 2.0*(q.w*q.y - q.z*q.x);
            angles[1] = Math.toDegrees(Math.abs(sinp) >= 1.0? Math.signum(sinp)*(Math.PI/2.0): Math.asin(sinp));
            angles[2] = Math.toDegrees(Math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z)));
        }   //getEulerAngles

        //
        // Implements TrcGyro abstract methods.
        //

        /**
         * This method returns the raw data of the specified type for the x-axis.
         *
         * @param dataType specifies the data type.
         * @return raw data of the specified type for the x-axis.
         */
        @Override
        public SensorData<Double> getRawXData(DataType dataType)
        {
            final String funcName = "getRawXData";
            double value = 0.0;
            long currTagId = FtcOpMode.getLoopCounter();

            if (dataType == DataType.ROTATION_RATE)
            {
                if (currTagId != turnRateTagId)
                {
                    turnRateData = imu.getAngularVelocity();
                    turnRateTagId = currTagId;
                }
                value = turnRateData.xRotationRate;
            }
            else if (dataType == DataType.HEADING)
            {
                if (currTagId != headingTagId)
                {
                    if (USE_QUATERNION)
                    {
                        getEulerAngles(eulerAngles);
                    }
                    else
                    {
                        headingData = imu.getAngularOrientation(
                                AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                    }
                    headingTagId = currTagId;
                }

                if (USE_QUATERNION)
                {
                    value = eulerAngles[0];
                }
                else
                {
                    value = headingData.firstAngle;
                }
            }
            SensorData<Double> data = new SensorData<>(TrcUtil.getCurrentTime(), value);

            if (debugEnabled)
            {
                dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
                dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                        "=(timestamp:%.3f,value:%f", data.timestamp, data.value);
            }

            return data;
        }   //getRawXData

        /**
         * This method returns the raw data of the specified type for the y-axis.
         *
         * @param dataType specifies the data type.
         * @return raw data of the specified type for the y-axis.
         */
        @Override
        public SensorData<Double> getRawYData(DataType dataType)
        {
            final String funcName = "getRawYData";
            double value = 0.0;
            long currTagId = FtcOpMode.getLoopCounter();

            if (dataType == DataType.ROTATION_RATE)
            {
                if (currTagId != turnRateTagId)
                {
                    turnRateData = imu.getAngularVelocity();
                    turnRateTagId = currTagId;
                }
                value = turnRateData.yRotationRate;
            }
            else if (dataType == DataType.HEADING)
            {
                if (currTagId != headingTagId)
                {
                    if (USE_QUATERNION)
                    {
                        getEulerAngles(eulerAngles);
                    }
                    else
                    {
                        headingData = imu.getAngularOrientation(
                                AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                    }
                    headingTagId = currTagId;
                }

                if (USE_QUATERNION)
                {
                    value = eulerAngles[1];
                }
                else
                {
                    value = headingData.secondAngle;
                }
            }
            SensorData<Double> data = new SensorData<>(TrcUtil.getCurrentTime(), value);

            if (debugEnabled)
            {
                dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
                dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                        "=(timestamp:%.3f,value:%f", data.timestamp, data.value);
            }

            return data;
        }   //getRawYData

        /**
         * This method returns the raw data of the specified type for the z-axis.
         *
         * @param dataType specifies the data type.
         * @return raw data of the specified type for the z-axis.
         */
        @Override
        public SensorData<Double> getRawZData(DataType dataType)
        {
            final String funcName = "getRawZData";
            double value = 0.0;
            long currTagId = FtcOpMode.getLoopCounter();

            if (dataType == DataType.ROTATION_RATE)
            {
                if (currTagId != turnRateTagId)
                {
                    turnRateData = imu.getAngularVelocity();
                    turnRateTagId = currTagId;
                }
                value = turnRateData.zRotationRate;
            }
            else if (dataType == DataType.HEADING)
            {
                if (currTagId != headingTagId)
                {
                    if (USE_QUATERNION)
                    {
                        getEulerAngles(eulerAngles);
                    }
                    else
                    {
                        headingData = imu.getAngularOrientation(
                                AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                    }
                    headingTagId = currTagId;
                }

                if (USE_QUATERNION)
                {
                    value = -eulerAngles[2];
                }
                else
                {
                    //
                    // The Z-axis returns positive heading in the anticlockwise direction, so we must negate it for
                    // our convention.
                    //
                    value = -headingData.thirdAngle;
                }
            }
            SensorData<Double> data = new SensorData<>(TrcUtil.getCurrentTime(), value);

            if (debugEnabled)
            {
                dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
                dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                        "=(timestamp:%.3f,value:%f", data.timestamp, data.value);
            }

            return data;
        }   //getRawZData

    }   //class Gyro

    /**
     * This class implements the accelerometer part of hte BNO055 IMU. It extends TrcAccelerometer so that it
     * implements the standard accelerometer interface.
     */
    public class Accelerometer extends TrcAccelerometer
    {
        private Acceleration accelData = null;
        private long accelTagId = -1;
        private Velocity velData = null;
        private long velTagId = -1;
        private Position posData = null;
        private long posTagId = -1;

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param instanceName specifies the instance name.
         */
        public Accelerometer(String instanceName)
        {
            //
            // BNO055 IMU has a 3-axis accelerometer.
            //
            super(instanceName, 3, ACCEL_HAS_X_AXIS | ACCEL_HAS_Y_AXIS | ACCEL_HAS_Z_AXIS, null);
        }   //Accelerometer

        //
        // Implements TrcAccelerometer abstract methods.
        //

        /**
         * This method returns the raw data of the specified type for the x-axis.
         *
         * @param dataType specifies the data type.
         * @return raw data of the specified type for the x-axis.
         */
        @Override
        public SensorData<Double> getRawXData(DataType dataType)
        {
            final String funcName = "getRawXData";
            double value = 0.0;
            long currTagId = FtcOpMode.getLoopCounter();

            if (dataType == DataType.ACCELERATION)
            {
                if (currTagId != accelTagId)
                {
                    accelData = imu.getAcceleration();
                    accelTagId = currTagId;
                }
                value = accelData.xAccel;
            }
            else if (dataType == DataType.VELOCITY)
            {
                if (currTagId != velTagId)
                {
                    velData = imu.getVelocity();
                    velTagId = currTagId;
                }
                value = velData.xVeloc;
            }
            else if (dataType == DataType.DISTANCE)
            {
                if (currTagId != posTagId)
                {
                    posData = imu.getPosition();
                    posTagId = currTagId;
                }
                value = posData.x;
            }
            SensorData<Double> data = new SensorData<>(TrcUtil.getCurrentTime(), value);

            if (debugEnabled)
            {
                dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
                dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                        "=(timestamp:%.3f,value:%f", data.timestamp, data.value);
            }

            return data;
        }   //getRawXData

        /**
         * This method returns the raw data of the specified type for the y-axis.
         *
         * @param dataType specifies the data type.
         * @return raw data of the specified type for the y-axis.
         */
        @Override
        public SensorData<Double> getRawYData(DataType dataType)
        {
            final String funcName = "getRawYData";
            double value = 0.0;
            long currTagId = FtcOpMode.getLoopCounter();

            if (dataType == DataType.ACCELERATION)
            {
                if (currTagId != accelTagId)
                {
                    accelData = imu.getAcceleration();
                    accelTagId = currTagId;
                }
                value = accelData.yAccel;
            }
            else if (dataType == DataType.VELOCITY)
            {
                if (currTagId != velTagId)
                {
                    velData = imu.getVelocity();
                    velTagId = currTagId;
                }
                value = velData.yVeloc;
            }
            else if (dataType == DataType.DISTANCE)
            {
                if (currTagId != posTagId)
                {
                    posData = imu.getPosition();
                    posTagId = currTagId;
                }
                value = posData.y;
            }
            SensorData<Double> data = new SensorData<>(TrcUtil.getCurrentTime(), value);

            if (debugEnabled)
            {
                dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
                dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                        "=(timestamp:%.3f,value:%f", data.timestamp, data.value);
            }

            return data;
        }   //getRawYData

        /**
         * This method returns the raw data of the specified type for the z-axis.
         *
         * @param dataType specifies the data type.
         * @return raw data of the specified type for the z-axis.
         */
        @Override
        public SensorData<Double> getRawZData(DataType dataType)
        {
            final String funcName = "getRawZData";
            double value = 0.0;
            long currTagId = FtcOpMode.getLoopCounter();

            if (dataType == DataType.ACCELERATION)
            {
                if (currTagId != accelTagId)
                {
                    accelData = imu.getAcceleration();
                    accelTagId = currTagId;
                }
                value = accelData.zAccel;
            }
            else if (dataType == DataType.VELOCITY)
            {
                if (currTagId != velTagId)
                {
                    velData = imu.getVelocity();
                    velTagId = currTagId;
                }
                value = velData.zVeloc;
            }
            else if (dataType == DataType.DISTANCE)
            {
                if (currTagId != posTagId)
                {
                    posData = imu.getPosition();
                    posTagId = currTagId;
                }
                value = posData.z;
            }
            SensorData<Double> data = new SensorData<>(TrcUtil.getCurrentTime(), value);

            if (debugEnabled)
            {
                dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
                dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                        "=(timestamp:%.3f,value:%f", data.timestamp, data.value);
            }

            return data;
        }   //getRawZData

    }   //class Accelerometer

    public BNO055IMU imu = null;
    public TrcGyro gyro = null;
    public TrcAccelerometer accel = null;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     */
    public FtcBNO055Imu(HardwareMap hardwareMap, String instanceName)
    {
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }
        //
        // Initialize the BNO055 IMU.
        //
        BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
        if (USE_QUATERNION)
        {
            imuParams.mode = BNO055IMU.SensorMode.IMU;
            imuParams.useExternalCrystal = true;
            imuParams.pitchMode = BNO055IMU.PitchMode.WINDOWS;
        }
        else
        {
            imuParams.calibrationDataFile = "BNO055IMUCalibration.json";
        }
        imuParams.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParams.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParams.loggingEnabled = true;
        imuParams.loggingTag = "IMU";
        imuParams.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, instanceName);
        imu.initialize(imuParams);
        //
        // Create the gyro object of the IMU.
        // Note that the heading data on the z-axis is in Ordinal system with a range of -180 to 180 degrees.
        // So we need to initialize the Cartesian converter with the range values.
        //
        gyro = new Gyro(instanceName);
        //
        // Note:
        // We can convert only X (roll) and Z (yaw) axes to Cartesian.
        // For Y (pitch), when pointing upright, it will return 90-degree but pitching forward or backward will
        // yield the same decrement. So one can't tell if it is rotating forward or backward. This makes it
        // impossible to do the Cartesian conversion.
        //
        gyro.setXValueRange(-180.0, 180.0);
        gyro.setZValueRange(-180.0, 180.0);
        //
        // Create the accelerometer object of the IMU.
        //
        accel = new Accelerometer(instanceName);
    }   //FtcBNO055Imu

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FtcBNO055Imu(String instanceName)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName);
    }   //FtcBNO055Imu

}   //class FtcBNO055Imu
