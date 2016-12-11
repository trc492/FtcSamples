package samples;

import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import ftclib.FtcDcMotor;
import ftclib.FtcMRI2cColorSensor;
import ftclib.FtcMRI2cGyro;
import ftclib.FtcOpMode;

@TeleOp(name="Test: Sensor Sample Time", group="3543TestSamples")
@Disabled
public class FtcTestSensorSampleTime extends FtcOpMode
{
    private static final boolean USE_MY_GYRO = true;
    private static final boolean USE_MY_COLOR_SENSOR = false;
    private static final boolean USE_SDK_COLOR_SENSOR = false;

    private enum SensorType
    {
        DRIVEBASE_ENCODERS,
        GYRO
    }

    private static final String TAG = "TrcDbg";
    private static final double DRIVE_POWER = 0.2;
    private static final double TURN_POWER = 0.5;
    private static SensorType sensorType = SensorType.GYRO;
    private static final boolean LEFTWHEEL_INVERTED = false;
    private static final boolean RIGHTWHEEL_INVERTED = true;

    private FtcDcMotor lfWheel;
    private FtcDcMotor rfWheel;
    private FtcDcMotor lrWheel;
    private FtcDcMotor rrWheel;
    private ModernRoboticsI2cGyro gyro;
    private FtcMRI2cGyro i2cGyro;
    private ColorSensor colorSensor;
    private FtcMRI2cColorSensor i2cColorSensor;

    private long minLoopInterval = Long.MAX_VALUE;
    private long maxLoopInterval = Long.MIN_VALUE;
    private long loopCount = 0;
    private long prevLoopTime = 0;

    private long minSampleInterval = Long.MAX_VALUE;
    private long maxSampleInterval = Long.MIN_VALUE;
    private long sampleCount = 0;
    private long prevSampleTime;

    private long startTime;
    private double prevSample;

    @Override
    public void initRobot()
    {
        Log.i(TAG, "initRobot started...");
        lfWheel = new FtcDcMotor("lfWheel");
        rfWheel = new FtcDcMotor("rfWheel");
        lrWheel = new FtcDcMotor("lrWheel");
        rrWheel = new FtcDcMotor("rrWheel");
        lfWheel.setInverted(LEFTWHEEL_INVERTED);
        lrWheel.setInverted(LEFTWHEEL_INVERTED);
        rfWheel.setInverted(RIGHTWHEEL_INVERTED);
        rrWheel.setInverted(RIGHTWHEEL_INVERTED);

        lfWheel.resetPosition();
        rfWheel.resetPosition();
        lrWheel.resetPosition();
        rrWheel.resetPosition();

        if (USE_MY_GYRO)
        {
            i2cGyro = new FtcMRI2cGyro("i2cGyroSensor");
            i2cGyro.resetZIntegrator();
            gyro = null;
        }
        else
        {
            gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyroSensor");
            gyro.resetZAxisIntegrator();
            i2cGyro = null;
        }

        if (USE_MY_COLOR_SENSOR)
        {
            i2cColorSensor = new FtcMRI2cColorSensor("i2cColorSensor", 0x40, false);
        }
        else
        {
            i2cColorSensor = null;
        }

        if (USE_SDK_COLOR_SENSOR)
        {
            colorSensor = hardwareMap.colorSensor.get("colorSensor");
        }
        else
        {
            colorSensor = null;
        }

        Log.i(TAG, "initRobot completed!");
    }   //initRobot

    @Override
    public void startMode()
    {
        Log.i(TAG, String.format("startMode [%d]", System.nanoTime()));
        startTime = System.nanoTime();
        prevSampleTime = startTime;
//        prevSample = getSensorValue();
    }   //startMode

    @Override
    public void stopMode()
    {
        lfWheel.setPower(0.0);
        lrWheel.setPower(0.0);
        rfWheel.setPower(0.0);
        rrWheel.setPower(0.0);

        long endTime = System.nanoTime();
        Log.i(TAG, String.format(
                "Loop: MinInterval=%7.3f, MaxInterval=%7.3f, AvgInterval=%7.3f",
                minLoopInterval/1000000.0, maxLoopInterval/1000000.0,
                (endTime - startTime)/1000000.0/loopCount));
        Log.i(TAG, String.format(
                "Sensor: MinSampleInterval=%7.3f, MaxSampleInterval=%7.3f, AvgSampleInterval=%7.3f",
                minSampleInterval/1000000.0, maxSampleInterval/1000000.0,
                (endTime - startTime)/1000000.0/sampleCount));
    }   //stopMode

    @Override
    public void runContinuous(double elapsedTime)
    {
        long currTime = System.nanoTime();
        double  currSample = getSensorValue();

        Log.i(TAG, String.format("RunContinuous [%7.3f]", (currTime - startTime)/1000000.0));
        if (prevLoopTime != 0)
        {
            long loopInterval = currTime - prevLoopTime;

            if (currSample != prevSample)
            {
                long sampleTime = currTime - prevSampleTime;
                sampleCount++;
                prevSample = currSample;
                prevSampleTime = currTime;
                if (sampleTime < minSampleInterval)
                    minSampleInterval = sampleTime;
                else if (sampleTime > maxSampleInterval)
                    maxSampleInterval = sampleTime;
            }

            if (loopInterval < minLoopInterval)
            {
                minLoopInterval = loopInterval;
            }
            else if (loopInterval > maxLoopInterval)
            {
                maxLoopInterval = loopInterval;
            }

            int sensorValue;
            if (USE_MY_COLOR_SENSOR)
            {
                sensorValue = (Integer)i2cColorSensor.getWhiteValue().value;
            }
            else if (USE_SDK_COLOR_SENSOR)
            {
                sensorValue = colorSensor.argb();
            }
            else
            {
                sensorValue = 0;
            }

            runRobot(String.format("[%4d:%7.3f] LoopInterval=%7.3f, ",
                     loopCount, (currTime - startTime)/1000000.0, loopInterval/1000000.0),
                     String.format(", sensor=%d", sensorValue));
        }
        prevLoopTime = currTime;
        loopCount++;
    }   //runContinuous

    private double getSensorValue()
    {
        double value = 0;

        switch (sensorType)
        {
            case DRIVEBASE_ENCODERS:
                value = (lfWheel.getPosition() + rfWheel.getPosition() +
                         lrWheel.getPosition() + rrWheel.getPosition())/4;
                break;

            case GYRO:
                if (USE_MY_GYRO)
                {
                    Log.i(TAG, "About to get heading value.");
                    value = (Integer)i2cGyro.getIntegratedZ().value;
                    Log.i(TAG, String.format("Got heading value %f.", value));
                }
                else
                {
                    value = -gyro.getIntegratedZValue();
                }
                break;
        }

        return value;
    }

    private void runRobot(String prefix, String subfix)
    {
        switch (sensorType)
        {
            case DRIVEBASE_ENCODERS:
                //
                // Driving forward and checking encoders.
                //
                lfWheel.setPower(DRIVE_POWER);
                rfWheel.setPower(DRIVE_POWER);
                lrWheel.setPower(DRIVE_POWER);
                rrWheel.setPower(DRIVE_POWER);
                Log.i(TAG, prefix + String.format("lf=%d, rf=%d, lr=%d, rr=%d" + subfix,
                        lfWheel.getPosition(), rfWheel.getPosition(), lrWheel.getPosition(), rrWheel.getPosition()));
                break;

            case GYRO:
                //
                // Turning right and checking gyro.
                //
                lfWheel.setPower(TURN_POWER);
                lrWheel.setPower(TURN_POWER);
                rfWheel.setPower(-TURN_POWER);
                rrWheel.setPower(-TURN_POWER);
                if (USE_MY_GYRO)
                {
                    Log.i(TAG, prefix + String.format("heading=%d", (Integer)i2cGyro.getIntegratedZ().value));
                }
                else
                {
                    Log.i(TAG, prefix + String.format("heading=%d", -gyro.getIntegratedZValue()));
                }
                break;
        }
    }

}   //class FtcTestSensorSampleTime
