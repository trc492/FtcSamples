package samples;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Locale;

import ftclib.FtcAnalogGyro;
import ftclib.FtcAndroidGyro;
import ftclib.FtcDcMotor;
import ftclib.FtcMRGyro;
import ftclib.FtcOpMode;
import trclib.TrcGyro;

@TeleOp(name="Test: Sensor Sample Time", group="3543TestSamples")
@Disabled
public class FtcTestSensorSampleTime extends FtcOpMode
{
    private enum SensorType
    {
        DRIVEBASE_ENCODERS,
        MR_GYRO,
        ANALOG_GYRO,
        ANDROID_GYRO
    }

    private static final String TAG = "TrcDbg";
    private static final double DRIVE_POWER = 0.2;
    private static final double TURN_POWER = 0.5;
    private static SensorType sensorType = SensorType.MR_GYRO;
    private static final boolean LEFTWHEEL_INVERTED = false;
    private static final boolean RIGHTWHEEL_INVERTED = true;
    private static final double ANALOG_GYRO_VOLT_PER_DEG_PER_SEC = 0.007;

    private FtcDcMotor lfWheel;
    private FtcDcMotor rfWheel;
    private FtcDcMotor lrWheel;
    private FtcDcMotor rrWheel;
    private TrcGyro gyro = null;

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

        switch (sensorType)
        {
            case MR_GYRO:
                gyro = new FtcMRGyro("gyroSensor");
                ((FtcMRGyro)gyro).calibrate();
                break;

            case ANALOG_GYRO:
                gyro = new FtcAnalogGyro("analogGyro", ANALOG_GYRO_VOLT_PER_DEG_PER_SEC);
                ((FtcAnalogGyro)gyro).calibrate();
                break;

            case ANDROID_GYRO:
                gyro = new FtcAndroidGyro("androidGyro");
                ((FtcAndroidGyro)gyro).calibrate();
                break;
        }

        Log.i(TAG, "initRobot completed!");
    }   //initRobot

    @Override
    public void startMode()
    {
        switch (sensorType)
        {
            case MR_GYRO:
            case ANALOG_GYRO:
            case ANDROID_GYRO:
                gyro.setEnabled(true);
        }

        startTime = System.nanoTime();
        prevSampleTime = startTime;
        Log.i(TAG, String.format("startMode [%d]", System.nanoTime()));
        prevSample = getSensorValue();
    }   //startMode

    @Override
    public void stopMode()
    {
        lfWheel.setPower(0.0);
        lrWheel.setPower(0.0);
        rfWheel.setPower(0.0);
        rrWheel.setPower(0.0);

        switch (sensorType)
        {
            case MR_GYRO:
            case ANALOG_GYRO:
            case ANDROID_GYRO:
                gyro.setEnabled(false);
        }

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

            runRobot(String.format(Locale.US, "[%4d:%7.3f] LoopInterval=%7.3f, Sensor=%7.2f: ",
                                   loopCount, (currTime - startTime)/1000000.0, loopInterval/1000000.0,
                                   getSensorValue()));
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
                         lrWheel.getPosition() + rrWheel.getPosition())/4.0;
                break;

            case MR_GYRO:
            case ANALOG_GYRO:
            case ANDROID_GYRO:
                value = (Double)gyro.getZHeading().value;
                break;
        }

        return value;
    }

    private void runRobot(String prefix)
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
                Log.i(TAG, prefix + String.format("lf=%.0f, rf=%.0f, lr=%.0f, rr=%.0f",
                                                  lfWheel.getPosition(), rfWheel.getPosition(),
                                                  lrWheel.getPosition(), rrWheel.getPosition()));
                break;

            case MR_GYRO:
            case ANALOG_GYRO:
            case ANDROID_GYRO:
                //
                // Turning right and checking gyro.
                //
                lfWheel.setPower(TURN_POWER);
                lrWheel.setPower(TURN_POWER);
                rfWheel.setPower(-TURN_POWER);
                rrWheel.setPower(-TURN_POWER);
                Log.i(TAG, prefix);
                break;
        }
    }

}   //class FtcTestSensorSampleTime
