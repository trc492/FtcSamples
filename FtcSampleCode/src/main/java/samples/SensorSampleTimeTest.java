package samples;

import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Test: Simple Sensor Sample Time", group="3543TestSamples")
@Disabled
public class SensorSampleTimeTest extends LinearOpMode
{
    private enum SensorType
    {
        DRIVEBASE_ENCODERS,
        GYRO
    }

    private static final String TAG = "TrcDbg";
    private static final double DRIVE_POWER = 0.2;
    private static final double TURN_POWER = 0.5;
    private static SensorType sensorType = SensorType.GYRO;
    private static final DcMotor.Direction LEFTWHEEL_DIRECTION = DcMotor.Direction.FORWARD;
    private static final DcMotor.Direction RIGHTWHEEL_DIRECTION = DcMotor.Direction.REVERSE;

    private DcMotor lfWheel;
    private DcMotor rfWheel;
    private DcMotor lrWheel;
    private DcMotor rrWheel;
    private ModernRoboticsI2cGyro gyro;

    public void runOpMode()
    {
        initRobot();

        waitForStart();

        long minLoopInterval = Long.MAX_VALUE;
        long maxLoopInterval = Long.MIN_VALUE;
        long loopCount = 0;
        long prevLoopTime = 0;

        long minSampleInterval = Long.MAX_VALUE;
        long maxSampleInterval = Long.MIN_VALUE;
        long sampleCount = 0;
        long prevSampleTime;

        long startTime = System.nanoTime();
        prevSampleTime = startTime;
        int prevSample = getSensorValue();

        while (opModeIsActive())
        {
            long currTime = System.nanoTime();
            int currSample = getSensorValue();
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
                runRobot(String.format("[%4d:%7.3f] LoopInterval=%7.3f, ",
                        loopCount, (currTime - startTime)/1000000.0, loopInterval/1000000.0));
            }

            prevLoopTime = currTime;
            loopCount++;
        }
        stopRobot();

        long endTime = System.nanoTime();
        Log.i(TAG, String.format(
                "Loop: MinInterval=%7.3f, MaxInterval=%7.3f, AvgInterval=%7.3f",
                minLoopInterval/1000000.0, maxLoopInterval/1000000.0,
                (endTime - startTime)/1000000.0/loopCount));
        Log.i(TAG, String.format(
                "Sensor: MinSampleInterval=%7.3f, MaxSampleInterval=%7.3f, AvgSampleInterval=%7.3f",
                minSampleInterval/1000000.0, maxSampleInterval/1000000.0,
                (endTime - startTime)/1000000.0/sampleCount));
    }   //runOpMode

    private void initRobot()
    {
        lfWheel = hardwareMap.dcMotor.get("lfWheel");
        rfWheel = hardwareMap.dcMotor.get("rfWheel");
        lrWheel = hardwareMap.dcMotor.get("lrWheel");
        rrWheel = hardwareMap.dcMotor.get("rrWheel");
        lfWheel.setDirection(LEFTWHEEL_DIRECTION);
        lrWheel.setDirection(LEFTWHEEL_DIRECTION);
        rfWheel.setDirection(RIGHTWHEEL_DIRECTION);
        rrWheel.setDirection(RIGHTWHEEL_DIRECTION);

        lfWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rfWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lrWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rrWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lfWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rfWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lrWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rrWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyroSensor");
        gyro.resetZAxisIntegrator();
    }   //initRobot

    private int getSensorValue()
    {
        int value = 0;

        switch (sensorType)
        {
            case DRIVEBASE_ENCODERS:
                value = (lfWheel.getCurrentPosition() + rfWheel.getCurrentPosition() +
                         lrWheel.getCurrentPosition() + rrWheel.getCurrentPosition())/4;
                break;

            case GYRO:
                value = -gyro.getIntegratedZValue();
                break;
        }

        return value;
    }   //getSensorValue

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
                Log.i(TAG, prefix + String.format("lf=%d, rf=%d, lr=%d, rr=%d",
                        lfWheel.getCurrentPosition(), rfWheel.getCurrentPosition(),
                        lrWheel.getCurrentPosition(), rrWheel.getCurrentPosition()));
                break;

            case GYRO:
                //
                // Turning right and checking gyro.
                //
                lfWheel.setPower(TURN_POWER);
                lrWheel.setPower(TURN_POWER);
                rfWheel.setPower(-TURN_POWER);
                rrWheel.setPower(-TURN_POWER);
                Log.i(TAG, prefix + String.format("heading=%d", -gyro.getIntegratedZValue()));
                break;
        }
    }   //runRobot

    private void stopRobot()
    {
        lfWheel.setPower(0.0);
        lrWheel.setPower(0.0);
        rfWheel.setPower(0.0);
        rrWheel.setPower(0.0);
    }   //stopRobot

}   //class SensorSampleTimeTest
