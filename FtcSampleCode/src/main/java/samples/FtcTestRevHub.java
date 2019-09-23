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

package samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import ftclib.FtcAnalogInput;
import ftclib.FtcBNO055Imu;
import ftclib.FtcColorSensor;
import ftclib.FtcDcMotor;
import ftclib.FtcDigitalInput;
import ftclib.FtcGamepad;
import ftclib.FtcOpMode;
import ftclib.FtcServo;
import trclib.TrcGameController;
import trclib.TrcRobot;

@TeleOp(name="Test: REV Expansion Hub", group="FtcTestSamples")
@Disabled
public class FtcTestRevHub extends FtcOpMode
{
    private static final int NUM_MOTORS = 4;
    private static final int NUM_SERVOS = 6;
    private static final int NUM_ANALOG_INPUTS = 4;
    private static final int NUM_DIGITAL_INPUTS = 8;

    private FtcGamepad gamepad;
    private FtcBNO055Imu imu;
    private FtcColorSensor colorSensor;
    private FtcDcMotor[] motors = new FtcDcMotor[NUM_MOTORS];
    private FtcServo[] servos = new FtcServo[NUM_SERVOS];
    private FtcAnalogInput[] analogInputs = new FtcAnalogInput[NUM_ANALOG_INPUTS];
    private FtcDigitalInput[] digitalInputs = new FtcDigitalInput[NUM_DIGITAL_INPUTS];
    private boolean dpadRightPressed = false;

    @Override
    public void initRobot()
    {
        gamepad = new FtcGamepad("GamePad", gamepad1, this::buttonEvent);
        gamepad.setYInverted(true);

        imu = new FtcBNO055Imu("imu");
        colorSensor = new FtcColorSensor("colorSensor");

        for (int i = 0; i < motors.length; i++)
        {
            motors[i] = new FtcDcMotor("motor" + i);
        }

        for (int i = 0; i < servos.length; i++)
        {
            servos[i] = new FtcServo("servo" + i);
        }

        for (int i = 0; i < analogInputs.length; i++)
        {
            analogInputs[i] = new FtcAnalogInput("analog" + i);
        }

        for (int i = 0; i < digitalInputs.length; i++)
        {
            digitalInputs[i] = new FtcDigitalInput("digital" + i);
        }
    }   //initRobot

    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        imu.gyro.resetZIntegrator();
        imu.gyro.setEnabled(true);
    }   //startMode

    @Override
    public void stopMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        imu.gyro.setEnabled(false);
    }   //stopMode

    @Override
    public void runPeriodic(double elapsedTime)
    {
        //
        // Test I2C IMU.
        //
        telemetry.addData("Heading", "Heading: x=%6.1f,y=%6.1f,z=%6.1f",
                imu.gyro.getXHeading().value, imu.gyro.getYHeading().value, imu.gyro.getZHeading().value);
        telemetry.addData("TurnRate", "TurnRate: x=%6.1f,y=%6.1f,z=%6.1f",
                imu.gyro.getXRotationRate().value, imu.gyro.getYRotationRate().value, imu.gyro.getZRotationRate().value);
        telemetry.addData("ImuAngle", "Angle:x=%6.1f,y=%6.1f,z=%6.1f",
                imu.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle,
                imu.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle,
                imu.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
        telemetry.addData("Accel", "x=%6.1f,y=%6.1f,z=%6.1f",
                imu.accel.getXAcceleration().value, imu.accel.getYAcceleration().value, imu.accel.getZAcceleration().value);
        telemetry.addData("Vel", "Vel: x=%6.1f,y=%6.1f,z=%6.1f",
                imu.accel.getXVelocity().value, imu.accel.getYVelocity().value, imu.accel.getZVelocity().value);
        telemetry.addData("Dist", "x=%6.1f,y=%6.1f,z=%6.1f",
                imu.accel.getXDistance().value, imu.accel.getYDistance().value, imu.accel.getZDistance().value);
        //
        // Test I2C Color Sensor.
        //
        telemetry.addData("ColorRGB", "Color=%x,rgb=%f/%f/%f",
                colorSensor.getRawData(0, FtcColorSensor.DataType.COLOR_NUMBER).value.intValue(),
                colorSensor.getRawData(0, FtcColorSensor.DataType.RED).value,
                colorSensor.getRawData(0, FtcColorSensor.DataType.GREEN).value,
                colorSensor.getRawData(0, FtcColorSensor.DataType.BLUE).value);
        telemetry.addData("ColorHSV", "HSV=%f/%f/%f",
                colorSensor.getRawData(0, FtcColorSensor.DataType.HUE).value,
                colorSensor.getRawData(0, FtcColorSensor.DataType.SATURATION).value,
                colorSensor.getRawData(0, FtcColorSensor.DataType.VALUE).value);
        //
        // Test DC motors.
        //
        double motorPower = gamepad.getRightStickY(true);
        for (int i = 0; i < motors.length; i++)
        {
            motors[i].set(motorPower);
            telemetry.addData("Motor" + i, "power=%.1f, enc=%d", motorPower, motors[i].getPosition());
        }
        //
        // Test Servos.
        //
        double pos = dpadRightPressed? 1.0: 0.0;
        for (int i = 0; i < servos.length; i++)
        {
            servos[i].setPosition(pos);
        }
        telemetry.addData("Servo", "pos=%.0f", pos);
        //
        // Test Analog Inputs.
        //
        double data;
        for (int i = 0; i < analogInputs.length; i++)
        {
            data = analogInputs[i].getData(0).value;
            telemetry.addData("Analog" + i, "volt=%.2f", data);
        }
        //
        // Test Digital Inputs.
        //
        boolean state;
        for (int i = 0; i < digitalInputs.length; i++)
        {
            state = digitalInputs[i].isActive();
            telemetry.addData("Digital" + i, "state=%s", state);
        }

        telemetry.update();
    }   //runPeriodic

    private void buttonEvent(TrcGameController gameCtrl, int button, boolean pressed)
    {
        switch(button)
        {
            case FtcGamepad.GAMEPAD_DPAD_RIGHT:
                dpadRightPressed = pressed;
                break;
        }
    }   //buttonEvent

}   //class FtcTestRevHub
