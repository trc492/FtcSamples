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

package samples;

import android.widget.TextView;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import FtcSampleCode.R;
import ftclib.FtcDcMotor;
import ftclib.FtcGamepad;
import ftclib.FtcMRGyro;
import ftclib.FtcOpMode;
import hallib.HalDashboard;
import trclib.TrcDriveBase;
import trclib.TrcGameController;

@TeleOp(name="TeleOp: Mecanum Drive", group="3543TeleOpSamples")
@Disabled
public class FtcTeleOpMecanumDrive extends FtcOpMode implements TrcGameController.ButtonHandler
{
    private HalDashboard dashboard;
    private FtcMRGyro gyro;
    private FtcGamepad gamepad;
    private FtcDcMotor motorLeftFront;
    private FtcDcMotor motorRightFront;
    private FtcDcMotor motorLeftRear;
    private FtcDcMotor motorRightRear;
    private TrcDriveBase driveBase;
    private boolean fixedOnTarget = false;

    //
    // Implements FtcOpMode abstract methods.
    //

    @Override
    public void initRobot()
    {
        hardwareMap.logDevices();
        dashboard = HalDashboard.getInstance();
        FtcRobotControllerActivity activity = (FtcRobotControllerActivity)hardwareMap.appContext;
        dashboard.setTextView((TextView)activity.findViewById(R.id.textOpMode));
        //
        // Initializing sensors.
        //
        gyro = new FtcMRGyro("gyro_sensor");
        gyro.calibrate();
        //
        // Initializing gamepads.
        //
        gamepad = new FtcGamepad("Gamepad", gamepad1, this);
        gamepad.setYInverted(true);
        //
        // DriveBase subsystem.
        //
        motorLeftFront = new FtcDcMotor("motor_1");
        motorRightFront = new FtcDcMotor("motor_2");
        motorLeftRear = new FtcDcMotor("motor_3");
        motorRightRear = new FtcDcMotor("motor_4");
        motorLeftFront.setInverted(true);
        motorLeftRear.setInverted(true);
        driveBase = new TrcDriveBase(
                motorLeftFront, motorLeftRear, motorRightFront, motorRightRear, gyro);
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void startMode()
    {
        dashboard.clearDisplay();
        gyro.setEnabled(true);
        driveBase.resetPosition();
    }   //startMode

    @Override
    public void stopMode()
    {
        gyro.setEnabled(false);
    }   //stopMode

    @Override
    public void runPeriodic(double elapsedTime)
    {
        final int LABEL_WIDTH = 200;
        //
        // DriveBase subsystem.
        //
        double x = gamepad.getLeftStickX(true);
        double y = gamepad.getRightStickY(true);
        double rotation = gamepad.getRightTrigger(true);
        if (rotation == 0.0)
        {
            rotation = -gamepad.getLeftTrigger(true);
        }
        driveBase.mecanumDrive_Cartesian(x, y, rotation, false,
                                         fixedOnTarget? (Double)gyro.getZHeading().value: 0.0);

        dashboard.displayPrintf(1, LABEL_WIDTH, "Text: ", "*** Robot Data ***");
        dashboard.displayPrintf(2, LABEL_WIDTH, "x: ", "%.2f", x);
        dashboard.displayPrintf(3, LABEL_WIDTH, "y: ", "%.2f", y);
        dashboard.displayPrintf(4, LABEL_WIDTH, "rotation: ", "%.2f", rotation);
    }   //runPeriodic

    //
    // Implements TrcGameController.ButtonHandler interface.
    //

    @Override
    public void buttonEvent(TrcGameController gamepad, int button, boolean pressed)
    {
        if (gamepad == this.gamepad)
        {
            switch (button)
            {
                case FtcGamepad.GAMEPAD_A:
                    break;

                case FtcGamepad.GAMEPAD_Y:
                    break;

                case FtcGamepad.GAMEPAD_X:
                    break;

                case FtcGamepad.GAMEPAD_B:
                    break;

                case FtcGamepad.GAMEPAD_RBUMPER:
                    fixedOnTarget = pressed;
                    break;
            }
        }
    }   //buttonEvent

}   //class FtcTeleOpMecanumDrive
