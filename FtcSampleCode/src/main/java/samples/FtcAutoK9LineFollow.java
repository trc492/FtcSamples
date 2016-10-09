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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.LightSensor;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import FtcSampleCode.R;
import ftclib.FtcDcMotor;
import ftclib.FtcOpMode;
import hallib.HalDashboard;
import trclib.TrcDriveBase;

@Autonomous(name="Auto: K9Bot Line Following", group="3543AutoSamples")
//@Disabled
public class FtcAutoK9LineFollow extends FtcOpMode
{
    private static final double MOTOR_POWER     = 0.15;
    private static final double LIGHT_THRESHOLD = 0.5;

    private HalDashboard dashboard;
    private LightSensor reflectedLight;

    private FtcDcMotor motorLeft;
    private FtcDcMotor motorRight;
    private TrcDriveBase driveBase;

    //
    // Implements FtcOpMode abstract methods.
    //

    @Override
    public void initRobot()
    {
        hardwareMap.logDevices();
        dashboard = getDashboard();
        FtcRobotControllerActivity activity = (FtcRobotControllerActivity)hardwareMap.appContext;
        dashboard.setTextView((TextView)activity.findViewById(R.id.textOpMode));
        //
        // Sensors.
        //
        reflectedLight = hardwareMap.lightSensor.get("light_sensor");
        //
        // DriveBase subsystem.
        //
        motorLeft = new FtcDcMotor("motor_1");
        motorRight = new FtcDcMotor("motor_2");
        motorLeft.setInverted(true);
        driveBase = new TrcDriveBase(motorLeft, motorRight);
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void startMode()
    {
        dashboard.clearDisplay();
        reflectedLight.enableLed(true);
        driveBase.resetPosition();
    }   //startMode

    @Override
    public void stopMode()
    {
        reflectedLight.enableLed(false);
    }   //stopMode

    @Override
    public void runContinuous(double elapsedTime)
    {
        final int LABEL_WIDTH = 200;
        double left = 0.0;
        double right = 0.0;
        double reflection = reflectedLight.getLightDetected();
        //
        // Following the left edge of a white line.
        //
        if (reflection < LIGHT_THRESHOLD)
        {
            //
            // We see the floor, turn right back to the line edge.
            //
            left = MOTOR_POWER;
            right = 0.0;
        }
        else
        {
            //
            // We see the line, turn left back to the line edge.
            //
            left = 0.0;
            right = MOTOR_POWER;
        }
        driveBase.tankDrive(left, right);

        dashboard.displayPrintf(1, LABEL_WIDTH, "Text: ", "*** Robot Data ***");
        dashboard.displayPrintf(2, LABEL_WIDTH, "reflection: ", "%.2f", reflection);
        dashboard.displayPrintf(3, LABEL_WIDTH, "left power: ", "%.2f", left);
        dashboard.displayPrintf(4, LABEL_WIDTH, "right power: ", "%.2f", right);
    }   //runContinuous

}   //class FtcAutoK9LineFollow
