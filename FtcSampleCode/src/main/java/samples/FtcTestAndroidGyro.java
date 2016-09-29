/*
 * Copyright (c) 2016 Titan Robotics Club (http://www.titanrobotics.com)
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
import ftclib.FtcAndroidGyro;
import ftclib.FtcOpMode;
import hallib.HalDashboard;
import trclib.TrcGyro;

@TeleOp(name="Test: Android Gyro", group="Ftc3543Sample")
//@Disabled
public class FtcTestAndroidGyro extends FtcOpMode
{
    private HalDashboard dashboard;
    private FtcAndroidGyro gyro;

    //
    // Implements FtcOpMode abstract method.
    //

    @Override
    public void initRobot()
    {
        hardwareMap.logDevices();
        dashboard = getDashboard();
        FtcRobotControllerActivity activity = (FtcRobotControllerActivity)hardwareMap.appContext;
        dashboard.setTextView((TextView)activity.findViewById(R.id.textOpMode));

        gyro = new FtcAndroidGyro("AndroidGyro");
        gyro.calibrate();
        double scale = 180.0/Math.PI;
        gyro.setXScale(scale);
        gyro.setYScale(scale);
        gyro.setZScale(scale);
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void startMode()
    {
        dashboard.clearDisplay();
        gyro.setEnabled(true);
    }   //startMode

    @Override
    public void stopMode()
    {
        gyro.setEnabled(false);
    }   //stopMode

    @Override
    public void runPeriodic(double elapsedTime)
    {
        final int LABEL_WIDTH = 100;

        dashboard.displayPrintf(1, LABEL_WIDTH, "Raw: ", "x=%.2f,y=%.2f,z=%.2f (rad/s)",
                                gyro.getRawXData(TrcGyro.DataType.ROTATION_RATE).value,
                                gyro.getRawYData(TrcGyro.DataType.ROTATION_RATE).value,
                                gyro.getRawZData(TrcGyro.DataType.ROTATION_RATE).value);
        dashboard.displayPrintf(2, LABEL_WIDTH, "Rot: ", "x=%.2f,y=%.2f,z=%.2f (deg/s)",
                                gyro.getXRotationRate().value,
                                gyro.getYRotationRate().value,
                                gyro.getZRotationRate().value);
        dashboard.displayPrintf(3, LABEL_WIDTH, "Heading: ", "x=%.2f,y=%.2f,z=%.2f (deg)",
                                gyro.getXHeading().value,
                                gyro.getYHeading().value,
                                gyro.getZHeading().value);
    }   //runPeriodic

}   //class FtcTestAndroidGyro
