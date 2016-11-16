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

import android.hardware.SensorManager;
import android.widget.TextView;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import FtcSampleCode.R;
import ftclib.FtcAndroidAccel;
import ftclib.FtcOpMode;
import hallib.HalDashboard;
import trclib.TrcAccelerometer;
import trclib.TrcIIRFilter;

@TeleOp(name="Test: Android Accelerometer", group="3543TestSamples")
@Disabled
public class FtcTestAndroidAccel extends FtcOpMode
{
    private HalDashboard dashboard;
    private FtcAndroidAccel accel;

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
        // Create 3 filters, one for each axis.
        // Accelerometer is very sensitive, so the data is very noisy.
        //
        TrcIIRFilter[] filters = new TrcIIRFilter[3];
        for (int i = 0; i < 3; i++)
        {
            filters[i] = new TrcIIRFilter("AndroidAccel" + i, 0.8);
        }

        accel = new FtcAndroidAccel("AndroidAccel");
        accel.calibrate();
        //
        // Scale the data from meters to inches.
        // 1 meter = 39.3701 inches
        //
        accel.setXScale(39.3701);
        accel.setYScale(39.3701);
        accel.setZScale(39.3701);
        accel.setSamplingPeriod(SensorManager.SENSOR_DELAY_FASTEST);
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void startMode()
    {
        dashboard.clearDisplay();
        accel.setEnabled(true);
    }   //startMode

    @Override
    public void stopMode()
    {
        accel.setEnabled(false);
    }   //stopMode

    @Override
    public void runPeriodic(double elapsedTime)
    {
        final int LABEL_WIDTH = 100;
        dashboard.displayPrintf(1, LABEL_WIDTH, "Raw: ", "x=%.2f,y=%.2f,z=%.2f",
                                accel.getRawXData(TrcAccelerometer.DataType.ACCELERATION).value,
                                accel.getRawYData(TrcAccelerometer.DataType.ACCELERATION).value,
                                accel.getRawZData(TrcAccelerometer.DataType.ACCELERATION).value);
        dashboard.displayPrintf(2, LABEL_WIDTH, "Accel: ", "x=%.2f,y=%.2f,z=%.2f (in/s2)",
                                accel.getXAcceleration().value,
                                accel.getYAcceleration().value,
                                accel.getZAcceleration().value);
        dashboard.displayPrintf(3, LABEL_WIDTH, "Vel: ", "x=%.2f,y=%.2f,z=%.2f (in/s)",
                                accel.getXVelocity().value,
                                accel.getYVelocity().value,
                                accel.getZVelocity().value);
        dashboard.displayPrintf(4, LABEL_WIDTH, "Dist: ", "x=%.2f,y=%.2f,z=%.2f (in)",
                                accel.getXDistance().value,
                                accel.getYDistance().value,
                                accel.getZDistance().value);
    }   //runPeriodic

}   //class FtcTestAndroidAccel
