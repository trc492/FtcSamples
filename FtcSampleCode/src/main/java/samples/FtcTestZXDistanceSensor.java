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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import FtcSampleCode.R;
import ftclib.FtcOpMode;
import ftclib.FtcZXDistanceSensor;
import hallib.HalDashboard;
import trclib.TrcSensor;

@Autonomous(name="Test: ZX Distance Sensor", group="Ftc3543Sample")
//@Disabled
public class FtcTestZXDistanceSensor extends FtcOpMode
{
    private HalDashboard dashboard;
    private FtcZXDistanceSensor sensor;

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
        sensor = new FtcZXDistanceSensor("zxSensor", FtcZXDistanceSensor.ALTERNATE_I2CADDRESS);
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void startMode()
    {
        dashboard.clearDisplay();
    }   //startMode

    @Override
    public void runPeriodic(double elapsedTime)
    {
        final int LABEL_WIDTH = 200;
        TrcSensor.SensorData data;

        dashboard.displayPrintf(1, LABEL_WIDTH, "Model: ", "%d", sensor.getModelVersion());
        dashboard.displayPrintf(2, LABEL_WIDTH, "RegVersion: ", "%d", sensor.getRegMapVersion());
        dashboard.displayPrintf(3, LABEL_WIDTH, "Status: ", "%02x", sensor.getStatus());

        //
        // The data may not be ready yet, check it!
        //

        data = sensor.getGesture();
        if (data.value != null)
        {
            dashboard.displayPrintf(4, LABEL_WIDTH, "Gesture: ", "%s", data.value.toString());
        }

        data = sensor.getGestureSpeed();
        if (data.value != null)
        {
            dashboard.displayPrintf(5, LABEL_WIDTH, "GestureSpeed: ", "%d", data.value);
        }

        data = sensor.getX();
        if (data.value != null)
        {
            dashboard.displayPrintf(6, LABEL_WIDTH, "X: ", "%d", data.value);
        }

        data = sensor.getZ();
        if (data.value != null)
        {
            dashboard.displayPrintf(7, LABEL_WIDTH, "Z: ", "%d", data.value);
        }

        data = sensor.getLeftRangingData();
        if (data.value != null)
        {
            dashboard.displayPrintf(8, LABEL_WIDTH, "LRng: ", "%d", data.value);
        }

        data = sensor.getRightRangingData();
        if (data.value != null)
        {
            dashboard.displayPrintf(9, LABEL_WIDTH, "RRng: ", "%d", data.value);
        }
    }   //runPeriodic

}   //class FtcTestAndroidSensors
