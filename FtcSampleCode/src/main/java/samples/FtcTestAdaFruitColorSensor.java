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
import ftclib.FtcAdaFruitColorSensor;
import ftclib.FtcOpMode;
import hallib.HalDashboard;
import trclib.TrcSensor;

@TeleOp(name="Test: AdaFruit Color Sensor", group="3543TestSamples")
//@Disabled
public class FtcTestAdaFruitColorSensor extends FtcOpMode
{
    private HalDashboard dashboard;
    private FtcAdaFruitColorSensor sensor;

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
        sensor = new FtcAdaFruitColorSensor("adaFruitColorSensor");
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
        final int LABEL_WIDTH = 100;
        TrcSensor.SensorData data;

        dashboard.displayPrintf(1, LABEL_WIDTH, "ID: ", "%d", sensor.getID());
        dashboard.displayPrintf(2, LABEL_WIDTH, "Status: ", "%02x", sensor.getStatus());

        //
        // The data may not be ready yet, check it!
        //

        data = sensor.getClearValue();
        if (data.value != null)
        {
            dashboard.displayPrintf(3, LABEL_WIDTH, "Clear: ", "%d", data.value);
        }

        data = sensor.getRedValue();
        if (data.value != null)
        {
            dashboard.displayPrintf(4, LABEL_WIDTH, "Red: ", "%d", data.value);
        }

        data = sensor.getGreenValue();
        if (data.value != null)
        {
            dashboard.displayPrintf(5, LABEL_WIDTH, "Green: ", "%d", data.value);
        }

        data = sensor.getBlueValue();
        if (data.value != null)
        {
            dashboard.displayPrintf(6, LABEL_WIDTH, "Blue: ", "%d", data.value);
        }
    }   //runPeriodic

}   //class FtcTestAdaFruitColorSensor
