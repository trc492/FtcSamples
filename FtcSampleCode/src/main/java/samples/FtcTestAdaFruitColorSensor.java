/*
 * Titan Robotics Framework Library
 * Copyright (c) 2016 Titan Robotics Club (http://www.titanrobotics.net)
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import ftclib.FtcAdaFruitColorSensor;
import ftclib.FtcOpMode;
import hallib.HalDashboard;
import trclib.TrcSensor;

@Autonomous(name="Test: AdaFruit Color Sensor", group="Ftc3543Sample")
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
        TrcSensor.SensorData data;

        dashboard.displayPrintf(1, "ID:\t\t%d", sensor.getID());
        dashboard.displayPrintf(2, "Status:\t%02x", sensor.getStatus());

        //
        // The data may not be ready yet, check it!
        //

        data = sensor.getClearValue();
        if (data.value != null)
        {
            dashboard.displayPrintf(3, "Clear:\t%d", data.value);
        }

        data = sensor.getRedValue();
        if (data.value != null)
        {
            dashboard.displayPrintf(4, "Red:\t\t%d", data.value);
        }

        data = sensor.getGreenValue();
        if (data.value != null)
        {
            dashboard.displayPrintf(5, "Green:\t%d", data.value);
        }

        data = sensor.getBlueValue();
        if (data.value != null)
        {
            dashboard.displayPrintf(6, "Blue:\t\t%d", data.value);
        }
    }   //runPeriodic

}   //class FtcTestAdaFruitColorSensor
