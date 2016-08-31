/*
 * Titan Robotics Framework Library
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.net)
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

import ftclib.FtcMRI2cGyro;
import ftclib.FtcOpMode;
import hallib.HalDashboard;
import trclib.TrcSensor;

@Autonomous(name="Test: Modern Robotics I2C Gyro", group="Ftc3543Sample")
//@Disabled
public class FtcTestMRI2cGyro extends FtcOpMode
{
    private HalDashboard dashboard;
    private FtcMRI2cGyro gyro;

    //
    // Implements FtcOpMode abstract method.
    //

    @Override
    public void initRobot()
    {
        hardwareMap.logDevices();
        dashboard = getDashboard();
        gyro = new FtcMRI2cGyro("mrGyro");
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
        dashboard.displayPrintf(1, "FirmwareRev:\t\t\t\t%x", gyro.getFirmwareRevision());
        dashboard.displayPrintf(2, "ManufacturerCode:\t%x", gyro.getManufacturerCode());
        dashboard.displayPrintf(3, "IDCode:\t\t\t\t\t\t\t%x", gyro.getIdCode());
        TrcSensor.SensorData data = gyro.getHeading();
        //
        // The data may not be ready yet, check it!
        //
        if (data.value != null)
        {
            dashboard.displayPrintf(4, "Heading:\t\t\t%d", (Integer)gyro.getHeading().value);
            dashboard.displayPrintf(5, "IntegratedZ:\t%d", (Integer)gyro.getIntegratedZ().value);
            dashboard.displayPrintf(6, "RawXYZ:\t\t\t%d/%d/%d",
                                    (Integer)gyro.getRawX().value,
                                    (Integer)gyro.getRawY().value,
                                    (Integer)gyro.getRawZ().value);
            dashboard.displayPrintf(7, "ZOffset:\t\t\t%d", (Integer)gyro.getZOffset().value);
            dashboard.displayPrintf(8, "ZScaling:\t\t\t%d", (Integer)gyro.getZScaling().value);
        }
    }   //runPeriodic

}   //class FtcTestMRI2cGyro
