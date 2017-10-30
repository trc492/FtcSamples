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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import ftclib.FtcBNO055Imu;
import ftclib.FtcColorSensor;
import ftclib.FtcDigitalInput;
import ftclib.FtcDistanceSensor;
import ftclib.FtcOpMode;
import hallib.HalDashboard;

@TeleOp(name="Test: REV Expansion Hub", group="3543TestSamples")
//@Disabled
public class FtcTestRevHub extends FtcOpMode
{
    private HalDashboard dashboard;
    private FtcBNO055Imu imu;
    private FtcDigitalInput touchSensor;
    private FtcColorSensor colorSensor;
    private FtcDistanceSensor rangeSensor;

    //
    // Implements FtcOpMode abstract methods.
    //

    @Override
    public void initRobot()
    {
        //
        // Initializing sensors on or connected to the REV hub.
        //
        dashboard = HalDashboard.getInstance();
        imu = new FtcBNO055Imu("imu");
        touchSensor = new FtcDigitalInput("touchSensor");
        colorSensor = new FtcColorSensor("colorRangeSensor");
        rangeSensor = new FtcDistanceSensor("colorRangeSensor");
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void startMode()
    {
        dashboard.clearDisplay();
        imu.gyro.setEnabled(true);
    }   //startMode

    @Override
    public void stopMode()
    {
        imu.gyro.setEnabled(false);
    }   //stopMode

    @Override
    public void runPeriodic(double elapsedTime)
    {
        dashboard.displayPrintf(1, "Angle:x=%6.1f,y=%6.1f,z=%6.1f",
                imu.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle,
                imu.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle,
                imu.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
        dashboard.displayPrintf(2, "Heading: x=%6.1f,y=%6.1f,z=%6.1f",
                imu.gyro.getXHeading().value,
                imu.gyro.getYHeading().value,
                imu.gyro.getZHeading().value);
        dashboard.displayPrintf(3, "TurnRate: x=%6.1f,y=%6.1f,z=%6.1f",
                imu.gyro.getXRotationRate().value,
                imu.gyro.getYRotationRate().value,
                imu.gyro.getZRotationRate().value);
        dashboard.displayPrintf(4, "Accel: x=%6.1f,y=%6.1f,z=%6.1f",
                imu.accel.getXAcceleration().value,
                imu.accel.getYAcceleration().value,
                imu.accel.getZAcceleration().value);
        dashboard.displayPrintf(5, "Vel: x=%6.1f,y=%6.1f,z=%6.1f",
                imu.accel.getXVelocity().value,
                imu.accel.getYVelocity().value,
                imu.accel.getZVelocity().value);
        dashboard.displayPrintf(6, "Dist: x=%6.1f,y=%6.1f,z=%6.1f",
                imu.accel.getXDistance().value,
                imu.accel.getYDistance().value,
                imu.accel.getZDistance().value);
        dashboard.displayPrintf(7, "Touch=%s", touchSensor.isActive());
        dashboard.displayPrintf(8, "Color=%x,rgb=%d/%d/%d",
                colorSensor.getRawData(0, FtcColorSensor.DataType.COLOR_NUMBER).value,
                colorSensor.getRawData(0, FtcColorSensor.DataType.RED).value,
                colorSensor.getRawData(0, FtcColorSensor.DataType.GREEN).value,
                colorSensor.getRawData(0, FtcColorSensor.DataType.BLUE).value);
        dashboard.displayPrintf(9, "Range=%f",
                rangeSensor.getRawData(0, FtcDistanceSensor.DataType.DISTANCE_INCH).value);
    }   //runPeriodic

}   //class FtcTestRevHub
