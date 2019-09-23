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

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorManager;
import android.widget.TextView;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import java.util.List;

import FtcSampleCode.R;
import ftclib.FtcAndroidSensor;
import ftclib.FtcOpMode;
import hallib.HalDashboard;
import trclib.TrcRobot;

@TeleOp(name="Test: Android Sensors", group="FtcTestSamples")
@Disabled
public class FtcTestAndroidSensors extends FtcOpMode
{
    private HalDashboard dashboard;
    private FtcAndroidSensor accel;
    private FtcAndroidSensor gravity;
    private FtcAndroidSensor gyro;
    private FtcAndroidSensor linearAccel;
    private FtcAndroidSensor rotation;
    private FtcAndroidSensor magnetic;
//    private FtcAndroidSensor orientation;
    private FtcAndroidSensor proximity;
    private FtcAndroidSensor light;

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
        // Enumerates all Android sensors.
        //
        SensorManager sensorManager = (SensorManager)activity.getSystemService(Context.SENSOR_SERVICE);
        List<Sensor> sensorList = sensorManager.getSensorList(Sensor.TYPE_ALL);
        dashboard.displayPrintf(0, "Android Sensors:");
        int lineNum = 1;
        for (Sensor sensor: sensorList)
        {
            dashboard.displayPrintf(lineNum, "%02d->%s, %s, %d",
                                    sensor.getType(), sensor.getVendor(), sensor.getName(), sensor.getVersion());
            lineNum++;
            if (lineNum >= dashboard.getNumTextLines())
            {
                break;
            }
        }

        //
        // Create Android sensors.
        //
        accel = FtcAndroidSensor.createInstance("accel", Sensor.TYPE_ACCELEROMETER, 3);
        gravity = FtcAndroidSensor.createInstance("gravity", Sensor.TYPE_GRAVITY, 3);
        gyro = FtcAndroidSensor.createInstance("gyro", Sensor.TYPE_GYROSCOPE, 3);
        linearAccel = FtcAndroidSensor.createInstance("linearAccel", Sensor.TYPE_LINEAR_ACCELERATION, 3);
        rotation = FtcAndroidSensor.createInstance("rotation", Sensor.TYPE_ROTATION_VECTOR, 4);
        magnetic = FtcAndroidSensor.createInstance("magnetic", Sensor.TYPE_MAGNETIC_FIELD, 3);
//        orientation = FtcAndroidSensor.createInstance("orientation", Sensor.TYPE_ORIENTATION, 3);
        proximity = FtcAndroidSensor.createInstance("proximity", Sensor.TYPE_PROXIMITY, 1);
        light = FtcAndroidSensor.createInstance("light", Sensor.TYPE_LIGHT, 1);
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        dashboard.clearDisplay();

        if (accel != null)
        {
            accel.setEnabled(true);
        }

        if (gravity != null)
        {
            gravity.setEnabled(true);
        }

        if (gyro != null)
        {
            gyro.setEnabled(true);
        }

        if (linearAccel != null)
        {
            linearAccel.setEnabled(true);
        }

        if (rotation != null)
        {
            rotation.setEnabled(true);
        }

        if (magnetic != null)
        {
            magnetic.setEnabled(true);
        }

        /*
        if (orientation != null)
        {
            orientation.setTaskEnabled(true);
        }
        */

        if (proximity != null)
        {
            proximity.setEnabled(true);
        }

        if (light != null)
        {
            light.setEnabled(true);
        }
    }   //startMode

    @Override
    public void stopMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        if (accel != null)
        {
            accel.setEnabled(false);
        }

        if (gravity != null)
        {
            gravity.setEnabled(false);
        }

        if (gyro != null)
        {
            gyro.setEnabled(false);
        }

        if (linearAccel != null)
        {
            linearAccel.setEnabled(false);
        }

        if (rotation != null)
        {
            rotation.setEnabled(false);
        }

        if (magnetic != null)
        {
            magnetic.setEnabled(false);
        }

        /*
        if (orientation != null)
        {
            orientation.setTaskEnabled(false);
        }
        */

        if (proximity != null)
        {
            proximity.setEnabled(false);
        }

        if (light != null)
        {
            light.setEnabled(false);
        }
    }   //stopMode

    @Override
    public void runPeriodic(double elapsedTime)
    {
        final int LABEL_WIDTH = 160;
        if (accel != null)
        {
            dashboard.displayPrintf(1, LABEL_WIDTH, "Accel: ", "x=%.2f,y=%.2f,z=%.2f (m/s2)",
                                    accel.getProcessedData(0, null).value,
                                    accel.getProcessedData(1, null).value,
                                    accel.getProcessedData(2, null).value);
        }
        else
        {
            dashboard.displayPrintf(1, LABEL_WIDTH, "Accel: ", "none.");
        }

        if (gravity != null)
        {
            dashboard.displayPrintf(2, LABEL_WIDTH, "Gravity: ", "x=%.2f,y=%.2f,z=%.2f (m/s2)",
                                    gravity.getProcessedData(0, null).value,
                                    gravity.getProcessedData(1, null).value,
                                    gravity.getProcessedData(2, null).value);
        }
        else
        {
            dashboard.displayPrintf(2, LABEL_WIDTH, "Gravity: ", "none.");
        }

        if (gyro != null)
        {
            dashboard.displayPrintf(3, LABEL_WIDTH, "Gyro: ", "x=%.2f,y=%.2f,z=%.2f (deg/s)",
                                    (Double)gyro.getProcessedData(0, null).value * 180.0 / Math.PI,
                                    (Double)gyro.getProcessedData(1, null).value * 180.0 / Math.PI,
                                    (Double)gyro.getProcessedData(2, null).value * 180.0 / Math.PI);
        }
        else
        {
            dashboard.displayPrintf(3, LABEL_WIDTH, "Gyro: ", "none.");
        }

        if (linearAccel != null)
        {
            dashboard.displayPrintf(4, LABEL_WIDTH, "LinearAccel: ", "x=%.2f,y=%.2f,z=%.2f (m/s2)",
                                    linearAccel.getProcessedData(0, null).value,
                                    linearAccel.getProcessedData(1, null).value,
                                    linearAccel.getProcessedData(2, null).value);
        }
        else
        {
            dashboard.displayPrintf(4, LABEL_WIDTH, "LinearAccel: ", "none.");
        }

        if (rotation != null)
        {
            dashboard.displayPrintf(5, LABEL_WIDTH, "Rotation: ", "x=%.2f,y=%.2f,z=%.2f,s=%.2f",
                                    rotation.getProcessedData(0, null).value,
                                    rotation.getProcessedData(1, null).value,
                                    rotation.getProcessedData(2, null).value,
                                    rotation.getProcessedData(3, null).value);
        }
        else
        {
            dashboard.displayPrintf(5, LABEL_WIDTH, "Rotation: ", "none.");
        }

        if (magnetic != null)
        {
            dashboard.displayPrintf(6, LABEL_WIDTH, "Magnetic: ", "x=%.2f,y=%.2f,z=%.2f (uT)",
                                    magnetic.getProcessedData(0, null).value,
                                    magnetic.getProcessedData(1, null).value,
                                    magnetic.getProcessedData(2, null).value);
        }
        else
        {
            dashboard.displayPrintf(6, LABEL_WIDTH, "Magnetic: ", "none.");
        }

        /*
        if (orientation != null)
        {
            dashboard.displayPrintf(7, LABEL_WIDTH, "Orientation: ", "Azimuth=%.2f,Pitch=%.2f,Roll=%.2f (deg)",
                                    orientation.getProcessedData(0, null).value,
                                    orientation.getProcessedData(1, null).value,
                                    orientation.getProcessedData(2, null).value);
        }
        else
        {
            dashboard.displayPrintf(7, LABEL_WIDTH, "Orietation: ", "none.");
        }
        */

        if (proximity != null)
        {
            dashboard.displayPrintf(8, LABEL_WIDTH, "Proximity: ", "%.0f cm", proximity.getProcessedData(0, null).value);
        }
        else
        {
            dashboard.displayPrintf(8, LABEL_WIDTH, "Proximity: ", "none.");
        }

        if (light != null)
        {
            dashboard.displayPrintf(9, LABEL_WIDTH, "Light: ", "%.0f lux", light.getProcessedData(0, null).value);
        }
        else
        {
            dashboard.displayPrintf(9, LABEL_WIDTH, "Light: ", "none.");
        }
    }   //runPeriodic

}   //class FtcTestAndroidSensors
