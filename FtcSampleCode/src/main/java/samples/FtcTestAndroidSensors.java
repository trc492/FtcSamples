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

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorManager;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.util.List;

import ftclib.FtcAndroidSensor;
import ftclib.FtcOpMode;
import hallib.HalDashboard;

@Autonomous(name="Test: Android Sensors", group="Ftc3543Sample")
//@Disabled
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
    // Implements FtcOpMode abstract method.
    //

    @Override
    public void initRobot()
    {
        hardwareMap.logDevices();
        dashboard = getDashboard();
        //
        // Enumerates all Android sensors.
        //
        Context context = FtcOpMode.getInstance().hardwareMap.appContext;
        SensorManager sensorManager =
                (SensorManager)context.getSystemService(Context.SENSOR_SERVICE);
        List<Sensor> sensorList = sensorManager.getSensorList(Sensor.TYPE_ALL);
        dashboard.displayPrintf(0, "Android Sensors:");
        int lineNum = 1;
        for (Sensor sensor: sensorList)
        {
            dashboard.displayPrintf(lineNum, "%02d->%s, %s, %d",
                                    sensor.getType(), sensor.getVendor(),
                                    sensor.getName(), sensor.getVersion());
            lineNum++;
            if (lineNum >= HalDashboard.MAX_NUM_TEXTLINES)
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
        linearAccel = FtcAndroidSensor.createInstance(
                "linearAccel", Sensor.TYPE_LINEAR_ACCELERATION, 3);
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
    public void startMode()
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
            orientation.setEnabled(true);
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
    public void stopMode()
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
            orientation.setEnabled(false);
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
        if (accel != null)
        {
            dashboard.displayPrintf(1, "Accel:x=%.2f,y=%.2f,z=%.2f (m/s2)",
                                    accel.getData(0, null).value,
                                    accel.getData(1, null).value,
                                    accel.getData(2, null).value);
        }
        else
        {
            dashboard.displayPrintf(1, "Accel:none.");
        }

        if (gravity != null)
        {
            dashboard.displayPrintf(2, "Gravity:x=%.2f,y=%.2f,z=%.2f (m/s2)",
                                    gravity.getData(0, null).value,
                                    gravity.getData(1, null).value,
                                    gravity.getData(2, null).value);
        }
        else
        {
            dashboard.displayPrintf(2, "Gravity:none.");
        }

        if (gyro != null)
        {
            dashboard.displayPrintf(3, "Gyro:x=%.2f,y=%.2f,z=%.2f (deg/s)",
                                    (Double)gyro.getData(0, null).value*180.0/Math.PI,
                                    (Double)gyro.getData(1, null).value*180.0/Math.PI,
                                    (Double)gyro.getData(2, null).value*180.0/Math.PI);
        }
        else
        {
            dashboard.displayPrintf(3, "Gyro:none.");
        }

        if (linearAccel != null)
        {
            dashboard.displayPrintf(4, "LinearAccel:x=%.2f,y=%.2f,z=%.2f (m/s2)",
                                    linearAccel.getData(0, null).value,
                                    linearAccel.getData(1, null).value,
                                    linearAccel.getData(2, null).value);
        }
        else
        {
            dashboard.displayPrintf(4, "LinearAccel:none.");
        }

        if (rotation != null)
        {
            dashboard.displayPrintf(5, "Rotation:x=%.2f,y=%.2f,z=%.2f,s=%.2f",
                                    rotation.getData(0, null).value,
                                    rotation.getData(1, null).value,
                                    rotation.getData(2, null).value,
                                    rotation.getData(3, null).value);
        }
        else
        {
            dashboard.displayPrintf(5, "Rotation:none.");
        }

        if (magnetic != null)
        {
            dashboard.displayPrintf(6, "Magnetic:x=%.2f,y=%.2f,z=%.2f (uT)",
                                    magnetic.getData(0, null).value,
                                    magnetic.getData(1, null).value,
                                    magnetic.getData(2, null).value);
        }
        else
        {
            dashboard.displayPrintf(6, "Magnetic:none.");
        }

        /*
        if (orientation != null)
        {
            dashboard.displayPrintf(7, "Orientation:Azimuth=%.2f,Pitch=%.2f,Roll=%.2f (deg)",
                                    orientation.getData(0, null).value,
                                    orientation.getData(1, null).value,
                                    orientation.getData(2, null).value);
        }
        else
        {
            dashboard.displayPrintf(7, "Orietation:none.");
        }
        */

        if (proximity != null)
        {
            dashboard.displayPrintf(8, "Proximity:%.0f cm", proximity.getData(0, null).value);
        }
        else
        {
            dashboard.displayPrintf(8, "Proximity:none.");
        }

        if (light != null)
        {
            dashboard.displayPrintf(9, "Light:%.0f lux", light.getData(0, null).value);
        }
        else
        {
            dashboard.displayPrintf(9, "Light:none.");
        }
    }   //runPeriodic

}   //class FtcTestAndroidSensors
