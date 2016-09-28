/*
 * Copyright (c) 2016 Titan Robotics Club (http://www.titanrobotics.com)
 * Based on sample code by Robert Atkinson.
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

import android.speech.tts.TextToSpeech;
import android.widget.TextView;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

import java.util.Locale;

import FtcSampleCode.R;
import ftclib.FtcOpMode;
import ftclib.FtcVuforia;
import hallib.HalDashboard;

@Autonomous(name="Test: Vuforia Targets Tracking", group="Ftc3543Sample")
//@Disabled
public class FtcTestVuforia extends FtcOpMode
{
    private final boolean trackRobotLocation = true;
    private final float MM_PER_INCH = 25.4f;
    private final float ROBOT_WIDTH = 18*MM_PER_INCH;               // in mm
    private final float FTC_FIELD_WIDTH = (12*12 - 2)*MM_PER_INCH;  // in mm
    private final float TARGET_HEIGHT = 160.0f;                     // in mm
    //
    // If you copy our code, please register your own account and generate your own free license key at this site:
    // https://developer.vuforia.com/license-manager
    //
    private final String VUFORIA_LICENSE_KEY =
            "AdCwzDH/////AAAAGeDkDS3ukU9+lIXc19LMh+cKk29caNhOl8UqmZOymRGwVwT1ZN8uaPdE3Q+zceDu9AKNsqL9qLblSFV" +
            "/x8Y3jfOZdjMFs0CQSQOEyWv3xfJsdSmevXDQDQr+4KI31HY2YSf/KB/kyxfuRMk4Pi+vWS+oLl65o7sWPiyFgzoM74ENyb" +
            "j4FgteD/2b6B+UFuwkHWKBNpp18wrpkaiFfr/FCbRFcdWP5mrjlEZM6eOj171dybw97HPeZbGihnnxOeeUv075O7P167AVq" +
            "aiPy2eRK7OCubR32KXOqQKoyF6AXp+qu2cOTApXS5dqOOseEm+HE4eMF0S2Pld3i5AWBIR+JlPXDuc9LwoH2Q8iDwUK1+4g";
    private final int CAMERAVIEW_ID = R.id.cameraMonitorViewId;
    private final VuforiaLocalizer.CameraDirection CAMERA_DIR = VuforiaLocalizer.CameraDirection.BACK;
    private final String TRACKABLES_FILE = "FTC_2016-17";
    //
    // Note that the order of the targets must match the order in the FTC_2016-17.xml file.
    //
    private FtcVuforia.Target[] targets =
    {
            //
            // Blue alliance beacon 1.
            //
            new FtcVuforia.Target("wheels", 90.0f, 0.0f, 0.0f, 12.0f*MM_PER_INCH, FTC_FIELD_WIDTH/2.0f, TARGET_HEIGHT),
            //
            // Red alliance beacon 2.
            //
            new FtcVuforia.Target("tools", 90.0f, 0.0f, 90.0f, -FTC_FIELD_WIDTH/2.0f, 30.0f*MM_PER_INCH, TARGET_HEIGHT),
            //
            // Blue alliance beacon 2 location.
            //
            new FtcVuforia.Target("legos", 90.0f, 0.0f, 0.0f, -30.0f*MM_PER_INCH, FTC_FIELD_WIDTH/2.0f, TARGET_HEIGHT),
            //
            // Red alliance beacon 1 location.
            //
            new FtcVuforia.Target("gears", 90.0f, 0.0f, 90.0f, -FTC_FIELD_WIDTH/2.0f, -12.0f*MM_PER_INCH, TARGET_HEIGHT)
    };
    private final boolean speechEnabled = true;

    private HalDashboard dashboard;
    private FtcVuforia vuforia;
    private OpenGLMatrix lastKnownRobotLocation = null;
    private TextToSpeech textToSpeech = null;
    private boolean[] targetsFound = null;

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

        vuforia = new FtcVuforia(VUFORIA_LICENSE_KEY, CAMERAVIEW_ID, CAMERA_DIR, TRACKABLES_FILE, targets.length);
        //
        // Phone location: Mounted center on the front of the robot with the back camera facing outward.
        //
        OpenGLMatrix phoneLocationOnRobot =
                trackRobotLocation? vuforia.locationMatrix(90.0f, 0.0f, 0.0f, 0.0f, ROBOT_WIDTH/2.0f, 0.0f): null;

        vuforia.setTargets(targets, phoneLocationOnRobot);
        //
        // Text To Speech.
        //
        if (speechEnabled)
        {
            textToSpeech = new TextToSpeech(
                    hardwareMap.appContext,
                    new TextToSpeech.OnInitListener()
                    {
                        @Override
                        public void onInit(int status)
                        {
                            if (status != TextToSpeech.ERROR)
                            {
                                textToSpeech.setLanguage(Locale.US);
                            }
                        }
                    });
            targetsFound = new boolean[targets.length];
            for (int i = 0; i < targetsFound.length; i++)
            {
                targetsFound[i] = false;
            }
        }
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void startMode()
    {
        dashboard.clearDisplay();
        vuforia.setTrackingEnabled(true);
    }   //startMode

    @Override
    public void stopMode()
    {
        vuforia.setTrackingEnabled(false);
        if (textToSpeech != null)
        {
            textToSpeech.stop();
            textToSpeech.shutdown();
        }
    }   //stopMode

    @Override
    public void runPeriodic(double elapsedTime)
    {
        final int LABEL_WIDTH = 100;

        for (int i = 0; i < targets.length; i++)
        {
            VuforiaTrackable target = vuforia.getTarget(i);
            boolean visible = vuforia.isTargetVisible(target);
            if (speechEnabled)
            {
                if (visible != targetsFound[i])
                {
                    targetsFound[i] = visible;
                    String sentence = String.format(
                            "%s is %s.", target.getName(), visible ? "in view" : "out of view");
                    textToSpeech.speak(sentence, TextToSpeech.QUEUE_FLUSH, null);
                }
            }
            dashboard.displayPrintf(
                    i*2 + 1, LABEL_WIDTH, target.getName() + ": ", "%s", visible? "Visible": "NotVisiable");

            OpenGLMatrix pose = vuforia.getTargetPose(target);
            if (pose != null)
            {
                VectorF translation = pose.getTranslation();
                dashboard.displayPrintf(
                        i*2 + 2, LABEL_WIDTH, "x,y,z in. = ", "%6.2f,%6.2f,%6.2f",
                        translation.get(0)/MM_PER_INCH,
                        translation.get(1)/MM_PER_INCH,
                        -translation.get(2)/MM_PER_INCH);
            }

            OpenGLMatrix robotLocation = vuforia.getRobotLocation(target);
            if (robotLocation != null)
            {
                lastKnownRobotLocation = robotLocation;
            }
        }

        dashboard.displayPrintf(9, "Robot Location");
        if (lastKnownRobotLocation != null)
        {
            dashboard.displayPrintf(10, lastKnownRobotLocation.formatAsTransform());
        }
    }   //runPeriodic

}   //class FtcTestVuforia
