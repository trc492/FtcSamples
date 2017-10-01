/*
 * Copyright (c) 2017 Titan Robotics Club (http://www.titanrobotics.com)
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

import java.util.Locale;

import FtcSampleCode.R;
import ftclib.FtcOpMode;
import ftclib.FtcVuforia;
import hallib.HalDashboard;

@TeleOp(name="Test: VuMark Tracking", group="3543TestSamples")
//@Disabled
public class FtcTestVuMark extends FtcOpMode
{
    //
    // If you copy our code, please register your own account and generate your own free license key at this site:
    // https://developer.vuforia.com/license-manager
    //
    private final float MM_PER_INCH = 25.4f;
    private final String VUFORIA_LICENSE_KEY =
            "AdCwzDH/////AAAAGeDkDS3ukU9+lIXc19LMh+cKk29caNhOl8UqmZOymRGwVwT1ZN8uaPdE3Q+zceDu9AKNsqL9qLblSFV" +
            "/x8Y3jfOZdjMFs0CQSQOEyWv3xfJsdSmevXDQDQr+4KI31HY2YSf/KB/kyxfuRMk4Pi+vWS+oLl65o7sWPiyFgzoM74ENyb" +
            "j4FgteD/2b6B+UFuwkHWKBNpp18wrpkaiFfr/FCbRFcdWP5mrjlEZM6eOj171dybw97HPeZbGihnnxOeeUv075O7P167AVq" +
            "aiPy2eRK7OCubR32KXOqQKoyF6AXp+qu2cOTApXS5dqOOseEm+HE4eMF0S2Pld3i5AWBIR+JlPXDuc9LwoH2Q8iDwUK1+4g";
    private final VuforiaLocalizer.CameraDirection CAMERA_DIR = VuforiaLocalizer.CameraDirection.BACK;
    private final String TRACKABLES_FILE = "RelicVuMark";
    private final boolean SPEECH_ENABLED = true;

    private HalDashboard dashboard;
    private FtcVuforia vuforia;
    private TextToSpeech textToSpeech = null;
    private RelicRecoveryVuMark prevVuMark = null;

    private VectorF getVuMarkPosition()
    {
        VectorF targetPos = null;
        VuforiaTrackable target = vuforia.getTarget(0);
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(target);

        if (vuforia.isTargetVisible(target) && vuMark != RelicRecoveryVuMark.UNKNOWN)
        {
            OpenGLMatrix pose = vuforia.getTargetPose(target);
            if (pose != null)
            {
                targetPos = pose.getTranslation();
            }
        }

        return targetPos;
    }   //getVuMarkPosition

    private Orientation getVuMarkOrientation()
    {
        Orientation targetAngle = null;
        VuforiaTrackable target = vuforia.getTarget(0);
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(target);

        if (vuforia.isTargetVisible(target) && vuMark != RelicRecoveryVuMark.UNKNOWN)
        {
            OpenGLMatrix pose = vuforia.getTargetPose(target);
            if (pose != null)
            {
                targetAngle = Orientation.getOrientation(
                        pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            }
        }

        return targetAngle;
    }   //getVuMarkOrientation

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

        int cameraViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        vuforia = new FtcVuforia(VUFORIA_LICENSE_KEY, cameraViewId, CAMERA_DIR, TRACKABLES_FILE, 1);
        vuforia.setTargetInfo(0, "relicVuMarkTemplate");
        //
        // Text To Speech.
        //
        if (SPEECH_ENABLED)
        {
            textToSpeech = FtcOpMode.getInstance().getTextToSpeech();
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
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(vuforia.getTarget(0));

        if (vuMark != RelicRecoveryVuMark.UNKNOWN)
        {
            VectorF pos = getVuMarkPosition();
            Orientation orientation = getVuMarkOrientation();

            dashboard.displayPrintf(1, "%s: x=%6.2f,y=%6.2f,z=%6.2f",
                    vuMark.toString(), pos.get(0)/MM_PER_INCH, pos.get(1)/MM_PER_INCH, pos.get(2)/MM_PER_INCH);
            dashboard.displayPrintf(2, "%s: xRot=%6.2f,yRot=%6.2f,zRot=%6.2f",
                    vuMark.toString(), orientation.firstAngle, orientation.secondAngle, orientation.thirdAngle);
        }

        if (vuMark != prevVuMark)
        {
            String sentence = null;

            if (vuMark != RelicRecoveryVuMark.UNKNOWN)
            {
                sentence = String.format("%s is %s.", vuMark.toString(), "in view");
            }
            else if (prevVuMark != null)
            {
                sentence = String.format("%s is %s.", prevVuMark.toString(), "out of view");
            }

            if (sentence != null)
            {
                dashboard.displayPrintf(3, sentence);
                if (textToSpeech != null)
                {
                    //
                    // ZTE phones are on KitKat and running level 19 APIs, so we need to use the deprecated version
                    // to be compatible with it.
                    //
                    textToSpeech.speak(sentence, TextToSpeech.QUEUE_FLUSH, null);
                }
            }

            prevVuMark = vuMark;
        }
    }   //runPeriodic

}   //class FtcTestVuMark
