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

import android.speech.tts.TextToSpeech;
import android.widget.TextView;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

import java.io.InputStream;
import java.util.Locale;

import FtcSampleCode.R;
import ftclib.FtcAnalogOutTone;
import ftclib.FtcAndroidTone;
import ftclib.FtcDcMotor;
import ftclib.FtcGamepad;
import ftclib.FtcMRGyro;
import ftclib.FtcOpMode;
import ftclib.FtcSongXml;
import ftclib.FtcVuforia;
import hallib.HalDashboard;
import trclib.TrcBooleanState;
import trclib.TrcDriveBase;
import trclib.TrcPidController;
import trclib.TrcPidDrive;
import trclib.TrcSong;
import trclib.TrcSongPlayer;

@TeleOp(name="TeleOp: Wild Thumper", group="Ftc3543Sample")
//@Disabled
public class FtcTeleOpWildThumper extends FtcOpMode implements FtcGamepad.ButtonHandler, TrcPidController.PidInput
{
    private static final double ATTACK = 0.0;           // in seconds
    private static final double DECAY = 0.0;            // in seconds
    private static final double SUSTAIN = 1.0;          // in proportion
    private static final double RELEASE = 0.02;         // in seconds
    private static final double LOW_BEEP = 440.0;       // in Hz
    private static final double HIGH_BEEP = 880.0;      // in Hz
    private static final double BEEP_DURATION = 0.2;    // in seconds
    private static final double BAR_DURATION = 1.920;   // in seconds
    private static final int SONG_RESOURCE = R.raw.songcollection;

    private static final boolean SIX_WHEELS = false;
    private static final boolean LEFTWHEEL_INVERTED = false;
    private static final boolean RIGHTWHEEL_INVERTED = true;
    private static final boolean BRAKE_MODE_ON = true;
    public static final double VISIONDRIVE_KP           = 0.03;
    public static final double VISIONDRIVE_KI           = 0.0;
    public static final double VISIONDRIVE_KD           = 0.0;
    public static final double VISIONDRIVE_KF           = 0.0;
    public static final double VISIONDRIVE_TOLERANCE    = 2.0;
    public static final double VISIONDRIVE_SETTLING     = 0.2;

    public static final double VISIONTURN_KP            = 0.05;
    public static final double VISIONTURN_KI            = 0.0;
    public static final double VISIONTURN_KD            = 0.0;
    public static final double VISIONTURN_KF            = 0.0;
    public static final double VISIONTURN_TOLERANCE     = 2.0;
    public static final double VISIONTURN_SETTLING      = 0.2;

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
    private final boolean TRACK_ROBOT_LOC = true;
    private final boolean SPEECH_ENABLED = true;

    private HalDashboard dashboard;
    // Input and sensors.
    private FtcGamepad gamepad;
    private FtcMRGyro gyro;
    // Sound devices.
    private FtcAndroidTone androidTone = null;
    private FtcAnalogOutTone analogTone = null;
    private TrcSong[] collection = null;
    private int songIndex = -1;
    private TrcSongPlayer songPlayer = null;
    private TrcBooleanState envelopeToggle = new TrcBooleanState("EnvelopeToggle", true);
    private TrcBooleanState analogToneToggle = new TrcBooleanState("AnalogToneToggle", false);
    // Vision targeting.
    private FtcVuforia vuforia;
    private OpenGLMatrix lastKnownRobotLocation = null;
    private int targetIndex = 0;
    private float lastTargetX = 0.0f;
    private float lastTargetZ = 0.0f;
    // Text To Speech.
    private TextToSpeech textToSpeech = null;
    private boolean[] targetsFound = null;
    // Drive Base.
    private FtcDcMotor lfMotor;
    private FtcDcMotor rfMotor;
    private FtcDcMotor lmMotor;
    private FtcDcMotor rmMotor;
    private FtcDcMotor lrMotor;
    private FtcDcMotor rrMotor;
    private TrcDriveBase driveBase;
    private TrcPidController visionDrivePidCtrl;
    private TrcPidController visionTurnPidCtrl;
    private TrcPidDrive visionPidDrive;
    private TrcBooleanState followTargetToggle = new TrcBooleanState("FollowTarget", false);

    //
    // Implements FtcOpMode abstract methods.
    //

    @Override
    public void initRobot()
    {
        hardwareMap.logDevices();
        dashboard = getDashboard();
        FtcRobotControllerActivity activity = (FtcRobotControllerActivity)hardwareMap.appContext;
        dashboard.setTextView((TextView)activity.findViewById(R.id.textOpMode));
        //
        // Initializing Gamepads.
        //
        gamepad = new FtcGamepad("Gamepad", gamepad1, this);
        gamepad.setYInverted(true);
        //
        // Initializing sensors.
        //
        gyro = new FtcMRGyro("gyroSensor");
        gyro.calibrate();
        //
        // Initializing Tone support.
        //
        androidTone = new FtcAndroidTone("AndroidTone");
        androidTone.setSoundEnvelope(ATTACK, DECAY, SUSTAIN, RELEASE);
        androidTone.setSoundEnvelopeEnabled(envelopeToggle.getState());
        analogTone = new FtcAnalogOutTone("AnalogTone");
        InputStream songStream = activity.getResources().openRawResource(SONG_RESOURCE);
        try
        {
            FtcSongXml songXml = new FtcSongXml("Songs", songStream);
            collection = songXml.getCollection();
        }
        catch (Exception e)
        {
            e.printStackTrace();
        }
        songIndex = -1;
        //
        // Vision targeting.
        //
        vuforia = new FtcVuforia(VUFORIA_LICENSE_KEY, CAMERAVIEW_ID, CAMERA_DIR, TRACKABLES_FILE, targets.length);
        //
        // Phone location: Mounted center on the front of the robot with the back camera facing outward.
        //
        OpenGLMatrix phoneLocationOnRobot =
                TRACK_ROBOT_LOC? vuforia.locationMatrix(90.0f, 0.0f, 0.0f, 0.0f, ROBOT_WIDTH/2.0f, 0.0f): null;

        vuforia.setTargets(targets, phoneLocationOnRobot);
        //
        // Text To Speech.
        //
        if (SPEECH_ENABLED)
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
        //
        // DriveBase subsystem.
        //
        lfMotor = new FtcDcMotor("lfWheel");
        rfMotor = new FtcDcMotor("rfWheel");
        if (SIX_WHEELS)
        {
            lmMotor = new FtcDcMotor("lmWheel");
            rmMotor = new FtcDcMotor("rmWheel");
        }
        lrMotor = new FtcDcMotor("lrWheel");
        rrMotor = new FtcDcMotor("rrWheel");

        lfMotor.setInverted(LEFTWHEEL_INVERTED);
        rfMotor.setInverted(RIGHTWHEEL_INVERTED);
        if (SIX_WHEELS)
        {
            lmMotor.setInverted(LEFTWHEEL_INVERTED);
            rmMotor.setInverted(RIGHTWHEEL_INVERTED);
        }
        lrMotor.setInverted(LEFTWHEEL_INVERTED);
        rrMotor.setInverted(RIGHTWHEEL_INVERTED);
        //
        // 6V motors are too fast when driven with 12V so we need to use coast mode or the Thumper will tip forward
        // when stopping.
        //
        lfMotor.setBrakeModeEnabled(BRAKE_MODE_ON);
        rfMotor.setBrakeModeEnabled(BRAKE_MODE_ON);
        if (SIX_WHEELS)
        {
            lmMotor.setBrakeModeEnabled(BRAKE_MODE_ON);
            rmMotor.setBrakeModeEnabled(BRAKE_MODE_ON);
        }
        lrMotor.setBrakeModeEnabled(BRAKE_MODE_ON);
        rrMotor.setBrakeModeEnabled(BRAKE_MODE_ON);

        if (SIX_WHEELS)
        {
            driveBase = new TrcDriveBase(lfMotor, lmMotor, lrMotor, rfMotor, rmMotor, rrMotor);
        }
        else
        {
            driveBase = new TrcDriveBase(lfMotor, lrMotor, rfMotor, rrMotor);
        }
        //
        // PID Drive.
        //
        visionDrivePidCtrl = new TrcPidController(
                "visionDrivePidCtrl",
                VISIONDRIVE_KP, VISIONDRIVE_KI,
                VISIONDRIVE_KD, VISIONDRIVE_KF,
                VISIONDRIVE_TOLERANCE, VISIONDRIVE_SETTLING,
                this);
        visionTurnPidCtrl = new TrcPidController(
                "visionTurnPidCtrl",
                VISIONTURN_KP, VISIONTURN_KI,
                VISIONTURN_KD, VISIONTURN_KF,
                VISIONTURN_TOLERANCE, VISIONTURN_SETTLING,
                this);
        visionPidDrive = new TrcPidDrive(
                "visionPidDrive", driveBase, null, visionDrivePidCtrl, visionTurnPidCtrl);
    }   //initRobot

    /**
     * This method is called to start/stop the song. It takes care of keeping track of the song state and
     * will do the right thing if it is a start or a resume of the song.
     *
     * @param index specifies the index of the song to start or stop.
     * @param start specifies true to start the song, false to stop.
     */
    private void startSong(int index, boolean start)
    {
        if (start)
        {
            if (songPlayer == null)
            {
                //
                // This is the first time we start the song. So create the song player and associate it with the
                // appropriate tone generator.
                //
                songPlayer = new TrcSongPlayer("SongPlayer", analogToneToggle.getState() ? analogTone : androidTone);
            }
            songPlayer.playSong(collection[index], BAR_DURATION, true, false);
            songIndex = index;
        }
        else if (songPlayer != null)
        {
            //
            // Pause the song.
            //
            songPlayer.pause();
            songIndex = -1;
        }
    }   //startSong

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void startMode()
    {
        dashboard.clearDisplay();
        vuforia.setTrackingEnabled(true);
        driveBase.resetPosition();
        //
        // There is an issue with the gamepad objects that may not be valid
        // before waitForStart() is called. So we call the setGamepad method
        // here to update their references in case they have changed.
        //
        gamepad.setGamepad(gamepad1);
    }   //startMode

    @Override
    public void stopMode()
    {
        vuforia.setTrackingEnabled(false);
        //
        // If there is a SongPlayer, stop it.
        //
        if (songPlayer != null)
        {
            songPlayer.stop();
        }

        if (textToSpeech != null)
        {
            textToSpeech.stop();
            textToSpeech.shutdown();
        }
    }   //stopMode

    @Override
    public void runPeriodic(double elapsedTime)
    {
        double left = gamepad.getLeftStickY(true);
        double right = gamepad.getRightStickY(true);

        final int LABEL_WIDTH = 100;
        dashboard.displayPrintf(1, LABEL_WIDTH, "Power(L/R) = ", "%.2f/%.2f", left, right);
        dashboard.displayPrintf(2, LABEL_WIDTH, "GyroHeading = ", "%.2f", gyro.getZHeading().value);
        dashboard.displayPrintf(3, LABEL_WIDTH, "SoundEnvelope = ", "%s", envelopeToggle.getState()? "ON": "OFF");
        dashboard.displayPrintf(4, LABEL_WIDTH, "ToneDevice = ", "%s",
                                analogToneToggle.getState()? "AnalogOut": "Android");

        for (int i = 0; i < targets.length; i++)
        {
            VuforiaTrackable target = vuforia.getTarget(i);
            boolean visible = vuforia.isTargetVisible(target);

            if (SPEECH_ENABLED)
            {
                if (visible != targetsFound[i])
                {
                    targetsFound[i] = visible;
                    String sentence = String.format(
                            "%s is %s.", target.getName(), visible? "in view": "out of view");
                    textToSpeech.speak(sentence, TextToSpeech.QUEUE_FLUSH, null);
                }
            }

            String label = String.format(i == targetIndex? "<%s> = ": "%s = ", target.getName());
            OpenGLMatrix pose = vuforia.getTargetPose(target);
            if (pose != null)
            {
                VectorF translation = pose.getTranslation();
                dashboard.displayPrintf(
                        i + 5, LABEL_WIDTH, label, "%6.2f,%6.2f,%6.2f",
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

        if (lastKnownRobotLocation != null)
        {
            dashboard.displayPrintf(9, LABEL_WIDTH, "RobotLoc = ", lastKnownRobotLocation.formatAsTransform());
        }
    }   //runPeriodic

    //
    // Implements FtcGamepad.ButtonHandler interface.
    //

    @Override
    public void gamepadButtonEvent(FtcGamepad gamepad, int button, boolean pressed)
    {
        if (gamepad == this.gamepad)
        {
            switch (button)
            {
                case FtcGamepad.GAMEPAD_DPAD_UP:
                    if (pressed)
                    {
                        targetIndex = 0;
                    }
                    break;

                case FtcGamepad.GAMEPAD_DPAD_RIGHT:
                    if (pressed)
                    {
                        targetIndex = 1;
                    }
                    break;

                case FtcGamepad.GAMEPAD_DPAD_DOWN:
                    if (pressed)
                    {
                        targetIndex = 2;
                    }
                    break;

                case FtcGamepad.GAMEPAD_DPAD_LEFT:
                    if (pressed)
                    {
                        targetIndex = 3;
                    }
                    break;

                case FtcGamepad.GAMEPAD_A:
                    if (pressed)
                    {
                        followTargetToggle.toggleState();
                        if (followTargetToggle.getState())
                        {
                            visionPidDrive.setTarget(24.0, 0.0, true, null);
                        }
                        else
                        {
                            visionPidDrive.cancel();
                        }
                    }
                    break;

                case FtcGamepad.GAMEPAD_Y:
                    //
                    // Press this button to change the tone device between the Android phone or the Analog Output Port.
                    //
                    if (pressed)
                    {
                        analogToneToggle.toggleState();
                        //
                        // Since we changed the tone device, we need to destroy the old song player and
                        // create a new one with a different tone device.
                        //
                        int lastSongIndex = songIndex;
                        if (songPlayer != null)
                        {
                            songPlayer.stop();
                            songPlayer = null;
                        }
                        if (lastSongIndex != -1)
                        {
                            startSong(lastSongIndex, true);
                        }
                    }
                    break;

                case FtcGamepad.GAMEPAD_X:
                    //
                    // Press this button to turn on/off sound envelope.
                    //
                    if (pressed)
                    {
                        androidTone.setSoundEnvelopeEnabled(envelopeToggle.toggleState());
                    }
                    break;

                case FtcGamepad.GAMEPAD_B:
                    //
                    // Press this button to play a 440Hz beep and release to play a 880Hz beep.
                    //
                    if (pressed)
                    {
                        if (analogToneToggle.getState())
                        {
                            analogTone.playTone(LOW_BEEP, BEEP_DURATION);
                        }
                        else
                        {
                            androidTone.playTone(LOW_BEEP, BEEP_DURATION);
                        }
                    }
                    else
                    {
                        if (analogToneToggle.getState())
                        {
                            analogTone.playTone(HIGH_BEEP, BEEP_DURATION);
                        }
                        else
                        {
                            androidTone.playTone(HIGH_BEEP, BEEP_DURATION);
                        }
                    }
                    break;

                case FtcGamepad.GAMEPAD_LBUMPER:
                    if (pressed)
                    {
                        if (songIndex == 0)
                        {
                            //
                            // This song was playing, stop it.
                            //
                            startSong(0, false);
                        }
                        else if (songIndex == 1)
                        {
                            //
                            // The other song was playing, stop that and start this one.
                            //
                            startSong(1, false);
                            startSong(0, true);
                        }
                        else
                        {
                            //
                            // No song was playing, statt this one.
                            //
                            startSong(0, true);
                        }
                    }
                    break;

                case FtcGamepad.GAMEPAD_RBUMPER:
                    if (pressed)
                    {
                        if (songIndex == 1)
                        {
                            //
                            // This song was playing, stop it.
                            //
                            startSong(1, false);
                        }
                        else if (songIndex == 0)
                        {
                            //
                            // The other song was playing, stop that and start this one.
                            //
                            startSong(0, false);
                            startSong(1, true);
                        }
                        else
                        {
                            //
                            // No song was playing, statt this one.
                            //
                            startSong(1, true);
                        }
                    }
                    break;
            }
        }
    }   //gamepadButtonEvent

    //
    // Implements TrcPidController.PidInput
    //

    @Override
    public double getInput(TrcPidController pidCtrl)
    {
        double input = 0.0;

        VuforiaTrackable target = vuforia.getTarget(targetIndex);
        OpenGLMatrix pose = vuforia.getTargetPose(target);
        if (pose != null)
        {
            VectorF translation = pose.getTranslation();
            lastTargetX = translation.get(0)/MM_PER_INCH;
            lastTargetZ = -translation.get(2)/MM_PER_INCH;
        }

        if (pidCtrl == visionDrivePidCtrl)
        {
            input = lastTargetZ;
        }
        else if (pidCtrl == visionTurnPidCtrl)
        {
            input = Math.toDegrees(Math.atan2(lastTargetX, lastTargetZ));
        }

        return input;
    }   //getInput

    //
    // Implements TextToSpeech.OnInitListener interface.
    //

    public void onInit(int status)
    {
        if (status != TextToSpeech.ERROR)
        {
            textToSpeech.setLanguage(Locale.US);
        }
    }   //onInit

}   //class FtcTeleOpWildThumper
