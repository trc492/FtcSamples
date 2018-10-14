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
import hallib.HalDashboard;
import trclib.TrcBooleanState;
import trclib.TrcGameController;
import trclib.TrcRobot;
import trclib.TrcSimpleDriveBase;
import trclib.TrcSong;
import trclib.TrcSongPlayer;

@TeleOp(name="TeleOp: Wild Thumper", group="3543TeleOpSamples")
@Disabled
public class FtcTeleOpWildThumper extends FtcOpMode implements TrcGameController.ButtonHandler
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

    private HalDashboard dashboard;
    //
    // Input and sensors.
    //
    private FtcGamepad gamepad;
    private FtcMRGyro gyro;
    //
    // Sound devices.
    //
    private FtcAndroidTone androidTone = null;
    private FtcAnalogOutTone analogTone = null;
    private TrcSong[] collection = null;
    private int songIndex = -1;
    private TrcSongPlayer songPlayer = null;
    private TrcBooleanState envelopeToggle = new TrcBooleanState("EnvelopeToggle", true);
    private TrcBooleanState analogToneToggle = new TrcBooleanState("AnalogToneToggle", false);
    //
    // Drive Base.
    //
    private FtcDcMotor lfMotor;
    private FtcDcMotor rfMotor;
    private FtcDcMotor lmMotor;
    private FtcDcMotor rmMotor;
    private FtcDcMotor lrMotor;
    private FtcDcMotor rrMotor;
    private TrcSimpleDriveBase driveBase;

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
            driveBase = new TrcSimpleDriveBase(lfMotor, lmMotor, lrMotor, rfMotor, rmMotor, rrMotor);
        }
        else
        {
            driveBase = new TrcSimpleDriveBase(lfMotor, lrMotor, rfMotor, rrMotor);
        }
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
    public void startMode(TrcRobot.RunMode prevMode)
    {
        dashboard.clearDisplay();
        driveBase.resetOdometry();
    }   //startMode

    @Override
    public void stopMode(TrcRobot.RunMode nextMode)
    {
        //
        // If there is a SongPlayer, stop it.
        //
        if (songPlayer != null)
        {
            songPlayer.stop();
        }
    }   //stopMode

    @Override
    public void runPeriodic(double elapsedTime)
    {
        double left = gamepad.getLeftStickY(true);
        double right = gamepad.getRightStickY(true);
        driveBase.tankDrive(left, right);

        final int LABEL_WIDTH = 180;
        dashboard.displayPrintf(1, LABEL_WIDTH, "Power(L/R) = ", "%.2f/%.2f", left, right);
        dashboard.displayPrintf(2, LABEL_WIDTH, "GyroHeading = ", "%.2f", gyro.getZHeading().value);
        dashboard.displayPrintf(3, LABEL_WIDTH, "SoundEnvelope = ", "%s", envelopeToggle.getState()? "ON": "OFF");
        dashboard.displayPrintf(4, LABEL_WIDTH, "ToneDevice = ", "%s",
                                analogToneToggle.getState()? "AnalogOut": "Android");
    }   //runPeriodic

    //
    // Implements TrcGameController.ButtonHandler interface.
    //

    @Override
    public void buttonEvent(TrcGameController gamepad, int button, boolean pressed)
    {
        if (gamepad == this.gamepad)
        {
            switch (button)
            {
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
    }   //buttonEvent

}   //class FtcTeleOpWildThumper
