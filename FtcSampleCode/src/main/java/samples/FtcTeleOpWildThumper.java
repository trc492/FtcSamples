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

import android.widget.TextView;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import java.io.InputStream;

import FtcSampleCode.R;
import ftclib.FtcAnalogOutTone;
import ftclib.FtcAndroidTone;
import ftclib.FtcDcMotor;
import ftclib.FtcGamepad;
import ftclib.FtcMRGyro;
import ftclib.FtcOpMode;
import hallib.HalDashboard;
import trclib.TrcBooleanState;
import trclib.TrcDriveBase;
import trclib.TrcSong;
import trclib.TrcSongPlayer;

@TeleOp(name="TeleOp: Wild Thumper", group="Ftc3543Sample")
//@Disabled
public class FtcTeleOpWildThumper extends FtcOpMode implements FtcGamepad.ButtonHandler
{
    private static final double ATTACK = 0.0;           // in seconds
    private static final double DECAY = 0.0;            // in seconds
    private static final double SUSTAIN = 1.0;          // in proportion
    private static final double RELEASE = 0.02;         // in seconds
    private static final double LOW_BEEP = 440.0;       // in Hz
    private static final double HIGH_BEEP = 880.0;      // in Hz
    private static final double BEEP_DURATION = 0.2;    // in seconds
    private static final double BAR_DURATION = 1.920;   // in seconds
    private static final String SONG_NAME = "LesMiserables";
    private static final int SONG_RESOURCE = R.raw.lesmiserables;

    private static final boolean LEFTWHEEL_INVERTED = false;
    private static final boolean RIGHTWHEEL_INVERTED = true;
    private static final double DRIVE_SCALE = 0.3;
    private static final boolean MOTORS_6V = false;     // true if robot has 6V motors instead of 12V.

    private HalDashboard dashboard;
    // Input and sensors.
    private FtcGamepad gamepad;
    private FtcMRGyro gyro;
    // Sound devices.
    private FtcAndroidTone androidTone = null;
    private FtcAnalogOutTone analogTone = null;
    private TrcSongPlayer songPlayer = null;
    private TrcBooleanState songToggle = new TrcBooleanState("SongToggle", false);
    private TrcBooleanState envelopeToggle = new TrcBooleanState("EnvelopeToggle", true);
    private TrcBooleanState analogToneToggle = new TrcBooleanState("AnalogToneToggle", false);
    private TrcSong lesMiserables = null;
    // Drive Base.
    private FtcDcMotor lfMotor;
    private FtcDcMotor rfMotor;
    private FtcDcMotor lmMotor;
    private FtcDcMotor rmMotor;
    private FtcDcMotor lrMotor;
    private FtcDcMotor rrMotor;
    private TrcDriveBase driveBase;
    private boolean brakeModeOn;
    private boolean fullPowerEnabled = false;

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
        InputStream input = activity.getResources().openRawResource(SONG_RESOURCE);
        lesMiserables = new TrcSong(SONG_NAME, input);
        //
        // DriveBase subsystem.
        //
        lfMotor = new FtcDcMotor("lfWheel");
        rfMotor = new FtcDcMotor("rfWheel");
        lmMotor = new FtcDcMotor("lmWheel");
        rmMotor = new FtcDcMotor("rmWheel");
        lrMotor = new FtcDcMotor("lrWheel");
        rrMotor = new FtcDcMotor("rrWheel");

        lfMotor.setInverted(LEFTWHEEL_INVERTED);
        rfMotor.setInverted(RIGHTWHEEL_INVERTED);
        lmMotor.setInverted(LEFTWHEEL_INVERTED);
        rmMotor.setInverted(RIGHTWHEEL_INVERTED);
        lrMotor.setInverted(LEFTWHEEL_INVERTED);
        rrMotor.setInverted(RIGHTWHEEL_INVERTED);
        //
        // 6V motors are too fast when driven with 12V so we need to use coast mode or the Thumper will tip forward
        // when stopping.
        //
        brakeModeOn = !MOTORS_6V;
        lfMotor.setBrakeModeEnabled(brakeModeOn);
        rfMotor.setBrakeModeEnabled(brakeModeOn);
        lmMotor.setBrakeModeEnabled(brakeModeOn);
        rmMotor.setBrakeModeEnabled(brakeModeOn);
        lrMotor.setBrakeModeEnabled(brakeModeOn);
        rrMotor.setBrakeModeEnabled(brakeModeOn);

        driveBase = new TrcDriveBase(lfMotor, lmMotor, lrMotor, rfMotor, rmMotor, rrMotor);
    }   //initRobot

    /**
     * This method is called to start/stop the song. It takes care of keeping track of the song state and
     * will do the right thing if it is a start or a resume of the song.
     *
     * @param start specifies true to start the song, false to stop.
     */
    private void startSong(boolean start)
    {
        if (start)
        {
            if (songPlayer == null)
            {
                //
                // This is the first time we start the song. So create the song player and start the song.
                //
                songPlayer = new TrcSongPlayer("SongPlayer", analogToneToggle.getState()? analogTone: androidTone);
                songPlayer.playSong(lesMiserables, BAR_DURATION, true);
            }
            else
            {
                //
                // The song player already exists, it is actually a resume.
                //
                songPlayer.resume();
            }
        }
        else if (songPlayer != null)
        {
            //
            // Pause the song.
            //
            songPlayer.pause();
        }
    }   //startSong

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void startMode()
    {
        dashboard.clearDisplay();
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
        //
        // DriveBase subsystem:
        // WildThumper has 6V motors. We are driving it with 12V and it seems fine.
        // But it is running really fast, so we are multiplying the drive power with a
        // scaling factor to slow it down. However, it has 6 high traction wheels and
        // it has a hard time doing in-place turn or even pivot turn. So we will give
        // it full power for turning and only slows down for driving forward and backward.
        // However, if the fullPowerEnabled button is pressed and held down, we will give
        // it full power regardless.
        //
        double left = gamepad.getLeftStickY(true);
        double right = gamepad.getRightStickY(true);
        if (MOTORS_6V)
        {
            if (!fullPowerEnabled && left*right > 0.0)
            {
                left *= DRIVE_SCALE;
                right *= DRIVE_SCALE;
            }
        }
        driveBase.tankDrive(left, right);

        final int DISPLAY_WIDTH = 444;
        final int LABEL_WIDTH = DISPLAY_WIDTH/2;
        dashboard.displayCenterPrintf(1, DISPLAY_WIDTH, "WildThumper");
        dashboard.displayPrintf(2, LABEL_WIDTH, "left power = ", "%.2f", left);
        dashboard.displayPrintf(3, LABEL_WIDTH, "right power = ", "%.2f", right);
        dashboard.displayPrintf(4, LABEL_WIDTH, "Gyro heading = ", "%.2f", gyro.getZHeading().value);
        dashboard.displayPrintf(5, LABEL_WIDTH, "Full Power = ", "%s", Boolean.toString(fullPowerEnabled));
        dashboard.displayPrintf(6, LABEL_WIDTH, "SoundEnvelope = ", "%s", envelopeToggle.getState()? "ON": "OFF");
        dashboard.displayPrintf(7, LABEL_WIDTH, "Tone device = ", "%s",
                                analogToneToggle.getState()? "AnalogOut": "Android");
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
                case FtcGamepad.GAMEPAD_A:
                    //
                    // Press this button to start/stop the song.
                    //
                    if (pressed)
                    {
                        startSong(songToggle.toggleState());
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
                        if (songPlayer != null)
                        {
                            songPlayer.stop();
                            songPlayer = null;
                        }
                        startSong(songToggle.getState());
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
                    //
                    // Press and hold this button for full power driving.
                    //
                    fullPowerEnabled = pressed;
                    break;
            }
        }
    }   //gamepadButtonEvent

}   //class FtcTeleOpWildThumper
