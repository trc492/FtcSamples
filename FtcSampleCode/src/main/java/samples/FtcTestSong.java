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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import ftclib.FtcOpMode;
import ftclib.FtcAndroidTone;
import hallib.HalDashboard;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcSong;
import trclib.TrcSongPlayer;
import trclib.TrcStateMachine;

@TeleOp(name="Test: Android Song", group="3543TestSamples")
@Disabled
public class FtcTestSong extends FtcOpMode
{
    private static final FtcAndroidTone.Waveform WAVE_FORM = FtcAndroidTone.Waveform.SINE_WAVE;
    private static final int SAMPLE_RATE = 16*1024;     // ~16kHz
    private static final double ATTACK = 0.0;           // in seconds
    private static final double DECAY = 0.0;            // in seconds
    private static final double SUSTAIN = 1.0;          // in proportion
    private static final double RELEASE = 0.02;         // in seconds
    private static final double BAR_DURATION = 1.920;   // in seconds
    //
    // Note string syntax:
    //  <note>[#|b]<octave>.<noteType>[+]{.<noteType>[+]}
    //  where <note>     - 'A' through 'G'
    //        #          - sharp
    //        b          - flat
    //        <octave>   - 1 through 8 (e.g. C4 is the middle C)
    //        <noteType> - note type (1: whole, 2: half, 4: quarter, ...)
    //                   - add half time
    //
    private static final String[] starWarsSections =
    {
            // section 1
            "1:G4.12,G4.12,G4.12,"
                    + "C5.2,G5.2,"
                    + "F5.12,E5.12,D5.12,C6.2,G5.4,"
                    + "F5.12,E5.12,D5.12,C6.2,G5.4,"
                    + "F5.12,E5.12,F5.12,D5.2,G4.8,G4.8,"
                    + "C5.2,G5.2",
            // section 2
            "2:F5.12,E5.12,D5.12,C6.2,G5.4,"
                    + "F5.12,E5.12,D5.12,C6.2,G5.4,"
                    + "F5.12,E5.12,F5.12,D5.2,G4.8,G4.8,"
                    + "A4.4+,A4.8,F5.8,E5.8,D5.8,C5.8,"
                    + "C5.12,D5.12,E5.12,D5.4,B4.4,G4.8,G4.8,"
                    + "A4.4+,A4.8,F5.8,E5.8,D5.8,C5.8,"
                    + "G5.4,D5.2,G4.8,G4.8",
            // section 3
            "3:A4.4+,A4.8,F5.8,E5.8,D5.8,C5.8,"
                    + "C5.12,D5.12,E5.12,D5.4,B4.4,G5.8,G5.8,"
                    + "C6.8,Bb5.8,Ab5.8,G5.8,F5.8,Eb5.8,D5.8,C5.8,"
                    + "G5.2+"
    };
    private static final String starWarsSequence = "1,2,3";
    private static final String[] lesMiserablesSections =
    {
            // section 1
            "1:A4.8+,G4.16,"
                    + "F4.8+,G4.16,A4.8+,Bb4.16,C5.4,A4.12,G4.12,F4.12,"
                    + "E4.8+,D4.16,E4.8+,F4.16,C4.4,D4.12,C4.12,Bb3.12,"
                    + "A3.8+,C4.16,F4.8+,A4.16,G4.8+,F#4.16,G4.8+,D4.16,"
                    + "F4.8+,E4.16,E4.8+,F4.16,G4.4,A4.8+,G4.16",
            // section 2
            "2:F4.8+,G4.16,A4.8+,Bb4.16,C5.4,A4.12,G4.12,F4.12,"
                    + "E4.8+,D4.16,E4.8+,F4.16,C4.4,D4.12,C4.12,Bb3.12,"
                    + "A3.8+,C4.16,F4.8+,A4.16,G4.12,F#4.12,G4.12,Bb4.8+,E4.16,"
                    + "F4.4,R.2,E4.8+,E4.16",
            // section 3
            "3:A4.8+,G#4.16,A4.8+,B4.16,C5.8+,B4.16,A4.8+,C5.16,"
                    + "B4.8+,A4.16,G4.8+,A4.16,B4.4,R.12,B4.12,C5.12,"
                    + "D5.8+,C5.16,B4.8+,C5.16,D5.8+,C5.16,B4.8+,D5.16,"
                    + "C5.8+,B4.16,A4.8+,B4.16,C5.4,R.8+,A4.16",
            // section 4
            "4:C5.12,B4.12,A4.12,C5.12,B4.12,A4.12,C5.12,B4.12,A4.12,C5.12,B4.12,C5.12,"
                    + "D5.2,R.4,E5.8+,D5.16,"
                    + "C5.8+,D5.16,E5.8+,F5.16,G5.4,E5.12,D5.12,C5.12,"
                    + "B4.8+,A4.16,B4.8+,C5.16,G4.4,A4.12,G4.12,F4.12",
            // section 5
            "5:E4.8+,G4.16,C5.8+,E5.16,D5.8+,C#5.16,D5.8+,A4.16,"
                    + "C5.8+,B4.16,B4.8+,C5.16,D5.4,E5.8+,D5.16,"
                    + "C5.8+,D5.16,E5.8+,F5.16,G5.4,E5.12,D5.12,C5.12,"
                    + "B4.8+,A4.16,B4.8+,C5.16,G4.4,A4.12,G4.12,F4.12",
            // section 6
            "6:E4.8+,G4.16,C5.8+,E5.16,D5.12,C#5.12,D5.12,F5.8+,B4.16,"
                    + "C5.4,R.2,E4.8+,E4.16",
            // section 7
            "7:E4.8+,G4.16,C5.8+,E5.16,D5.12,C#5.12,D5.12,F5.8+,B4.16,"
                    + "C5.4,R.2.4"
    };
    private static final String lesMiserablesSequence = "1,2,3,4,5,6,3,4,5,7";

    private enum State
    {
        PLAY_STARWARS,
        PLAY_LESMISERABLES,
        DONE
    }   //enum State

    private HalDashboard dashboard;
    private FtcAndroidTone androidTone;
    private TrcSong starWars = new TrcSong("StarWars", starWarsSections, starWarsSequence);
    private TrcSong lesMiserables = new TrcSong("LesMiserables", lesMiserablesSections, lesMiserablesSequence);
    private TrcSongPlayer songPlayer;
    private TrcStateMachine sm;
    private TrcEvent event = null;

    //
    // Implements FtcOpMode abstract methods.
    //

    @Override
    public void initRobot()
    {
        hardwareMap.logDevices();
        dashboard = HalDashboard.getInstance();

        androidTone = new FtcAndroidTone("AndroidTone", WAVE_FORM, SAMPLE_RATE);
        androidTone.setSoundEnvelope(ATTACK, DECAY, SUSTAIN, RELEASE);
        androidTone.setSoundEnvelopeEnabled(true);
        songPlayer = new TrcSongPlayer("SongPlayer", androidTone);
        sm = new TrcStateMachine("SongPlayer");
        event = new TrcEvent("SongCompletion");
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        dashboard.clearDisplay();
        sm.start(State.PLAY_STARWARS);
    }   //startMode

    @Override
    public void stopMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        sm.stop();
        songPlayer.stop();
    }   //startMode

    @Override
    public void runPeriodic(double elapsedTime)
    {
        State state = (State)sm.getState();

        dashboard.displayPrintf(1, "State: %s", state == null? "Null": state.toString());
        if (sm.isReady())
        {
            state = (State)sm.getState();
            switch (state)
            {
                case PLAY_STARWARS:
                    songPlayer.playSong(starWars, BAR_DURATION, false, event);
                    sm.addEvent(event);
                    sm.waitForEvents(State.PLAY_LESMISERABLES);
                    break;

                case PLAY_LESMISERABLES:
                    songPlayer.playSong(lesMiserables, BAR_DURATION, false, event);
                    sm.addEvent(event);
                    sm.waitForEvents(State.DONE);
                    break;

                case DONE:
                default:
                    sm.stop();
                    break;
            }
        }
    }   //runPeriodic

}   //class FtcTestSong
