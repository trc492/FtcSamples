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

package trclib;

/**
 * This class implements a song player that can parse a notated song in a string buffer
 * and play the notes on a Tone device.
 */
public class TrcSongPlayer implements TrcTaskMgr.Task
{
    private static final String moduleName = "TrcSongPlayer";
    private static final boolean debugEnabled = false;
    private TrcDbgTrace dbgTrace = null;

    private String instanceName;
    private TrcTone tone;
    private double barDuration = 0.0;
    private TrcSong song = null;
    private boolean repeat = false;
    private TrcEvent event = null;
    private String[] notes = null;
    private int noteIndex = 0;
    private double volume = 1.0;

    /**
     * Constructor: Create and initialize an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param tone specifies the Tone player.
     */
    public TrcSongPlayer(String instanceName, TrcTone tone)
    {
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(
                    moduleName + "." + instanceName,
                    false,
                    TrcDbgTrace.TraceLevel.API,
                    TrcDbgTrace.MsgLevel.INFO);
        }

        this.instanceName = instanceName;
        this.tone = tone;
    }   //TrcSongPlayer

    /**
     * This method enables/disables the player task.
     *
     * @param enabled specifies true to enable, false otherwise.
     */
    private void setTaskEnabled(boolean enabled)
    {
        final String funcName = "setTaskEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC, "enabled=%s", Boolean.toString(enabled));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC);
        }

        TrcTaskMgr taskMgr = TrcTaskMgr.getInstance();
        if (enabled)
        {
            taskMgr.registerTask(instanceName, this, TrcTaskMgr.TaskType.STOP_TASK);
            taskMgr.registerTask(instanceName, this, TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        }
        else
        {
            taskMgr.unregisterTask(this, TrcTaskMgr.TaskType.STOP_TASK);
            taskMgr.unregisterTask(this, TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        }
    }   //setTaskEnabled

    /**
     * This method stops the sound and disables the player task.
     */
    public void stop()
    {
        final String funcName = "stop";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        tone.stop();
        setTaskEnabled(false);
    }   //stop

    /**
     * This method is called to pause the player.
     */
    public void pause()
    {
        stop();
    }   //pause

    /**
     * This method is called to resume the player.
     */
    public void resume()
    {
        setTaskEnabled(true);
    }   //resume

    /**
     * This method rewinds the song back to the beginning.
     */
    public void rewind()
    {
        final String funcName = "rewind";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (song != null)
        {
            song.rewind();
            notes = null;
            noteIndex = 0;
        }
    }   //rewind

    /**
     * This method plays the specified song.
     *
     * @param song specifies the song to be played.
     * @param barDuration specifies the bar duration in seconds.
     * @param repeat specifies true to play the song repeatedly, false otherwise.
     * @param event specifies the event to be notified on song completion.
     */
    private void playSong(TrcSong song, double barDuration, boolean repeat, TrcEvent event)
    {
        final String funcName = "playSong";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.song = song;
        this.barDuration = barDuration;
        this.repeat = repeat;
        this.event = repeat? null: event;
        rewind();
        setTaskEnabled(true);
    }   //playSong

    /**
     * This method plays the specified song.
     *
     * @param song specifies the song to be played.
     * @param barDuration specifies the bar duration in seconds.
     * @param repeat specifies true to play the song repeatedly, false otherwise.
     */
    public void playSong(TrcSong song, double barDuration, boolean repeat)
    {
        playSong(song, barDuration, repeat, null);
    }   //playSong

    /**
     * This method plays the specified song.
     *
     * @param song specifies the song to be played.
     * @param barDuration specifies the bar duration in seconds.
     * @param event specifies the event to be notified on song completion.
     */
    public void playSong(TrcSong song, double barDuration, TrcEvent event)
    {
        playSong(song, barDuration, false, event);
    }   //playSong

    /**
     * This method plays the specified song.
     *
     * @param song specifies the song to be played.
     * @param barDuration Specifies the bar duration in seconds.
     */
    public void playSong(TrcSong song, double barDuration)
    {
        playSong(song, barDuration, false, null);
    }   //playSong

    /**
     * This method parses the note string for frequency and duration and plays it.
     *
     * @param note Specifies the note string in the format:
     *        <note>[#|b]<octave>.<noteType>[+]{.<noteType>[+]}
     *        where <note>     - 'A' through 'G'
     *              #          - sharp
     *              b          - flat
     *              <octave>   - 1 through 8
     *              <noteType> - note type (1: whole, 2: half, 4: quarter, ...)
     *              +          - add half time
     * @param barDuration Specifies the bar duration in seconds.
     */
    private void playNote(String note, double barDuration)
    {
        final String funcName = "playNote";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "note=%s,barDur=%.3f", note, barDuration);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        int dotIndex = note.indexOf('.');

        if (dotIndex != -1)
        {
            double noteFreq = parseFrequency(note.substring(0, dotIndex));
            double noteLen = parseDuration(note.substring(dotIndex + 1), barDuration);
            tone.playTone(noteFreq, noteLen, volume);
        }
        else
        {
            throw new IllegalArgumentException("Missing note duration <" + note + ">.");
        }
    }   //playNote

    /**
     * This method parses the note frequency from the note string.
     *
     * @param note Specifies the note string in the format:
     *        <note>[#|b]<octave>
     *        where <note>   - 'A' through 'G'
     *              #        - sharp
     *              b        - flat
     *              <octave> - 1 through 8
     *
     * @return note frequency on success and throws IllegalArgumentException on failure.
     */
    private double parseFrequency(String note)
    {
        final String funcName = "parseFrequency";
        double freq;

        if (note.charAt(0) >= 'A' && note.charAt(0) <= 'G')
        {
            //
            // Parse note.
            //
            int noteNum = note.charAt(0) - 'C';

            if (noteNum < 0)
            {
                noteNum = (noteNum + 7)*2 + 3;
            }
            else if (noteNum > 2)
            {
                noteNum = noteNum*2 + 3;
            }
            else
            {
                noteNum = noteNum*2 + 4;
            }

            //
            // Parse sharp or flat.
            //
            int i = 1;
            if (i < note.length())
            {
                if (note.charAt(i) == '#')
                {
                    noteNum++;
                    i++;
                }
                else if (note.charAt(i) == 'b')
                {
                    noteNum--;
                    i++;
                }
            }

            //
            // Parse octave.
            //
            if (i < note.length())
            {
                if ((note.charAt(i) >= '1') && (note.charAt(i) <= '8'))
                {
                    noteNum += (note.charAt(i) - '1') * 12;
                    freq = 440.0 * Math.pow(2.0, (noteNum - 49.0) / 12.0);
                }
                else
                {
                    throw new IllegalArgumentException("Invalid note <" + note + ">, invalid octave.");
                }
            }
            else
            {
                throw new IllegalArgumentException("Invalid note <" + note + ">, missing octave.");
            }
        }
        else if (note.charAt(0) == 'R')
        {
            //
            // Note is a rest.
            //
            freq = 0.0;
        }
        else
        {
            throw new IllegalArgumentException("Invalid note <" + note + ">.");
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC, "note=%s", note);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC, "=%f", freq);
        }

        return freq;
    }   //parseFrequency

    /**
     * This method parses the note length from the note string.
     *
     * @param note Specifies the note string in the format:
     *        <noteType>[+]{.<noteType>[+]}
     *        where <noteType> - note type (1: whole, 2: half, 4: quarter, ...)
     *              +          - add half time
     * @param barDuration Specifies the bar duration in seconds.
     *
     * @return note length in seconds on success and throws IllegalArgumentException on failure.
     */
    private double parseDuration(String note, double barDuration)
    {
        final String funcName = "parseDuration";
        double noteLen = 0.0;
        int dotIndex;

        while ((dotIndex = note.indexOf('.')) != -1)
        {
            noteLen += parseDuration(note.substring(0, dotIndex), barDuration);
            note = note.substring(dotIndex + 1);
        }

        try
        {
            if (note.charAt(note.length() - 1) == '+')
            {
                //
                // 1.5 time.
                //
                noteLen += (barDuration/Integer.parseInt(note.substring(0, note.length() - 1)))*1.5;
            }
            else
            {
                noteLen += barDuration/Integer.parseInt(note);
            }
        }
        catch (Exception e)
        {
            throw new IllegalArgumentException("Invalid duration <" + note + ">.");
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC, "note=%s", note);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC, "=%f", noteLen);
        }

        return noteLen;
    }   //parseDuration

    /**
     * This method parses the dynamics volume.
     *
     * @param notation Specifies the dynamics volume notation.
     * @return note volume in the range of 1.0, or -1.0 if not a valid dynamics notation.
     */
    private double parseDynamicsVolume(String notation)
    {
        final String funcName = "parseDynamicsVolume";
        final String[] dynamics = {"ppp", "pp", "p", "mp", "mf", "f", "ff", "fff"};
        double volume = -1.0;

        for (int i = 0; i < dynamics.length; i++)
        {
            if (notation.equals(dynamics[i]))
            {
                volume = (i + 3)*0.1;
                break;
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC, "notation=%s", notation);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC, "=%f", volume);
        }

        return volume;
    }   //parseDynamicsVolume

    /**
     * This method checks if the notation is a dynamics. If so, parses it and performs the appropriate action.
      *
     * @param notation specifies the notation string.
     * @return true if the notation is a dynamics, false otherwise.
     */
    private boolean parseDynamics(String notation)
    {
        final String funcName = "parseDynamics";
        boolean isDynamics = false;

        if (notation.charAt(0) == '<')
        {
            // TODO: handle Crescendo.
            isDynamics = true;
        }
        else if (notation.charAt(0) == '>')
        {
            // TODO: handle Diminuendo.
            isDynamics = true;
        }
        else
        {
            double vol = parseDynamicsVolume(notation);
            //
            // If invalid, skip it assuming it is a future feature we don't handle yet.
            //
            if (vol != -1.0)
            {
                volume = vol;
                isDynamics = true;
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC, "notation=%s", notation);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC, "=%s", Boolean.toString(isDynamics));
        }

        return isDynamics;
    }   //parseDynamics

    /**
     * This method parses and performs the notation action. It throws an IllegalArgumentException if it is not
     * a recognized notation.
     *
     * @param notation specifies the notation string.
     */
    private void performNotation(String notation)
    {
        boolean found;

        found = parseDynamics(notation);

        if (!found)
        {
            throw new IllegalArgumentException("Invalid notation <" + notation + ">.");
        }
    }   //performNotation

    //
    // Implements TrcTaskMgr.Task
    //

    @Override
    public void startTask(TrcRobot.RunMode runMode)
    {
    }   //startTask

    /**
     * This method is called when the competition mode is about to end. It stops the
     * player if sound is playing.
     *
     * @param runMode specifies the competition mode that is about to
     */
    @Override
    public void stopTask(TrcRobot.RunMode runMode)
    {
        final String funcName = "stopTask";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.TASK,
                    "mode=%s", runMode.toString());
        }

        stop();

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //stopTask

    @Override
    public void prePeriodicTask(TrcRobot.RunMode runMode)
    {
    }   //prePeriodicTask

    @Override
    public void postPeriodicTask(TrcRobot.RunMode runMode)
    {
    }   //postPeriodicTask

    @Override
    public void preContinuousTask(TrcRobot.RunMode runMode)
    {
    }   //preContinuousTask

    /**
     * This method is called periodically to check and play the next note in the song.
     *
     * @param runMode specifies the competition mode that is running.
     */
    @Override
    public void postContinuousTask(TrcRobot.RunMode runMode)
    {
        final String funcName = "postContinuous";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.TASK,
                    "mode=%s", runMode.toString());
        }

        //
        // Move on to the next note only if the current note has finished playing.
        //
        if (!tone.isPlaying())
        {
            if (notes == null)
            {
                //
                // Either we just started a song or we just finished a section or we have finished the song.
                //
                String section = song.getNextSection();
                if (section != null)
                {
                    //
                    // There is a next section, parse the next section.
                    //
                    notes = section.split(",");
                    noteIndex = 0;
                }
                else if (repeat)
                {
                    //
                    // Resets the song to the beginning.
                    //
                    song.rewind();
                    section = song.getNextSection();
                    notes = section.split(",");
                    noteIndex = 0;
                }
                else
                {
                    //
                    // There is no more section. The song is done.
                    //
                    setTaskEnabled(false);
                    if (event != null)
                    {
                        event.set(true);
                    }
                }
            }

            if (notes != null)
            {
                while (noteIndex < notes.length && notes[noteIndex].charAt(0) == '@')
                {
                    performNotation(notes[noteIndex].substring(1));
                    noteIndex++;
                }

                if (noteIndex < notes.length)
                {
                    //
                    // There is a next note, play it.
                    //
                    playNote(notes[noteIndex], barDuration);
                    noteIndex++;
                    if (noteIndex >= notes.length)
                    {
                        //
                        // The section is done, null the notes array so we will parse the
                        // next section next time around.
                        //
                        notes = null;
                    }
                }
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //postContinuousTask

}   //class TrcSongPlayer
