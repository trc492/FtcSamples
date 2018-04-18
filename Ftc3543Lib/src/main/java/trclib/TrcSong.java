/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
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

import java.util.ArrayList;

/**
 * This class implements a song object that contains a song name, an array list of notated sections and a sequencing
 * array specifying the order of the sections. It also keeps track of the next note position of the song and provides
 * a getNextNote() method to retrieve the next note to play.
 */
public class TrcSong
{
    private static final String moduleName = "TrcSong";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    /**
     * This class implements a notated section that contains a section name, an array of notes and keeps track of
     * the next note of the section and provides a getNextNote() method to retrieve the next note to play.
     */
    private class Section
    {
        private final String name;
        private final String[] notes;
        private int noteIndex;

        /**
         * Constructor: Create an instance of the object.
         *
         * @param name specifies the name of the section.
         * @param section specifies the string that contains all the notated notes in the section.
         */
        public Section(String name, String section)
        {
            this.name = name;
            this.notes = section.split("[, \t\n]");
            noteIndex = 0;
        }   //Section

        /**
         * This method returns the name of the section.
         *
         * @return section name.
         */
        public String getName()
        {
            final String funcName = "section.toString";

            if (debugEnabled)
            {
                dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
                dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", name);
            }

            return name;
        }   //toString

        /**
         * This method determines if there is a next note in the section.
         *
         * @return true if there is a next note, false otherwise.
         */
        public boolean hasNextNote()
        {
            final String funcName = "section.hasNextNote";
            boolean hasNote = noteIndex < notes.length;

            if (debugEnabled)
            {
                dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
                dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Boolean.toString(hasNote));
            }

            return hasNote;
        }   //hasNextNote

        /**
         * This method returns the next note in the section.
         *
         * @return next note to play or null if no more note.
         */
        public String getNextNote()
        {
            final String funcName = "section.getNextNote";
            String note = null;

            if (debugEnabled)
            {
                dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            }

            //
            // Make sure it is not an empty string. If it is, skip it and get the next note.
            //
            while (noteIndex < notes.length && notes[noteIndex].length() == 0)
            {
                noteIndex++;
            }

            if (noteIndex < notes.length)
            {
                note = notes[noteIndex++];
            }

            if (debugEnabled)
            {
                dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", note == null? "null": note);
            }

            return note;
        }   //getNextNote

        /**
         * This method rewinds the note pointer back to the beginning of the section.
         */
        public void rewind()
        {
            final String funcName = "section.rewind";

            if (debugEnabled)
            {
                dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
                dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
            }

            noteIndex = 0;
        }   //rewind

    }   //class Section

    private final String songName;
    private double startVolume = 1.0;
    private ArrayList<Section> sections = new ArrayList<>();
    private String[] sequence = null;
    private int sequenceIndex = 0;
    private Section currSection = null;
    private double currVolume = startVolume;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param name specifies the name of the song.
     * @param startVolume specifies the starting volume.
     */
    public TrcSong(final String name, double startVolume)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName + "." + name, tracingEnabled, traceLevel, msgLevel);
        }

        this.songName = name;
        this.startVolume = startVolume;
    }   //TrcSong

    /**
     * Constructor: Create an instance of the object.
     *
     * @param name specifies the name of the song.
     */
    public TrcSong(String name)
    {
        this(name, 1.0);
    }   //TrcSong

    /**
     * Constructor: Create an instance of the object.
     *
     * @param name specifies the name of the song.
     * @param startVolume specifies the starting volume.
     * @param sections specifies an array of notated song sections.
     * @param sequence specifies the song sequence array.
     */
    public TrcSong(String name, double startVolume, String[] sections, String sequence)
    {
        this(name, startVolume);

        for (String s: sections)
        {
            String[] pair = s.split(":");

            if (pair.length != 2)
            {
                throw new IllegalArgumentException("Illegal section format, must be <Name>:<NoteList>.");
            }

            addSection(pair[0], pair[1]);
        }

        setSequence(sequence);
    }   //TrcSong

    /**
     * Constructor: Create an instance of the object.
     *
     * @param name specifies the name of the song.
     * @param sections specifies an array of notated song sections.
     * @param sequence specifies the song sequence array.
     */
    public TrcSong(String name, String[] sections, String sequence)
    {
        this(name, 1.0, sections, sequence);
    }   //TrcSong

    /**
     * This method returns the song name.
     *
     * @return instance name.
     */
    public String toString()
    {
        return songName;
    }   //toString

    /**
     * This method adds a notated section to the song.
     *
     * @param name specifies the section name.
     * @param section specifies the string that contains notated notes of the section.
     */
    public void addSection(String name, String section)
    {
        final String funcName = "addSection";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "name=%s,section=<%s>", name, section);
        }

        sections.add(new Section(name, section));

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //addSection

    /**
     * This method sets the sequencing of the song.
     *
     * @param sequence specifies the string that contains the sequence of section names.
     */
    public void setSequence(String sequence)
    {
        final String funcName = "setSequence";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "sequence=%s", sequence);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.sequence = sequence.split("[, \t\n]");
        sequenceIndex = 0;
    }   //setSequence

    /**
     * This method retrieves the next note of the song to be played.
     *
     * @return next notated note or null if no more note.
     */
    public String getNextNote()
    {
        final String funcName = "getNextNote";
        String note = null;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        //
        // Either we just started the song or the current section has no more note.
        //
        while (currSection == null || !currSection.hasNextNote())
        {
            //
            // Get the next section name from the sequence array skipping blank section names if any.
            //
            while (sequenceIndex < sequence.length && sequence[sequenceIndex].length() == 0)
            {
                sequenceIndex++;
            }

            if (sequenceIndex < sequence.length)
            {
                //
                // Find the section object with the given name.
                //
                String sectionName = sequence[sequenceIndex++];
                currSection = findSection(sectionName);
                if (currSection != null)
                {
                    currSection.rewind();
                }
                else
                {
                    throw new IllegalStateException("Section " + sectionName + " not found.");
                }
            }
            else
            {
                //
                // We ran out of sections.
                //
                break;
            }
        }

        if (currSection != null)
        {
            note = currSection.getNextNote();
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", note == null? "null": note);
        }

        return note;
    }   //getNextNote

    /**
     * This method returns the current volume of the song.
     *
     * @return current volume of the song.
     */
    public double getCurrentVolume()
    {
        final String funcName = "getCurrentVolume";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%.1f", currVolume);
        }

        return currVolume;
    }   //getCurrentVolume

    /**
     * This method sets the current volume of the song.
     *
     * @param vol specifies the current volume of the song.
     */
    public void setCurrentVolume(double vol)
    {
        final String funcName = "setCurrentVolume";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "vol=%f", vol);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        currVolume = vol;
    }   //setCurrentVolume

    /**
     * This method sets the start volume of the song.
     *
     * @param vol specifies the start volume of the song.
     */
    public void setStartVolume(double vol)
    {
        final String funcName = "setStartVolume";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "vol=%f", vol);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        startVolume = vol;
    }   //setStartVolume

    /**
     * This method rewinds the note pointer back to the beginning of the song.
     */
    public void rewind()
    {
        final String funcName = "rewind";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        sequenceIndex = 0;
        currSection = null;
        currVolume = startVolume;
    }   //rewind

    /**
     * This method finds the section with the specified section name.
     *
     * @param sectionName specifies the section name to look for.
     * @return section object associated with the specified name.
     */
    private Section findSection(String sectionName)
    {
        final String funcName = "findSection";
        Section section = null;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC, "name=%s", sectionName);
        }

        for (Section s: sections)
        {
            if (sectionName.equals(s.getName()))
            {
                section = s;
                break;
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC, "=%s",
                               section == null? "null": section.getName());
        }

        return section;
    }   //findSection

}   //TrcSong
