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

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Scanner;

/**
 * This class implements a notated song in formatted strings.
 */
public class TrcSong
{
    private static final String moduleName = "TrcSong";
    private static final boolean debugEnabled = false;
    private TrcDbgTrace dbgTrace = null;

    /**
     * This enum class lists all the song parser state.
     */
    private enum ParseState
    {
        //
        // The parser is not in any valid state.
        //
        None,
        //
        // The parser is parsing a song section.
        //
        Section,
        //
        // The parser is parsing the sequences of the song.
        //
        Song
    }   //enum ParseState

    private static final String sectionTag = "section";
    private static final String songTag = "song";

    private String[] songSections = null;
    private int[] sectionSequences = null;
    private int sequenceIndex = 0;

    /**
     * Constructor: Create an instance of this object.
     *
     * @param instanceName specifies the instance name.
     * @param input specifies the input stream from which the notated song is read.
     */
    public TrcSong(String instanceName, InputStream input)
    {
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(
                    moduleName + "." + instanceName,
                    false,
                    TrcDbgTrace.TraceLevel.API,
                    TrcDbgTrace.MsgLevel.INFO);
        }

        ParseState parseState = ParseState.None;
        Scanner inFile = new Scanner(input);
        ArrayList<String> sectionNames = new ArrayList<>();
        ArrayList<String> sections = new ArrayList<>();
        ArrayList<String> sequences = new ArrayList<>();
        String sectionName = null;
        String songSection = null;

        while (inFile.hasNextLine())
        {
            String line = inFile.nextLine().trim();
            //
            // Skip any blank lines.
            //
            if (line.length() == 0) continue;
            //
            // Parse a tag.
            //
            if (line.charAt(0) == '<' && line.charAt(line.length() - 1) == '>')
            {
                String tag = line.substring(1, line.length() - 1).trim();

                if (parseState == ParseState.None && tag.startsWith(sectionTag + "="))
                {
                    //
                    // Beginning of a song section.
                    //
                    sectionName = tag.substring(sectionTag.length() + 1);
                    songSection = "";
                    parseState = ParseState.Section;
                }
                else if (parseState == ParseState.Section && tag.equals("/" + sectionTag))
                {
                    //
                    // End of a song section.
                    //
                    sectionNames.add(sectionName);
                    sections.add(songSection);
                    parseState = ParseState.None;
                }
                else if (parseState == ParseState.None && tag.startsWith(songTag))
                {
                    //
                    // Beginning of the song sequence.
                    //
                    parseState = ParseState.Song;
                }
                else if (parseState == ParseState.Song && tag.equals("/" + songTag))
                {
                    //
                    // End of the song sequence.
                    //
                    parseState = ParseState.None;
                }
            }
            else if (parseState == ParseState.Section)
            {
                songSection += line;
            }
            else if (parseState == ParseState.Song)
            {
                sequences.add(line);
            }
            else
            {
                throw new RuntimeException("Invalid data <" + line + ">");
            }
        }
        inFile.close();
        //
        // Build the song sections array.
        //
        songSections = new String[sections.size()];
        for (int i = 0; i < songSections.length; i++)
        {
            songSections[i] = sections.get(i);
        }
        //
        // Build the section sequences array.
        //
        sectionSequences = new int[sequences.size()];
        for (int i = 0; i < sectionSequences.length; i++)
        {
            sectionSequences[i] = findSection(sectionNames, sequences.get(i));
            if (sectionSequences[i] == -1)
            {
                throw new RuntimeException("Section name " + sequences.get(i) + " not found.");
            }
        }
    }   //TrcSong

    /**
     * Constructor: Create an instance of this object.
     *
     * @param fileName specifies the file name from which the notated song is read.
     * @throws FileNotFoundException if the specified file name is not found.
     */
    public TrcSong(String fileName) throws FileNotFoundException
    {
        this(fileName, new FileInputStream(fileName));
    }   //TrcSong

    /**
     * Constructor: Create an instance of this object.
     *
     * @param instanceName specifies the instance name.
     * @param songSections specifies the song sections array.
     * @param sectionSequences specifies the section sequences array.
     */
    public TrcSong(String instanceName, String[] songSections, int[]sectionSequences)
    {
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(
                    moduleName + "." + instanceName,
                    false,
                    TrcDbgTrace.TraceLevel.API,
                    TrcDbgTrace.MsgLevel.INFO);
        }

        if (sectionSequences == null)
        {
            sectionSequences = new int[songSections.length];
            for (int i = 0; i < sectionSequences.length; i++)
            {
                sectionSequences[i] = i;
            }
        }

        this.songSections = songSections;
        this.sectionSequences = sectionSequences;
    }   //TrcSong

    /**
     * Constructor: Create an instance of this object.
     *
     * @param instanceName specifies the instance name.
     * @param songSections specifies the song sections array.
     */
    public TrcSong(String instanceName, String[] songSections)
    {
        this(instanceName, songSections, null);
    }   //TrcSong

    /**
     * This method returns the next song section to be played.
     *
     * @return the next song section, null if no more sections.
     */
    public String getNextSection()
    {
        final String funcName = "getNextSection";
        String section = null;
        int sectionIndex = -1;

        if (sequenceIndex < sectionSequences.length)
        {
            sectionIndex = sectionSequences[sequenceIndex];
            section = songSections[sectionIndex];
            sequenceIndex++;
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%d", sectionIndex);
        }

        return section;
    }   //getNextSection

    /**
     * This method resets the song back to the beginning.
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
    }   //rewind

    /**
     * This method looks up the section name and returns its section index.
     *
     * @param sectionNames specifies the section names array.
     * @param sectionName specifies the section name to look up.
     * @return the section index of the specified name.
     */
    private int findSection(ArrayList<String> sectionNames, String sectionName)
    {
        final String funcName = "findSection";
        int index = -1;

        for (int i = 0; i < sectionNames.size(); i++)
        {
            if (sectionName.equals(sectionNames.get(i)))
            {
                index = i;
                break;
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC, "section=%s", sectionName);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC, "=%d", index);
        }

        return index;
    }   //findSection

}   //class TrcSong
