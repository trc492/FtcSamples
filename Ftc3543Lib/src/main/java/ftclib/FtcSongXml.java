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

package ftclib;

import android.util.Xml;

import org.xmlpull.v1.XmlPullParser;
import org.xmlpull.v1.XmlPullParserException;

import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Stack;

import trclib.TrcDbgTrace;
import trclib.TrcSong;

/**
 * This class implements a parser of notated song in an xml file.
 */
public class FtcSongXml
{
    private static final String moduleName = "FtcSongXml";
    private static final boolean debugEnabled = false;
    private TrcDbgTrace dbgTrace = null;

    private String instanceName;
    private XmlPullParser parser = null;
    private ArrayList<TrcSong> collection = new ArrayList<>();

    /**
     * Constructor: Create an instance of this object.
     *
     * @param instanceName specifies the instance name.
     * @param input specifies the input stream from which the notated song is read.
     */
    public FtcSongXml(String instanceName, InputStream input) throws XmlPullParserException, IOException
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
        try
        {
            parser = Xml.newPullParser();
            parser.setFeature(XmlPullParser.FEATURE_PROCESS_NAMESPACES, false);
            parser.setInput(input, null);
            parser.nextTag();
            parseCollection();
        }
        finally
        {
            input.close();
        }
    }   //FtcSongXml

    /**
     * This method returns the song name.
     *
     * @return instance name.
     */
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method returns the number of songs in the collection xml.
     *
     * @return number of songs in the collection.
     */
    public int getNumSongs()
    {
        final String funcName = "getNumSongs";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%d", collection.size());
        }

        return collection.size();
    }   //getNumSongs

    /**
     * This method returns the name of the song with the specified song index in the collection.
     *
     * @param index specifies the song index in the collection.
     * @return song name.
     */
    public String getSongName(int index)
    {
        final String funcName = "getSongName";
        String name = collection.get(index).toString();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "index=%d", index);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", name);
        }

        return name;
    }   //getSongName

    /**
     * The method returns the song from the collection with the specified song index.
     *
     * @param index specifies the song index in the collection.
     * @return song in the collection with the specified index.
     */
    public TrcSong getSong(int index)
    {
        final String funcName = "getSong";
        TrcSong song = collection.get(index);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "index=%d", index);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", song.toString());
        }

        return song;
    }   //getSong

    /**
     * The method returns the song from the collection with the specified song name.
     *
     * @param name specifies the song name to look for..
     * @return song in the collection with the specified name, null if none found.
     */
    public TrcSong getSong(String name)
    {
        final String funcName = "getSong";
        TrcSong song = null;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "name=%s", name);
        }

        for (int i = 0; i < collection.size(); i++)
        {
            song = collection.get(i);
            if (name.equals(song.toString()))
            {
                break;
            }
            else
            {
                song = null;
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", song == null? "null": song.toString());
        }

        return song;
    }   //getSong

    /**
     * The method returns the song collection parsed from the XML file.
     *
     * @return song collection.
     */
    public TrcSong[] getCollection()
    {
        final String funcName = "getCollection";
        TrcSong[] songs = collection.toArray(new TrcSong[collection.size()]);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=(numSongs=%d)", songs.length);
        }

        return songs;
    }   //getCollection

    /**
     * This method parses the song collection in the XML file. The collection starts with a collection tag and
     * contains multiple song tags.
     *
     * @throws XmlPullParserException
     * @throws IOException
     */
    private void parseCollection() throws XmlPullParserException, IOException
    {
        final String funcName = "parseCollection";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC);
        }

        parser.require(XmlPullParser.START_TAG, null, "collection");
        while (parser.next() != XmlPullParser.END_TAG)
        {
            //
            // Skip everything until we find the next start tag.
            //
            if (parser.getEventType() != XmlPullParser.START_TAG)
            {
                continue;
            }

            //
            // Check if the start tag is a song tag. If not, skip the entire tag including the nested tags in it.
            //
            String name = parser.getName();
            if (name.equals("song"))
            {
                collection.add(parseSong());
            }
            else
            {
                skipTag();
            }
        }
        parser.require(XmlPullParser.END_TAG, null, "collection");

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC);
        }
    }   //parseCollection

    /**
     * This method parses the song in the XML file. The song starts with a song tag and contains one sequence tag
     * and multiple section tags.
     *
     * @throws XmlPullParserException
     * @throws IOException
     */
    private TrcSong parseSong() throws XmlPullParserException, IOException
    {
        final String funcName = "parseSong";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC);
        }

        parser.require(XmlPullParser.START_TAG, null, "song");
        TrcSong song = new TrcSong(parser.getAttributeValue(null, "name"));
        while (parser.next() != XmlPullParser.END_TAG)
        {
            //
            // Skip everything until we find the next start tag.
            //
            if (parser.getEventType() != XmlPullParser.START_TAG)
            {
                continue;
            }

            //
            // Check if the start tag is a sequence tag or a section tag. If not, skip the entire tag
            // including the nested tags in it.
            //
            String name = parser.getName();
            if (name.equals("sequence"))
            {
                parseSequence(song);
            }
            else if (name.equals("section"))
            {
                parseSection(song);
            }
            else
            {
                skipTag();
            }
        }
        parser.require(XmlPullParser.END_TAG, null, "song");

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC, "=%s", song.toString());
        }

        return song;
    }   //parseSong

    /**
     * This method parses the song sequence in the XML file. The song sequence starts with a sequence tag and
     * contains a sequence of section names separated by commas.
     *
     * @param song specifies the song object that the sequence belongs to.
     * @throws XmlPullParserException
     * @throws IOException
     */
    private void parseSequence(TrcSong song) throws XmlPullParserException, IOException
    {
        final String funcName = "parseSequence";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC, "song=%s", song.toString());
        }

        parser.require(XmlPullParser.START_TAG, null, "sequence");
        String sequence = "";
        while (parser.next() == XmlPullParser.TEXT)
        {
            sequence += parser.getText();
        }
        song.setSequence(sequence);
        parser.require(XmlPullParser.END_TAG, null, "sequence");

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC, "! (seq=%s)", sequence);
        }
    }   //parseSequence

    /**
     * This method parses the song section in the XML file. The song section starts with a section tag and
     * contains a sequence of notated notes separated by commas.
     *
     * @param song specifies the song object that the section belongs to.
     * @throws XmlPullParserException
     * @throws IOException
     */
    private void parseSection(TrcSong song) throws XmlPullParserException, IOException
    {
        final String funcName = "parseSection";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC, "song=%s", song.toString());
        }

        parser.require(XmlPullParser.START_TAG, null, "section");
        String name = parser.getAttributeValue(null, "name");
        String notation = "";
        while (parser.next() == XmlPullParser.TEXT)
        {
            notation += parser.getText();
        }
        song.addSection(name, notation);
        parser.require(XmlPullParser.END_TAG, null, "section");

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC, "! (%s=%s)", name, notation);
        }
    }   //parseSection

    /**
     * This method parses and skips  section in the XML file. The song section starts with a section tag and
     * contains a sequence of notated notes separated by commas.
     *
     * @throws XmlPullParserException
     * @throws IOException
     */
    private void skipTag() throws XmlPullParserException, IOException
    {
        final String funcName = "skipTag";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC);
        }

        if (parser.getEventType() != XmlPullParser.START_TAG)
        {
            throw new XmlPullParserException("Expected start tag.");
        }

        Stack<String> tagNames = new Stack<>();
        tagNames.push(parser.getName());
        while (!tagNames.empty())
        {
            switch (parser.next())
            {
                case XmlPullParser.END_TAG:
                    String startTag = tagNames.pop();
                    String endTag = parser.getName();
                    if (!endTag.equals(startTag))
                    {
                        throw new XmlPullParserException(
                                "Unmatched end tag <" + endTag + "> (expected <" + startTag + ">)");
                    }
                    break;

                case XmlPullParser.START_TAG:
                    tagNames.push(parser.getName());
                    break;
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC);
        }
    }   //skipTag

}   //class FtcSongXml
