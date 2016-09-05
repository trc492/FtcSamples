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

package hallib;

import android.util.Log;

import trclib.TrcDbgTrace;

public class HalDbgLog
{
    private static final String TAG = "TrcDbg";
    /*
    public static final String ESC_PREFIX       = "\u001b[";
    public static final String ESC_SUFFIX       = "m";
    public static final String ESC_SEP          = ";";

    public static final String SGR_RESET        = "0";
    public static final String SGR_BRIGHT       = "1";
    public static final String SGR_DIM          = "2";
    public static final String SGR_ITALIC       = "3";
    public static final String SGR_UNDERLINE    = "4";
    public static final String SGR_BLINKSLOW    = "5";
    public static final String SGR_BLINKFAST    = "6";
    public static final String SGR_REVERSE      = "7";
    public static final String SGR_HIDDEN       = "8";
    public static final String SGR_CROSSEDOUT   = "9";

    public static final String SGR_FG_BLACK     = "30";
    public static final String SGR_FG_RED       = "31";
    public static final String SGR_FG_GREEN     = "32";
    public static final String SGR_FG_YELLOW    = "33";
    public static final String SGR_FG_BLUE      = "34";
    public static final String SGR_FG_MAGENTA   = "35";
    public static final String SGR_FG_CYAN      = "36";
    public static final String SGR_FG_WHITE     = "37";

    public static final String SGR_BG_BLACK     = "40";
    public static final String SGR_BG_RED       = "41";
    public static final String SGR_BG_GREEN     = "42";
    public static final String SGR_BG_YELLOW    = "43";
    public static final String SGR_BG_BLUE      = "44";
    public static final String SGR_BG_MAGENTA   = "45";
    public static final String SGR_BG_CYAN      = "46";
    public static final String SGR_BG_WHITE     = "47";

    public static final String ESC_NORMAL       = ESC_PREFIX
                                                  + ESC_SUFFIX;
    public static final String ESC_BLINKSLOW    = ESC_PREFIX
                                                  + SGR_BLINKSLOW
                                                  + ESC_SUFFIX;
    public static final String ESC_BLINKFAST    = ESC_PREFIX
                                                  + SGR_BLINKFAST
                                                  + ESC_SUFFIX;

    public static final String ESC_FG_BLACK     = ESC_PREFIX
                                                  + SGR_FG_BLACK
                                                  + ESC_SUFFIX;
    public static final String ESC_FG_RED       = ESC_PREFIX
                                                  + SGR_FG_RED
                                                  + ESC_SUFFIX;
    public static final String ESC_FG_GREEN     = ESC_PREFIX
                                                  + SGR_FG_GREEN
                                                  + ESC_SUFFIX;
    public static final String ESC_FG_YELLOW    = ESC_PREFIX
                                                  + SGR_FG_YELLOW
                                                  + ESC_SUFFIX;
    public static final String ESC_FG_BLUE      = ESC_PREFIX
                                                  + SGR_FG_BLUE
                                                  + ESC_SUFFIX;
    public static final String ESC_FG_MAGENTA   = ESC_PREFIX
                                                  + SGR_FG_MAGENTA
                                                  + ESC_SUFFIX;
    public static final String ESC_FG_CYAN      = ESC_PREFIX
                                                  + SGR_FG_CYAN
                                                  + ESC_SUFFIX;
    public static final String ESC_FG_WHITE     = ESC_PREFIX
                                                  + SGR_FG_WHITE
                                                  + ESC_SUFFIX;

    public static final String ESC_BG_BLACK     = ESC_PREFIX
                                                  + SGR_BG_BLACK
                                                  + ESC_SUFFIX;
    public static final String ESC_BG_RED       = ESC_PREFIX
                                                  + SGR_BG_RED
                                                  + ESC_SUFFIX;
    public static final String ESC_BG_GREEN     = ESC_PREFIX
                                                  + SGR_BG_GREEN
                                                  + ESC_SUFFIX;
    public static final String ESC_BG_YELLOW    = ESC_PREFIX
                                                  + SGR_BG_YELLOW
                                                  + ESC_SUFFIX;
    public static final String ESC_BG_BLUE      = ESC_PREFIX
                                                  + SGR_BG_BLUE
                                                  + ESC_SUFFIX;
    public static final String ESC_BG_MAGENTA   = ESC_PREFIX
                                                  + SGR_BG_MAGENTA
                                                  + ESC_SUFFIX;
    public static final String ESC_BG_CYAN      = ESC_PREFIX
                                                  + SGR_BG_CYAN
                                                  + ESC_SUFFIX;
    public static final String ESC_BG_WHITE     = ESC_PREFIX
                                                  + SGR_BG_WHITE
                                                  + ESC_SUFFIX;

    public static final String ESC_FGB_BLACK    = ESC_PREFIX
                                                  + SGR_FG_BLACK
                                                  + ESC_SEP
                                                  + SGR_BRIGHT
                                                  + ESC_SUFFIX;
    public static final String ESC_FGB_RED      = ESC_PREFIX
                                                  + SGR_FG_RED
                                                  + ESC_SEP
                                                  + SGR_BRIGHT
                                                  + ESC_SUFFIX;
    public static final String ESC_FGB_GREEN    = ESC_PREFIX
                                                  + SGR_FG_GREEN
                                                  + ESC_SEP
                                                  + SGR_BRIGHT
                                                  + ESC_SUFFIX;
    public static final String ESC_FGB_YELLOW   = ESC_PREFIX
                                                  + SGR_FG_YELLOW
                                                  + ESC_SEP
                                                  + SGR_BRIGHT
                                                  + ESC_SUFFIX;
    public static final String ESC_FGB_BLUE     = ESC_PREFIX
                                                  + SGR_FG_BLUE
                                                  + ESC_SEP
                                                  + SGR_BRIGHT
                                                  + ESC_SUFFIX;
    public static final String ESC_FGB_MAGENTA  = ESC_PREFIX
                                                  + SGR_FG_MAGENTA
                                                  + ESC_SEP
                                                  + SGR_BRIGHT
                                                  + ESC_SUFFIX;
    public static final String ESC_FGB_CYAN     = ESC_PREFIX
                                                  + SGR_FG_CYAN
                                                  + ESC_SEP
                                                  + SGR_BRIGHT
                                                  + ESC_SUFFIX;
    public static final String ESC_FGB_WHITE    = ESC_PREFIX
                                                  + SGR_FG_WHITE
                                                  + ESC_SEP
                                                  + SGR_BRIGHT
                                                  + ESC_SUFFIX;

    public static final String ESC_BGB_BLACK    = ESC_PREFIX
                                                  + SGR_BG_BLACK
                                                  + ESC_SEP
                                                  + SGR_BRIGHT
                                                  + ESC_SUFFIX;
    public static final String ESC_BGB_RED      = ESC_PREFIX
                                                  + SGR_BG_RED
                                                  + ESC_SEP
                                                  + SGR_BRIGHT
                                                  + ESC_SUFFIX;
    public static final String ESC_BGB_GREEN    = ESC_PREFIX
                                                  + SGR_BG_GREEN
                                                  + ESC_SEP
                                                  + SGR_BRIGHT
                                                  + ESC_SUFFIX;
    public static final String ESC_BGB_YELLOW   = ESC_PREFIX
                                                  + SGR_BG_YELLOW
                                                  + ESC_SEP
                                                  + SGR_BRIGHT
                                                  + ESC_SUFFIX;
    public static final String ESC_BGB_BLUE     = ESC_PREFIX
                                                  + SGR_BG_BLUE
                                                  + ESC_SEP
                                                  + SGR_BRIGHT
                                                  + ESC_SUFFIX;
    public static final String ESC_BGB_MAGENTA  = ESC_PREFIX
                                                  + SGR_BG_MAGENTA
                                                  + ESC_SEP
                                                  + SGR_BRIGHT
                                                  + ESC_SUFFIX;
    public static final String ESC_BGB_CYAN     = ESC_PREFIX
                                                  + SGR_BG_CYAN
                                                  + ESC_SEP
                                                  + SGR_BRIGHT
                                                  + ESC_SUFFIX;
    public static final String ESC_BGB_WHITE    = ESC_PREFIX
                                                  + SGR_BG_WHITE
                                                  + ESC_SEP
                                                  + SGR_BRIGHT
                                                  + ESC_SUFFIX;
    */

    public static void msg(TrcDbgTrace.MsgLevel level, String msg)
    {
        switch (level)
        {
            case FATAL:
            case ERR:
                Log.e(TAG, msg);
                break;

            case WARN:
                Log.w(TAG, msg);
                break;

            case INFO:
                Log.i(TAG, msg);
                break;

            case VERBOSE:
                Log.v(TAG, msg);
                break;
        }
    }   //msg

    public static void traceMsg(String msg)
    {
        Log.d(TAG, msg);
    }   //trace

    public static void tracePrintf(String format, Object... args)
    {
        traceMsg(String.format(format, args));
    }   //tracePrintf

}   //class HalDbgLog
