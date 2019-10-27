/*
 * Copyright (c) 2019 Titan Robotics Club (http://www.titanrobotics.com)
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

import java.util.HashMap;
import java.util.Locale;

/**
 * This class implements the TrcHashMap. TrcHashMap extends HashMap and added the add method so you can easily
 * append multiple hash map entries in a single statement.
 */
public class TrcHashMap<K, V> extends HashMap<K, V>
{
    /**
     * This method adds an entry to the hash map and returns this hashmap object so you can chain multiple calls to
     * the add method in a single statement.
     *
     * @param key specifies the key of the mapping.
     * @param value specifies the value of the mappng.
     * @return this hashmap instance.
     */
    public TrcHashMap<K, V> add(K key, V value)
    {
        put(key, value);
        return this;
    }   //add

    /**
     * This method overrides the get method of the super class and throws an exception if the key cannot be found
     * in the hashmap.
     *
     * @param key specifies the key of the mapping to look for.
     * @return the value of the mapping found.
     * @throws IllegalArgumentException if key not in hashmap.
     */
    @Override
    public V get(Object key)
    {
        if (!containsKey(key))
        {
            throw new IllegalArgumentException(String.format(Locale.US, "Key \"%s\" not found.", key));
        }

        return super.get(key);
    }   //get

}   //class TrcHashMap
