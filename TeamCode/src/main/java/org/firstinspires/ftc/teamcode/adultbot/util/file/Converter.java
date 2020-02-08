package org.firstinspires.ftc.teamcode.adultbot.util.file;

public interface Converter<T> {
    /**
     * Convert an object to a String
     *
     * @param object the object
     * @return a String representing the object
     */
    String toString(T object);

    /**
     * Convert a String to an object
     *
     * @param string the String
     * @return the object
     */
    T fromString(String string);
}