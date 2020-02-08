package org.firstinspires.ftc.teamcode.adultbot.util.file;

public interface Converters {
    /**
     * Get a Converter for a certain class
     *
     * @param clazz the class that the converter converts
     * @return the Converter
     */
    <T> Converter<T> getConverter(Class<T> clazz);
}
