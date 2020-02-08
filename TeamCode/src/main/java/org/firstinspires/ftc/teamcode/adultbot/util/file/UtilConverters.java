package org.firstinspires.ftc.teamcode.adultbot.util.file;

import org.firstinspires.ftc.teamcode.adultbot.util.units.Angle;
import org.firstinspires.ftc.teamcode.adultbot.util.TeamColor;
import org.firstinspires.ftc.teamcode.adultbot.util.Vector2D;
import org.firstinspires.ftc.teamcode.adultbot.util.Vector3D;
import org.firstinspires.ftc.teamcode.adultbot.util.units.AngularVelocity;
import org.firstinspires.ftc.teamcode.adultbot.util.units.Distance;
import org.firstinspires.ftc.teamcode.adultbot.util.units.Time;
import org.firstinspires.ftc.teamcode.adultbot.util.units.Velocity;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class UtilConverters extends BasicConverters {
    private static final String DECIMAL_NUMBER = "([0-9\\.\\-]+)";
    private static final String COMMA = " *, *";
    static {
        converterMap.put(TeamColor.class, new Converter<TeamColor>() {

            @Override
            public String toString(TeamColor object) {
                return object.name();
            }

            @Override
            public TeamColor fromString(String string) {
                return TeamColor.fromString(string);
            }
        });
        converterMap.put(Vector2D.class, new Converter<Vector2D>() {

            @Override
            public String toString(Vector2D object) {
                return object.toString();
            }

            @Override
            public Vector2D fromString(String string) {
                Pattern pattern = Pattern.compile("\\(" + DECIMAL_NUMBER + COMMA + DECIMAL_NUMBER + "\\)");
                Matcher matcher = pattern.matcher(string);
                if (matcher.find()) {
                    return new Vector2D(Double.valueOf(matcher.group(1)), Double.valueOf(matcher.group(2)));
                } else {
                    return null;
                }
            }
        });
        converterMap.put(Vector3D.class, new Converter<Vector3D>() {

            @Override
            public String toString(Vector3D object) {
                return object.toString();
            }

            @Override
            public Vector3D fromString(String string) {
                Pattern pattern = Pattern.compile("\\(" + DECIMAL_NUMBER + COMMA + DECIMAL_NUMBER + COMMA + DECIMAL_NUMBER + "\\)");
                Matcher matcher = pattern.matcher(string);
                if (matcher.find()) {
                    return new Vector3D(Double.valueOf(matcher.group(1)), Double.valueOf(matcher.group(2)), Double.valueOf(matcher.group(3)));
                } else {
                    return null;
                }
            }
        });
        converterMap.put(Angle.class, new Converter<Angle>() {

            @Override
            public String toString(Angle object) {
                return String.valueOf(object.radians());
            }

            @Override
            public Angle fromString(String string) {
                try {
                    return Angle.fromRadians(Double.valueOf(string));
                } catch (NumberFormatException e) {
                    return null;
                }
            }
        });
        converterMap.put(AngularVelocity.class, new Converter<AngularVelocity>() {

            @Override
            public String toString(AngularVelocity object) {
                return String.valueOf(object.radiansPerSecond());
            }

            @Override
            public AngularVelocity fromString(String string) {
                try {
                    return new AngularVelocity(Angle.fromRadians(Double.valueOf(string)), Time.fromSeconds(1));
                } catch (NumberFormatException e) {
                    return null;
                }
            }
        });
        converterMap.put(Distance.class, new Converter<Distance>() {

            @Override
            public String toString(Distance object) {
                return String.valueOf(object.meters());
            }

            @Override
            public Distance fromString(String string) {
                try {
                    return Distance.fromMeters(Double.valueOf(string));
                } catch (NumberFormatException e) {
                    return null;
                }
            }
        });
        converterMap.put(Time.class, new Converter<Time>() {

            @Override
            public String toString(Time object) {
                return String.valueOf(object.seconds());
            }

            @Override
            public Time fromString(String string) {
                try {
                    return Time.fromSeconds(Double.valueOf(string));
                } catch (NumberFormatException e) {
                    return null;
                }
            }
        });
        converterMap.put(Velocity.class, new Converter<Velocity>() {

            @Override
            public String toString(Velocity object) {
                return String.valueOf(object.metersPerSecond());
            }

            @Override
            public Velocity fromString(String string) {
                try {
                    return new Velocity(Distance.fromMeters(Double.valueOf(string)), Time.fromSeconds(1));
                } catch (NumberFormatException e) {
                    return null;
                }
            }
        });
    }

    private static final Converters INSTANCE = new UtilConverters();

    public static Converters getInstance() {
        return INSTANCE;
    }

    protected UtilConverters() {
        super();
    }
}