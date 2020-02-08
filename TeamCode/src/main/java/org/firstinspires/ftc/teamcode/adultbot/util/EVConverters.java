package org.firstinspires.ftc.teamcode.adultbot.util;

import org.firstinspires.ftc.teamcode.adultbot.util.file.Converters;
import org.firstinspires.ftc.teamcode.adultbot.util.file.UtilConverters;

public class EVConverters extends UtilConverters {

    //    static {
//        BasicConverters.converterMap.put(BeaconColorResult.BeaconColor.class, new Converter<BeaconColorResult.BeaconColor>() {
//
//            @Override
//            public String toString(BeaconColorResult.BeaconColor object) {
//                return object.name();
//            }
//
//            @Override
//            public BeaconColorResult.BeaconColor fromString(String string) {
//                return BeaconColorResult.BeaconColor.valueOf(string);
//            }
//        });
//        BasicConverters.converterMap.put(BeaconColorResult.class, new Converter<BeaconColorResult>() {
//
//            @Override
//            public String toString(BeaconColorResult object) {
//                return object.toString();
//            }
//
//            @Override
//            public BeaconColorResult fromString(String string) {
//                String[] parts = string.split(" *\\| *");
//                if (parts.length != 2) return null;
//                return new BeaconColorResult(BeaconColorResult.BeaconColor.valueOf(parts[0]), BeaconColorResult.BeaconColor.valueOf(parts[1]));
//            }
//        });
//        BasicConverters.converterMap.put(BeaconName.class, new Converter<BeaconName>() {
//
//            @Override
//            public String toString(BeaconName object) {
//                return object.name();
//            }
//
//            @Override
//            public BeaconName fromString(String string) {
//                return BeaconName.valueOf(string);
//            }
//        });
//        BasicConverters.converterMap.put(byte[].class, new Converter<byte[]>() {
//
//            @Override
//            public String toString(byte[] object) {
//                return BaseEncoding.base64Url().encode(object);
//            }
//
//            @Override
//            public byte[] fromString(String string) {
//                return BaseEncoding.base64Url().decode(string);
//            }
//        });
//    }
//
    private static final Converters INSTANCE = new EVConverters();

    public static Converters getInstance() {
        return INSTANCE;
    }

    protected EVConverters() {
        super();
    }
}
