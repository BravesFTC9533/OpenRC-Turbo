package org.firstinspires.ftc.teamcode.adultbot.hardware.servos;

import org.firstinspires.ftc.teamcode.adultbot.util.EVConverters;
import org.firstinspires.ftc.teamcode.adultbot.util.FileUtil;
import org.firstinspires.ftc.teamcode.adultbot.util.InputExtractor;
import org.firstinspires.ftc.teamcode.adultbot.util.file.OptionsFile;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class Servos {
    private final Map<ServoName, ServoControl> servoMap;

    public Servos(Map<ServoName, ServoControl> servoMap) {
        this.servoMap = servoMap;
    }

    /**
     * causes the servo positions to advance
     *
     * @return false if any servo is not done
     */
    public boolean act() {
        boolean isDone = true;
        for (Map.Entry<ServoName, ServoControl> entry : servoMap.entrySet()) {
            ServoControl servoControl = entry.getValue();

            if (!servoControl.act()) {
                isDone = false;
            }
        }
        return isDone;
    }

    /**
     * @return false if any servo is not done
     */
    public boolean areDone() {
        for (Map.Entry<ServoName, ServoControl> entry : servoMap.entrySet()) {
            ServoControl servoControl = entry.getValue();

            if (!servoControl.isDone()) {
                return false;
            }
        }
        return true;
    }

    /**
     * @return a Map that links the names of the servos to the servos
     */
    public Map<ServoName, ServoControl> getServoMap() {
        return servoMap;
    }

    public List<ServoName> getServoNames() {
        return new ArrayList<>(servoMap.keySet());
    }

    /**
     * Store servo positions to a file
     *
     * @param filename the name of the file
     */
    public void storeServoPositions(String filename) {
        OptionsFile servoPos = new OptionsFile(EVConverters.getInstance());
        for (Map.Entry<ServoName, ServoControl> entry : servoMap.entrySet()) {
            ServoName servoName = entry.getKey();
            ServoControl servoControl = entry.getValue();

            servoPos.set(servoName.name(), servoControl.getCurrentPosition());
        }
        //servoPos.add("time", System.currentTimeMillis());
        servoPos.writeToFile(FileUtil.getOptionsFile(filename));
//        servoPos.writeToFile(FileUtil.getConfigsFile(servoMap.entrySet().iterator().next().getKey().getRobotCfg(), filename));
    }

    /**
     * Load the servo positions from a file
     *
     * @param filename the name of the file
     */
    public void loadServoPositions(String filename) {
        OptionsFile servoPos = new OptionsFile(EVConverters.getInstance(), FileUtil.getOptionsFile(filename));

        for (Map.Entry<ServoName, ServoControl> entry : servoMap.entrySet()) {
            ServoName servoName = entry.getKey();
            ServoControl servoControl = entry.getValue();

            try {
                Double position = servoPos.get(servoName.name(), Double.class);
                servoControl.setPosition(position);
            } catch (IllegalArgumentException ignored) {

            }

        }
    }

    public InputExtractor<Double> servoIE(final ServoName servoName) {
        return new InputExtractor<Double>() {
            @Override
            public Double getValue() {
                return servoMap.get(servoName).getCurrentPosition();
            }
        };
    }
}
