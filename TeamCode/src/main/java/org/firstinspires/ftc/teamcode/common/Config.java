package org.firstinspires.ftc.teamcode.common;

import android.content.Context;
import android.content.SharedPreferences;

public class Config {

    private static final String PREFERENCES = "RobotPref";
    private SharedPreferences sp;

    public static final String POSITION = "Position";
    public static Position _position;
    public enum Position {
        BLUE_BRICKS, BLUE_BUILDING, RED_BRICKS, RED_BUILDING;

        public static Position toPosition(String position) {
            try {
                return valueOf(position);
            } catch (Exception e) {
                return BLUE_BRICKS;
            }
        }
    }

    public static final String MAX_LIFT_TICKS = "MaxLiftTicks";
    public static int _maxLiftTicks;

    public static final String MAX_SERVO_POSITION = "MaxServoPosition";
    public static float _maxServoPosition;

    public Config(Context context) {
        sp = context.getSharedPreferences(PREFERENCES, Context.MODE_PRIVATE);
        load();
    }


    public void save() {
        SharedPreferences.Editor editor = sp.edit();

        editor.putString(POSITION, _position.name());
        editor.putInt(MAX_LIFT_TICKS, _maxLiftTicks);
        editor.putFloat(MAX_SERVO_POSITION, _maxServoPosition);

        editor.commit();
    }

    public void load() {
        _position = Position.toPosition(sp.getString(POSITION, "BLUE_BRICKS"));
        _maxLiftTicks = sp.getInt(MAX_LIFT_TICKS, 1000);
        _maxServoPosition = sp.getFloat(MAX_SERVO_POSITION, 0.5f);
    }

    public Position getPosition() {
        return _position;
    }

    public void setPosition(Position position) {
        this._position = position;
    }

    public int getMaxLiftTicks() {
        return _maxLiftTicks;
    }

    public void setMaxLiftTicks(int maxLiftTicks) {
        _maxLiftTicks = maxLiftTicks;
    }

    public void setMaxServoPosition(float maxServoPosition) {
        _maxServoPosition = maxServoPosition;
    }

    public float getMaxServoPosition() {
        return _maxServoPosition;
    }
}