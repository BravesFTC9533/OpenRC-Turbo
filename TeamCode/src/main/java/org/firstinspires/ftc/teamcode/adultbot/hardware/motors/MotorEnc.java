package org.firstinspires.ftc.teamcode.adultbot.hardware.motors;


public interface MotorEnc extends Motor {
    /**
     * A PID on the motor controller uses the encoder to regulate the speed of the motor.
     *
     * @param speed value to set the speed to
     */
    void setSpeed(double speed);

    /**
     * A PID on the motor controller uses the encoder to turn the motor to any encoder position.
     *
     * @param encoderTarget      position in encoder ticks to rotate to
     * @param maxCorrectionPower the max power to run the motor at when turning to the position
     */
    void setPosition(int encoderTarget, double maxCorrectionPower);

    /**
     * Set the encoder zero point to the current encoder value
     */
    void resetEncoder();

    /**
     * Get the encoder position relative to the zero value
     *
     * @return the encoder position
     */
    int getEncoderPosition();

}
