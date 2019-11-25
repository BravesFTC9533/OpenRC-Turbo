package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Robot;

public class MechDrive extends Drive {

    public final Robot robot;

    public final DcMotorEx fl;
    public final DcMotorEx fr;
    public final DcMotorEx bl;
    public final DcMotorEx br;

    public MechDrive(Robot robot, LinearOpMode opMode) {
        super(opMode);
        this.robot = robot;
        this.fl = robot.fl;
        this.fr = robot.fr;
        this.bl = robot.bl;
        this.br = robot.br;
    }

    // Scale motor power based on the max for all wheels
    // 1, 1, 1, 3 will become .33, .33, .33, 1
    public static double scalePower(double value, double max){
        if(max == 0){return  0;}
        return  value / max;
    }

    public void addTargetPosition(int leftTicks, int rightTicks) {
        fl.setTargetPosition(fl.getCurrentPosition() + leftTicks);
        bl.setTargetPosition(bl.getCurrentPosition() + leftTicks);
        fr.setTargetPosition(fr.getCurrentPosition() + rightTicks);
        br.setTargetPosition(br.getCurrentPosition() + rightTicks);
    }

    @Override
    public void drive(double v, double h, double r) {
        // add vectors
        double frontLeft =  v-h+r;
        double frontRight = v+h-r;
        double backRight =  v-h-r;
        double backLeft =   v+h+r;

        // since adding vectors can go over 1, figure out max to scale other wheels
        double max = Math.max(
                Math.abs(backLeft),
                Math.max(
                        Math.abs(backRight),
                        Math.max(
                                Math.abs(frontLeft), Math.abs(frontRight)
                        )
                )
        );
        // only need to scale power if max > 1
        if(max > 1){
            frontLeft = scalePower(frontLeft, max);
            frontRight = scalePower(frontRight, max);
            backLeft = scalePower(backLeft, max);
            backRight = scalePower(backRight, max);
        }

        fl.setPower(frontLeft);
        fr.setPower(frontRight);
        bl.setPower(backLeft);
        br.setPower(backRight);
    }

    @Override
    public void moveByInches(double power, float inches, double timeoutSeconds) {
        encoderDrive(power, (int) (Robot.COUNTS_PER_INCH * inches), (int) (Robot.COUNTS_PER_INCH * inches), timeoutSeconds);
    }

    @Override
    public void encoderDrive(double power, int leftTicks, int rightTicks, double timeoutSeconds) {
        addTargetPosition(leftTicks, rightTicks);
        robot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive(power, 0, 0);

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        while(opMode.opModeIsActive() && robot.isBusy() && runtime.seconds() <= timeoutSeconds) {}

        stop();
    }

    @Override
    public void stop() {
        drive(0, 0, 0);
    }
}
