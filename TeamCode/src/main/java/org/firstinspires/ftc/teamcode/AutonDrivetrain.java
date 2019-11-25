package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AutonDrivetrain {

    // Auton will implement a move along wall method, maybe

    /*
    3 Goal methods:
    - Move, at angle, distance
        - Sync version (1)
        - Async version (2)
    - Move at angle, speed (3)
        - Async
        - Used in moveAlongWall method, implemented in Auton
     */

    /*
    DC Motor RUN_TO_POSITION Methods:
    isBusy()
    - Careful, may take a long time to settle...
    TODO: DC Motor RUN_USING_ENCODERS Methods:
     */

    // TODO: Improvements
    // - Ramping up/down speed for smoothness

    // TODO: Sync and Async
    public void MoveCardinal(double power, double distance, int direction) {
        // direction: {0, 1, 2, 3} --> {0, 90, 180, 270}
        // Throw error if bad arg, because good code good
        /* Steps:
        1. Set all motors to target power
        2. Do nothing (or sleep) in a while loop that checks whether the average distance each motor
        has traveled is less than the target distance
        3. Set motor powers to zero, optionally
         */
    }

    public void move(double angle, double velocity) {
        // TODO
    }

    public void move(double angle, double velocity, double distance) {
        // TODO
    }

    // TODO: Remove wrapping for simplicity, probably...
    // And then I'll probably end up putting it back in, of course...

    // TODO: Is there any need to pass in telemetry?

    MotorWrapper lfMotor;
    MotorWrapper rfMotor;
    MotorWrapper lbMotor;
    MotorWrapper rbMotor;

    public AutonDrivetrain(HardwareMap hardwareMap) {
        lfMotor = new MotorWrapper(hardwareMap.dcMotor.get("lfMotor"));
        rfMotor = new MotorWrapper(hardwareMap.dcMotor.get("rfMotor"));
        lbMotor = new MotorWrapper(hardwareMap.dcMotor.get("lbMotor"));
        rbMotor = new MotorWrapper(hardwareMap.dcMotor.get("rbMotor"));
    }

    public void setRunMode(DcMotor.RunMode runMode) {
        lfMotor.setMode(runMode);
        rfMotor.setMode(runMode);
        lbMotor.setMode(runMode);
        rbMotor.setMode(runMode);
    }

    public void resetEncoders() {
        lfMotor.resetEncoder();
        rfMotor.resetEncoder();
        lbMotor.resetEncoder();
        rbMotor.resetEncoder();
    }

    public void setZeroPowerBehaviors(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        lfMotor.setZeroPowerBehavior(zeroPowerBehavior);
        rfMotor.setZeroPowerBehavior(zeroPowerBehavior);
        lbMotor.setZeroPowerBehavior(zeroPowerBehavior);
        rbMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setAllPowers(double power) {
        lfMotor.setPower(power);
        rfMotor.setPower(power);
        lbMotor.setPower(power);
        rbMotor.setPower(power);
    }

    public int getLfPosition() {
        return lfMotor.getCurrentPosition();
    }
    public int getRfPosition() {
        return rfMotor.getCurrentPosition();
    }
    public int getLbPosition() {
        return lfMotor.getCurrentPosition();
    }
    public int getRbPosition() {
        return rbMotor.getCurrentPosition();
    }

    class MotorWrapper {

        DcMotor motor;

        MotorWrapper(DcMotor motor) {
            this.motor = motor;
        }

        public void setMode(DcMotor.RunMode runMode) {
            motor.setMode(runMode);
        }

        public void resetEncoder() {
            DcMotor.RunMode currentMode = motor.getMode();
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(currentMode);
        }

        public int getCurrentPosition() {
            return motor.getCurrentPosition();
        }

        public void setPower(double power) {
            motor.setPower(power);
        }

        public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }
}
