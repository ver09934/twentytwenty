package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AutonDrivetrain {

    /*
    TODO:
    3 Goal methods:
    - Move, at angle, distance
        - Sync version (1)
        - Async version (2)
    - Move at angle, speed (3)
        - Async
        - Used in moveAlongWall method, which is implemented in Auton
    - How the "Async" will work:
        - Have a method called runMoveIteration or something like that...
        - Have class variables for state checking that get updated on each loop
        - Alternatively, if all data can be grabbed from motors directly,
        no need for the state variables
     */

    // TODO: Get real values
    public static final int TICKS_PER_ROTATION = 60;
    public static final double WHEEL_DIAMETER = 8; // [cm]
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER; // [cm]
    public static final double DISTANCE_PER_TICK = WHEEL_CIRCUMFERENCE / TICKS_PER_ROTATION;

    // TODO: Ramp up/down speed for smoothness
    // To maintain direction, ratio of motor speeds must stay the same

    // ---------- Movement methods ----------

    /* Steps:
    1. Set all motors to target power
    2. Do nothing (or sleep) in a while loop that checks whether the average distance each motor
    has traveled is less than the target distance
    3. Set motor powers to zero, optionally
     */
    // Sync
    public void MoveCardinal(double power, double distance, int direction) {

        resetAllEncoders();

        double targetTicks = distanceToEncoderTicks(distance);

        int[] motorDirections = {1, 1, 1, 1};

        // TODO
        if (direction == 0) {
            motorDirections[0] = -1;
            motorDirections[2] = -1;
        }
        else if (direction == 90) {
            motorDirections[1] = -1;
            motorDirections[3] = -1;
        }
        else if (direction == 180) {
            motorDirections[0] = -1;
            motorDirections[3] = -1;
        }
        else if (direction == 270) {
            motorDirections[1] = -1;
            motorDirections[4] = -1;
        }
        else {
            // This is bad, I know
            throw new RuntimeException("Direction not multiple of 90 between 0 and 270, inclusive");
        }

        lfMotor.setPower(power * motorDirections[0]);
        rfMotor.setPower(power * motorDirections[1]);
        lbMotor.setPower(power * motorDirections[2]);
        rbMotor.setPower(power * motorDirections[3]);

        int averageMotorTicks = 0;

        while (averageMotorTicks < targetTicks) {
            int lft = Math.abs(lfMotor.getCurrentPosition());
            int rft = Math.abs(lfMotor.getCurrentPosition());
            int lbt = Math.abs(lfMotor.getCurrentPosition());
            int rbt = Math.abs(lfMotor.getCurrentPosition());
            averageMotorTicks = (lft + rft + lbt + rbt) / 4;
        }

        setAllMotorPowers(0);
    }

    // Synchronous movement
    public void move(double angle, double velocity, double distance) {
        // TODO
    }

    // Fake asynchronous movement, with reachedTarget() or runIteration()
    // TODO: Real asynchronous w/ multithreading?
    public void move(double angle, double velocity) {
        // TODO
    }

    // ---------- Individual motor things ----------

    private DcMotor lfMotor;
    private DcMotor rfMotor;
    private DcMotor lbMotor;
    private DcMotor rbMotor;

    public AutonDrivetrain(HardwareMap hardwareMap) {
        lfMotor = hardwareMap.dcMotor.get("lfMotor");
        rfMotor = hardwareMap.dcMotor.get("rfMotor");
        lbMotor = hardwareMap.dcMotor.get("lbMotor");
        rbMotor = hardwareMap.dcMotor.get("rbMotor");
        setAllRunModes(DcMotor.RunMode.RUN_USING_ENCODER);
        setAllZeroPowerBehaviors(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setAllRunModes(DcMotor.RunMode runMode) {
        lfMotor.setMode(runMode);
        rfMotor.setMode(runMode);
        lbMotor.setMode(runMode);
        rbMotor.setMode(runMode);
    }

    public void setAllZeroPowerBehaviors(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        lfMotor.setZeroPowerBehavior(zeroPowerBehavior);
        rfMotor.setZeroPowerBehavior(zeroPowerBehavior);
        lbMotor.setZeroPowerBehavior(zeroPowerBehavior);
        rbMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void resetAllEncoders() {
        setAllRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setAllRunModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setAllMotorPowers(double power) {
        lfMotor.setPower(power);
        rfMotor.setPower(power);
        lbMotor.setPower(power);
        rbMotor.setPower(power);
    }

    // ---------- Other utilities ----------

    public static int distanceToEncoderTicks(double distance) {
        // Round to nearest integer by adding 0.5 and casting to int
        return (int)((distance / DISTANCE_PER_TICK) + 0.5);
    }

    public static double encoderTicksToDistance(int encoderTicks) {
        return DISTANCE_PER_TICK * encoderTicks;
    }
}
