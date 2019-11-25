package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AutonDrivetrain {

    // TODO: Ramp up/down speed for smoothness
    // To maintain direction, ratio of motor speeds must stay the same

    // TODO: May want a fake "async" version of dist, angle with a function that runs a loop iteration
    // (The same may be true of moveAlongWall)

    // TODO: Potentially convert power -> ticks/s -> cm/s
    // Thus, could specify a velocity instead of a power
    // Power is nice, though, because is scales 0 to 1

    // TODO: Get real values
    // TODO: Add gear ratio - needs to increase predicted ticks for a set distance
    public static final double GEAR_RATIO = 1; // TODO: Value and implement
    public static final int TICKS_PER_ROTATION = 60;
    public static final double WHEEL_DIAMETER = 8; // [cm]
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER; // [cm]
    public static final double DISTANCE_PER_TICK = WHEEL_CIRCUMFERENCE / TICKS_PER_ROTATION;

    // ---------- Movement methods ----------

    // Synchronous movement
    public void moveCardinal(double power, double distance, int direction) {

        resetAllEncoders();

        double targetTicks = distanceToEncoderTicks(distance);

        int[] motorDirections = {1, 1, 1, 1};

        if (direction == 0) {
            motorDirections[2] = -1;
            motorDirections[3] = -1;
        }
        else if (direction == 90) {
            motorDirections[0] = -1;
            motorDirections[2] = -1;
        }
        else if (direction == 180) {
            motorDirections[0] = -1;
            motorDirections[1] = -1;
        }
        else if (direction == 270) {
            motorDirections[1] = -1;
            motorDirections[3] = -1;
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

    // TODO
    // Synchronous movement
    public void moveDistance(double angle, double power, double distance) {

        // TODO: Perhaps methodize repeatable logic

        angle = Math.toRadians(angle);

        double x = Math.cos(angle);
        double y = Math.sin(angle);

        double powerLF = x - y;
        double powerRF = x + y;
        double powerLB = -x - y;
        double powerRB = -x + y;

        // double maxPossiblePower = Math.cos(Math.PI/4) + Math.sin(Math.PI/4);
        double maxPossiblePower = Math.sqrt(2);

        powerLF /= maxPossiblePower;
        powerRF /= maxPossiblePower;
        powerLB /= maxPossiblePower;
        powerRB /= maxPossiblePower;

        powerLF *= power;
        powerRF *= power;
        powerLB *= power;
        powerRB *= power;

        /*
        NOTE:
        The target distance is simply the target velocity times time!
        The target distances are thus simply proportional to the target wheel velocities!
        Target velocities are apparently a simple calculation based on angle!
        Once this is written, moveCardinal will no longer be needed!
        The question is, what is this constant of proportionality?
         */
    }

    // Simply set powers and return
    public void moveContinuous(double angle, double power) {
        // TODO
    }

    public void turn(double angle, double power) {
        // TODO: I doubt this will be fun...
        // This may need to be simply empirical...
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
