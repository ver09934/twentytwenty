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

    public static final double GEAR_RATIO = 1; // output/input teeth
    public static final int TICKS_PER_MOTOR_ROTATION = 280;
    // Round to nearest integer by adding 0.5 and casting to int
    public static final int TICKS_PER_WHEEL_ROTATION = (int)((TICKS_PER_MOTOR_ROTATION * GEAR_RATIO) + 0.5);
    public static final double WHEEL_DIAMETER = 10.16; // [cm]
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER; // [cm]
    public static final double DISTANCE_PER_TICK = WHEEL_CIRCUMFERENCE / TICKS_PER_WHEEL_ROTATION;
    // NOTE: Might it make more sense to do angle per tick?

    // ---------- Distance Utilities ----------

    public static int distanceToEncoderTicks(double distance) {
        // Round to nearest integer by adding 0.5 and casting to int
        return (int)((distance / DISTANCE_PER_TICK) + 0.5);
    }

    public static double encoderTicksToDistance(int encoderTicks) {
        return DISTANCE_PER_TICK * encoderTicks;
    }

    // ---------- Movement methods ----------

    // Synchronous movement
    public void moveCardinal(double power, double distance, int direction) {

        resetAllEncoders();

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

        // TODO: See if needed
        double maxPower = Math.sqrt(2);

        lfMotor.setPower(power * motorDirections[0] / maxPower);
        rfMotor.setPower(power * motorDirections[1] / maxPower);
        lbMotor.setPower(power * motorDirections[2] / maxPower);
        rbMotor.setPower(power * motorDirections[3] / maxPower);

        double targetTicks = distanceToEncoderTicks(distance);

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
    public void moveDistance(double angle, double power, double distance) {

        resetAllEncoders();

        angle = Math.toRadians(angle);

        double x = Math.cos(angle);
        double y = Math.sin(angle);

        double powerLF = x - y;
        double powerRF = x + y;
        double powerLB = -x - y;
        double powerRB = -x + y;

        double maxPossiblePower = Math.cos(Math.PI/4) + Math.sin(Math.PI/4);

        powerLF /= maxPossiblePower;
        powerRF /= maxPossiblePower;
        powerLB /= maxPossiblePower;
        powerRB /= maxPossiblePower;

        powerLF *= power;
        powerRF *= power;
        powerLB *= power;
        powerRB *= power;

        lfMotor.setPower(powerLF);
        rfMotor.setPower(powerRF);
        lbMotor.setPower(powerLB);
        rbMotor.setPower(powerRB);

        // double maxOutputPower = Math.max(Math.max(Math.max(powerLF, powerRF), powerLB), powerRB);

        double targetTicks = distanceToEncoderTicks(distance);

        double targetLFTicks = targetTicks * (powerLF / power);
        double targetRFTicks = targetTicks * (powerRF / power);
        double targetLBTicks = targetTicks * (powerLB / power);
        double targetRBTicks = targetTicks * (powerRB / power);

        targetLFTicks = Math.abs(targetLFTicks);
        targetRFTicks = Math.abs(targetRFTicks);
        targetLBTicks = Math.abs(targetLBTicks);
        targetRBTicks = Math.abs(targetRBTicks);

        boolean reachedTarget = false;

        while (!reachedTarget) {
            int lft = lfMotor.getCurrentPosition();
            int rft = lfMotor.getCurrentPosition();
            int lbt = lfMotor.getCurrentPosition();
            int rbt = lfMotor.getCurrentPosition();

            lft = Math.abs(lft);
            rft = Math.abs(rft);
            lbt = Math.abs(lbt);
            rbt = Math.abs(rbt);

            boolean[] successes = {
                    lft > targetLFTicks,
                    rft > targetRFTicks,
                    lbt > targetLBTicks,
                    rbt > targetRBTicks
            };

            int count = 0;
            for (boolean bool : successes) {
                count += bool ? 1 : 0;
            }

            if (count >= 2) {
                reachedTarget = true;
            }
        }

        setAllMotorPowers(0);

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
        // TODO: This will follow simply from above driving logic
    }

    public void turn(double angle, double power) {
        // TODO: This may need to be simply empirical...
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
}
