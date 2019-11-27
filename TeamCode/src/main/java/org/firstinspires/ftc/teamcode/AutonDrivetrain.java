package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutonDrivetrain {

    // TODO: Ramp up/down speed for smoothness
    // To maintain direction, ratio of motor speeds must stay the same

    // TODO: May want a fake "async" version of dist, angle with a function that runs a loop iteration
    // (The same may be true of moveAlongWall)

    // TODO: Potentially convert power -> ticks/s -> cm/s
    // Thus, could specify a velocity instead of a power
    // Power is nice, though, because is scales 0 to 1

    public static final double GEAR_RATIO = 1; // output/input teeth
    public static final int TICKS_PER_MOTOR_ROTATION = 1120;
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
            motorDirections[0] = -1;
            motorDirections[1] = -1;
        }
        else if (direction == 90) {
            motorDirections[0] = -1;
            motorDirections[2] = -1;
        }
        else if (direction == 180) {
            motorDirections[2] = -1;
            motorDirections[3] = -1;
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
            int rft = Math.abs(rfMotor.getCurrentPosition());
            int lbt = Math.abs(lbMotor.getCurrentPosition());
            int rbt = Math.abs(rbMotor.getCurrentPosition());
            averageMotorTicks = (lft + rft + lbt + rbt) / 4;

            telemetry.addLine("Target Ticks: " + targetTicks);
            telemetry.addLine("LF Actual: " + lft);
            telemetry.addLine("RF Actual: " + rft);
            telemetry.addLine("LB Actual: " + lbt);
            telemetry.addLine("RB Actual: " + rbt);
            telemetry.update();
        }

        setAllMotorPowers(0);
    }

    // Synchronous movement
    public void moveDistance(double power, double distance, double angle) {

        resetAllEncoders();

        angle = Math.toRadians(angle);

        double x = Math.cos(angle);
        double y = Math.sin(angle);

        double powerLF = -x - y;
        double powerRF = -x + y;
        double powerLB = x - y;
        double powerRB = x + y;

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

        /*
        double targetLFTicks = targetTicks * (powerLF / power);
        double targetRFTicks = targetTicks * (powerRF / power);
        double targetLBTicks = targetTicks * (powerLB / power);
        double targetRBTicks = targetTicks * (powerRB / power);
        */

        int targetLFTicks = (int) ((targetTicks * (powerLF / power)) + 0.5);
        int targetRFTicks = (int) ((targetTicks * (powerRF / power)) + 0.5);
        int targetLBTicks = (int) ((targetTicks * (powerLB / power)) + 0.5);
        int targetRBTicks = (int) ((targetTicks * (powerRB / power)) + 0.5);

        /*
        int[] targetTicksArray = {
                (int) (targetLFTicks + 0.5),
                (int) (targetRFTicks + 0.5),
                (int) (targetLBTicks + 0.5),
                (int) (targetRBTicks + 0.5)
        };
         */

        boolean reachedTarget = false;

        boolean lfStopped = false;
        boolean rfStopped = false;
        boolean lbStopped = false;
        boolean rbStopped = false;

        int lft, rft, lbt, rbt;

        while (!reachedTarget) {

            lft = lfMotor.getCurrentPosition();
            rft = rfMotor.getCurrentPosition();
            lbt = lbMotor.getCurrentPosition();
            rbt = rbMotor.getCurrentPosition();

            if (Math.abs(lft) >= Math.abs(targetLFTicks)) {
                lfMotor.setPower(0);
                lfStopped = true;
            }

            if (Math.abs(rft) >= Math.abs(targetRFTicks)) {
                rfMotor.setPower(0);
                rfStopped = true;
            }

            if (Math.abs(lbt) >= Math.abs(targetLBTicks)) {
                lbMotor.setPower(0);
                lbStopped = true;
            }

            if (Math.abs(rbt) >= Math.abs(targetRBTicks)) {
                rbMotor.setPower(0);
                rbStopped = true;
            }

            if (lfStopped && rfStopped && lbStopped && rbStopped) {
                reachedTarget = true;
            }

            /*
            lft = Math.abs(lft);
            rft = Math.abs(rft);
            lbt = Math.abs(lbt);
            rbt = Math.abs(rbt);

            int[] currentTicksArray = {
                    lft,
                    rft,
                    lbt,
                    rbt
            };

            boolean[] successes = {
                    lft > targetLFTicks && targetLFTicks > 0,
                    rft > targetRFTicks && targetRFTicks > 0,
                    lbt > targetLBTicks && targetLBTicks > 0,
                    rbt > targetRBTicks && targetRBTicks > 0
            };

            int count = 0;
            for (boolean bool : successes) {
                count += bool ? 1 : 0;
            }

            if (count >= 2) {
                reachedTarget = true;
            }

            int count = 0;
            for (int i = 0; i < 4; i++) {
                int target = targetTicksArray[i];
                int current = currentTicksArray[i];
                if (target >= 0) {
                    if (current >= 0.95 * target) {
                        count++;
                    }
                }
                else {
                    if (current <= 0.95 * target) {
                        count++;
                    }
                }
            }
            if (count == 4) {
                reachedTarget = true;
            }
            */

            telemetry.addLine("LF Target: " + targetLFTicks + " LF Actual: " + lft);
            telemetry.addLine("RF Target: " + targetRFTicks + " RF Actual: " + rft);
            telemetry.addLine("LB Target: " + targetLBTicks + " LB Actual: " + lbt);
            telemetry.addLine("RB Target: " + targetRBTicks + " RB Actual: " + rbt);
            telemetry.update();
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

    Telemetry telemetry;

    public AutonDrivetrain(HardwareMap hardwareMap, Telemetry telemetry) {
        lfMotor = hardwareMap.dcMotor.get("lfMotor");
        rfMotor = hardwareMap.dcMotor.get("rfMotor");
        lbMotor = hardwareMap.dcMotor.get("lbMotor");
        rbMotor = hardwareMap.dcMotor.get("rbMotor");
        setAllRunModes(DcMotor.RunMode.RUN_USING_ENCODER);
        setAllZeroPowerBehaviors(DcMotor.ZeroPowerBehavior.BRAKE);
        this.telemetry = telemetry;
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
