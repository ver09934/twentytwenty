package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Motors;

import java.util.ArrayList;
import java.util.Collections;

// ----- LINEAR MOVEMENT -----
public class Movement {
    LinearOpMode op;
    Motors motors;
    Sensors sensors;
    Telemetry telemetry;
    SkystoneAuton.AllianceColor color;

    Movement(LinearOpMode op, Motors motors, SkystoneAuton.AllianceColor color, Sensors sensors) {
        this.op = op;
        this.motors = motors;
        this.sensors = sensors;
        this.telemetry = op.telemetry;
        this.color = color;
    }

    public void moveCardinal(double power, double distance, int direction) {

        // Empirically determine a reasonable number of rampup ticks for the given power
        double rampupTicks = 600 * power;

        moveCardinal(power, distance, direction, true, rampupTicks);
    }

    public void moveCardinal(double power, double distance, int direction, boolean ramping, double rampupTicks) {

        // TODO
        if (color == color.RED) {
            direction = (int) reflectAngle(direction);
        }

        motors.resetAllEncoders();

        int[] motorDirections = {1, 1, 1, 1};

        if (direction == 0) {
            motorDirections[0] = -1;
            motorDirections[1] = -1;
        } else if (direction == 90) {
            motorDirections[0] = -1;
            motorDirections[2] = -1;
        } else if (direction == 180) {
            motorDirections[2] = -1;
            motorDirections[3] = -1;
        } else if (direction == 270) {
            motorDirections[1] = -1;
            motorDirections[3] = -1;
        } else {
            // This is bad, I know
            throw new RuntimeException("Direction not multiple of 90 between 0 and 270, inclusive");
        }

        // Empirically determined values for strafing sideways
        if (direction == 0 || direction == 180) {
            distance *= ((304.8) / (304.8 - 19));
        }

        // Used to be Math.sqrt(2) to make max speed consistent with movement at 45 degrees,
        // but we aborted that plan, giving us a higher possible cardinal max speed
        double maxPower = 1;

        // Distance things
        double targetTicks = motors.distanceToEncoderTicks(distance);

        int averageMotorTicks = 0;

        if (targetTicks / 2 < rampupTicks) {
            rampupTicks = targetTicks / 2;
        }

        double motorPower = 0;

        while (averageMotorTicks < targetTicks && !op.isStopRequested()) {

            int lft = Math.abs(motors.getlfMotor().getCurrentPosition());
            int rft = Math.abs(motors.getrfMotor().getCurrentPosition());
            int lbt = Math.abs(motors.getlbMotor().getCurrentPosition());
            int rbt = Math.abs(motors.getrbMotor().getCurrentPosition());
            averageMotorTicks = (lft + rft + lbt + rbt) / 4;

            double powerOffsetStart = 0.08;
            double powerOffsetEnd = 0.08;

            if (ramping) {
                power = Math.max(power, Math.max(powerOffsetEnd, powerOffsetStart));
            }

            if (ramping && averageMotorTicks < rampupTicks) {
                motorPower = powerOffsetStart + (power - powerOffsetStart) * (averageMotorTicks / rampupTicks);
                // motorPower = power * (powerOffsetStart + (1 - powerOffsetStart) * (averageMotorTicks / rampupTicks));
            } else if (ramping && averageMotorTicks > targetTicks - rampupTicks) {
                motorPower = powerOffsetEnd + (power - powerOffsetEnd) * (targetTicks - averageMotorTicks) / rampupTicks;
                // motorPower = power * (powerOffsetEnd + (1 - powerOffsetEnd) * (targetTicks - averageMotorTicks) / rampupTicks);
            } else {
                motorPower = power;
            }

            motors.getlfMotor().setPower(motorPower * motorDirections[0] / maxPower);
            motors.getrfMotor().setPower(motorPower * motorDirections[1] / maxPower);
            motors.getlbMotor().setPower(motorPower * motorDirections[2] / maxPower);
            motors.getrbMotor().setPower(motorPower * motorDirections[3] / maxPower);

            telemetry.addLine("Target Ticks: " + targetTicks);
            telemetry.addLine("LF Actual: " + lft);
            telemetry.addLine("RF Actual: " + rft);
            telemetry.addLine("LB Actual: " + lbt);
            telemetry.addLine("RB Actual: " + rbt);
            telemetry.update();
        }

        motors.setAllMotorPowers(0);
    }

// ----- ROTATIONAL MOVEMENT -----

    public void makeStraight() {

        double currentAngle = sensors.getIMUAngleConverted();

        double[] potentialValues = {0, 90, 180, 270};

        double[] diffs = new double[potentialValues.length];

        for (int i = 0; i < potentialValues.length; i++) {
            diffs[i] = Math.abs(getAngleDifference(currentAngle, potentialValues[i]));
        }

        ArrayList<Double> diffsArrayList = new ArrayList<>();
        for (double diff : diffs) {
            diffsArrayList.add(diff);
        }

        double min = Collections.min(diffsArrayList);
        int minIndex = diffsArrayList.indexOf(min);

        // Empirically set a reasonable motor power for turning
        double power = Math.max(0.1, min / 50);

        gotoDegreesRamping(power, potentialValues[minIndex], true);
    }

    public void gotoDegreesRamping(double power, double targetAngle) {
        gotoDegreesRamping(power, targetAngle, false);
    }

    public void gotoDegreesRamping(double power, double targetAngle, boolean useTimeout) {

        double startAngle = sensors.getIMUAngleConverted();
        double currentAngle = startAngle;

        double initialAbsAngleDelta = Math.abs(getAngleDifference(startAngle, targetAngle));

        double signedAngleDifference = getAngleDifference(currentAngle, targetAngle);
        double absAngleDifference = Math.abs(signedAngleDifference);
        double absAngleProgress = Math.abs(getAngleDifference(startAngle, currentAngle));

        double rampupAngle = 45;

        if (initialAbsAngleDelta / 2 < rampupAngle) {
            rampupAngle = Math.floor(initialAbsAngleDelta / (double) 2);
        }

        double abortTolerance = 2;
        double angleTolerance = 0.5;

        if (Math.abs(signedAngleDifference) < abortTolerance) {
            return;
        }

        double timeoutTime = 2.5;

        ElapsedTime timeoutTimer = new ElapsedTime();
        timeoutTimer.reset(); // Probably not necessary

        while (Math.abs(signedAngleDifference) > angleTolerance && !op.isStopRequested()) {

            // Could've just &&-ed this into the loop condition, but whatever, really
            if (useTimeout && timeoutTimer.seconds() > timeoutTime) {
                break;
            }

            currentAngle = sensors.getIMUAngleConverted();

            signedAngleDifference = getAngleDifference(currentAngle, targetAngle);
            absAngleDifference = Math.abs(signedAngleDifference);
            absAngleProgress = Math.abs(getAngleDifference(startAngle, currentAngle));

            double powerOffsetStart = 0.1;
            double powerOffsetEnd = 0.1;

            double motorPower;

            if (absAngleProgress < rampupAngle) {
                motorPower = power * (powerOffsetStart + (1 - powerOffsetStart) * (absAngleProgress / rampupAngle));
            } else if (absAngleDifference < rampupAngle) {
                motorPower = power * (powerOffsetEnd + (1 - powerOffsetEnd) * (absAngleDifference / rampupAngle));
            } else {
                motorPower = power;
            }

            motorPower = Math.copySign(motorPower, signedAngleDifference);

            motors.setAllMotorPowers(motorPower);
        }

        motors.setAllMotorPowers(0);
    }

// ----- ANGLE UTILS -----

    public static double getAngleDifference(double currentAngle, double targetAngle) {

        currentAngle = currentAngle % 360;
        targetAngle = targetAngle % 360;

        currentAngle = currentAngle < 0 ? currentAngle + 360 : currentAngle;
        targetAngle = targetAngle < 0 ? targetAngle + 360 : targetAngle;

        double angleDiff = targetAngle - currentAngle;

        if (Math.abs(angleDiff) <= 180) {
            return angleDiff;
        } else {
            if (angleDiff > 0) {
                return angleDiff - 360;
            } else {
                return 360 + angleDiff;
            }
        }
    }

    public static double reflectAngle(double angle) {
        angle = angle % 360;
        angle = angle < 0 ? angle + 360 : angle;
        if (0 <= angle && angle < 180) {
            return (180 - angle) % 360;
        } else if (180 <= angle && angle <= 360) {
            return (540 - angle) % 360;
        } else {
            return angle;
        }
    }
}