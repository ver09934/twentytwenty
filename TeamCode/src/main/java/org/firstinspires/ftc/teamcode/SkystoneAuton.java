package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.*;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Skystone Auton")
public class SkystoneAuton extends LinearOpMode {

    // TODO: May want to make the ramping floor an absolute of 0.1, instead of 0.1 * input_power
    // Could do a quick Math.max(0.1, ramp), but that's a bit messy...
    // Could do something like 0.1 + power * ramp, but then full power needs to be
    // set to 0.1 + power, which is bad...

    private ElapsedTime runtime;

    @Override
    public void runOpMode() {

        // ----- INIT SECTION -----

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        runtime = new ElapsedTime();

        initMotors();
        initServos();
        initColorSensors();
        initIMU();

        while (!this.isStarted()) {
            telemetry.addData("Status", "Waiting for start");
            telemetry.addData("Runtime", runtime.toString());
            telemetry.addData("IMU calibration status", imu.getCalibrationStatus().toString());
            telemetry.update();
        }

        // ----- RUN SECTION -----

        bothBlocksAuton();

        while (opModeIsActive()) {
            idle();
        }
    }

    // ----- META-METHODS -----

    double bigpow = 1.2;
    double pow = 0.5;
    double tinypow = 0.3;

    public void plateAuton() {}

    public void bothBlocksAuton() {

        moveCardinalRamping(bigpow, inchesToCm(27), 270);

        ArrayList values = new ArrayList<Double>();

        // Scan blocks
        for (int i = 0; i < 3; i++) {

            values.add(getLeftHSV()[2]);

            if (i < 2) {
                moveCardinalRamping(pow, inchesToCm(8), 180);
            }
        }

        int minIndex = values.indexOf(Collections.min(values));

        telemetry.addData("Values", values);
        telemetry.addData("Min Index", minIndex);
        telemetry.update();

        // Move back to the first black block
        double extraDistOne = 0;
        moveCardinalRamping(pow, inchesToCm(extraDistOne + (values.size() - minIndex - 1) * 8), 0);

        double blockGetPart1Dist = 1;
        double blockGetPart2Dist = 2;
        double backupDistance = 10;

        double middleDistance = 20;
        double otherSideDistance = 15;
        double totalOtherSideDistance = middleDistance + otherSideDistance;

        double blockDistance = 8;

        // Get block and back up
        moveCardinalRamping(tinypow, inchesToCm(blockGetPart1Dist), 270);
        autonGrabberLeft.setPosition(AUTON_GRABBER_LEFT_ACTIVE);
        moveCardinalRamping(tinypow, inchesToCm(blockGetPart2Dist), 270);
        moveCardinalRamping(bigpow, inchesToCm(blockGetPart1Dist + blockGetPart2Dist + backupDistance), 90);

        // Go to other side of field and release block
        moveCardinalRamping(bigpow, inchesToCm(totalOtherSideDistance + minIndex * blockDistance), 0);
        autonGrabberLeft.setPosition(AUTON_GRABBER_LEFT_PASSIVE);

        makeStraight();

        double extraDistTwo = 1.5;

        // Go back to other block
        moveCardinalRamping(bigpow, inchesToCm(totalOtherSideDistance + (3 + minIndex) * blockDistance + extraDistTwo), 180);
        moveCardinalRamping(bigpow, inchesToCm(backupDistance), 270);

        // Get block and back up
        moveCardinalRamping(tinypow, inchesToCm(blockGetPart1Dist), 270);
        autonGrabberLeft.setPosition(AUTON_GRABBER_LEFT_ACTIVE);
        moveCardinalRamping(tinypow, inchesToCm(blockGetPart2Dist), 270);
        moveCardinalRamping(bigpow, inchesToCm(blockGetPart1Dist + blockGetPart2Dist + backupDistance), 90);

        // Go to other side of field and release block
        moveCardinalRamping(bigpow, inchesToCm(totalOtherSideDistance + 3 * blockDistance + extraDistTwo), 0);
        autonGrabberLeft.setPosition(AUTON_GRABBER_LEFT_PASSIVE);

        // Park
        moveCardinalRamping(bigpow, inchesToCm(otherSideDistance), 180);
    }

    // ----- TEST META-METHODS -----

    public void testMoveOne() {
        for (int j = 0; j < 1; j++) {
            for (int i = 0; i < 360; i += 90) {
                gotoDegreesRamping(0.5, i);
                sleep(1000);
            }
        }

        while (opModeIsActive()) {
            // makeStraight(0.5);
            makeStraight();
            sleep(5000);
        }
    }

    public void testMoveTwo() {
        for (int i = 0; i < 360; i += 90) {
            moveCardinalRamping(1, inchesToCm(30), i);
            sleep(1000);
        }

        for (int i = 0; i < 360; i += 90) {
            moveCardinalRamping(0.5, inchesToCm(12), i);
            sleep(1000);
        }

        for (int i = 0; i < 360; i += 90) {
            moveCardinalRamping(0.3, inchesToCm(4), i);
            sleep(1000);
        }
    }

    // ----- SERVO STUFF -----

    public Servo autonGrabberLeft;
    public Servo blockServo;

    public double AUTON_GRABBER_LEFT_PASSIVE = 0.74;
    public double AUTON_GRABBER_LEFT_ACTIVE = 0.04;

    private double blockServoClosedPosition = 0;
    private double blockServoOpenPosition = 0.5;

    public void initServos() {
        autonGrabberLeft = hardwareMap.servo.get("autonGrabberLeft");
        autonGrabberLeft.setPosition(AUTON_GRABBER_LEFT_PASSIVE);

        blockServo = hardwareMap.servo.get("testServo");
        // blockServo.setPosition(blockServoOpenPosition);
    }

    // ----- COLOR SENSOR STUFF -----

    public ColorSensor leftColorColorSensor;
    public DistanceSensor leftColorDistanceSensor;

    public void initColorSensors() {
        leftColorColorSensor = hardwareMap.get(ColorSensor.class, "leftColorSensor");
        leftColorDistanceSensor = hardwareMap.get(DistanceSensor.class, "leftColorSensor");
    }

    public float[] getLeftHSV() {

        double dist = leftColorDistanceSensor.getDistance(DistanceUnit.CM);

        double a = leftColorColorSensor.alpha();
        double r = leftColorColorSensor.red();
        double g = leftColorColorSensor.green();
        double b = leftColorColorSensor.blue();

        float hsvValues[] = {0F, 0F, 0F};
        final double SCALE_FACTOR = 255;

        Color.RGBToHSV(
                (int) (r * SCALE_FACTOR),
                (int) (g * SCALE_FACTOR),
                (int) (b * SCALE_FACTOR),
                hsvValues
        );

        return hsvValues;
    }

    // ----- DISTANCE SENSOR STUFF -----

    public DistanceSensor frontDistanceSensor;
    public DistanceSensor leftDistanceSensor;
    public DistanceSensor rightDistanceSensor;

    public DistanceUnit distanceUnit = DistanceUnit.CM;

    public void initDistanceSensors() {
        frontDistanceSensor = hardwareMap.get(DistanceSensor.class, "frontDistanceSensor");
        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");
    }

    // frontDistanceSensor.getDistance(distanceUnit);

    // ----- IMU STUFF -----

    public BNO055IMU imu;

    public void initIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Status", "Calibrating IMU");
        telemetry.update();

        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
    }

    public double getIMUAngleConverted() {
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double angle = orientation.firstAngle;
        angle =  angle < 0 ? angle + 360 : angle;
        return angle;
    }

    // ----- MOTOR STUFF -----

    private DcMotor lfMotor;
    private DcMotor rfMotor;
    private DcMotor lbMotor;
    private DcMotor rbMotor;

    public void initMotors() {
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

    // ----- DRIVING STUFF -----

    public static final double GEAR_RATIO = 1; // output/input teeth
    public static final int TICKS_PER_MOTOR_ROTATION = 1120;
    public static final int TICKS_PER_WHEEL_ROTATION = (int)((TICKS_PER_MOTOR_ROTATION * GEAR_RATIO) + 0.5);
    public static final double WHEEL_DIAMETER = 10.16; // [cm]
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER; // [cm]
    public static final double DISTANCE_PER_TICK = WHEEL_CIRCUMFERENCE / TICKS_PER_WHEEL_ROTATION;

    public static int distanceToEncoderTicks(double distance) {
        return (int)((distance / DISTANCE_PER_TICK) + 0.5);
    }

    public static double inchesToCm(double inches) {
        return inches * 2.54;
    }

    public void moveCardinal(double power, double distance, int direction, boolean stopMotors) {

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

        if (direction == 0 || direction == 180) {
            // Empirically determined values for strafing sideways
            distance *= ((304.8) / (304.8 - 19));
        }

        // TODO: See if needed
        double maxPower = Math.sqrt(2);

        lfMotor.setPower(power * motorDirections[0] / maxPower);
        rfMotor.setPower(power * motorDirections[1] / maxPower);
        lbMotor.setPower(power * motorDirections[2] / maxPower);
        rbMotor.setPower(power * motorDirections[3] / maxPower);

        double targetTicks = distanceToEncoderTicks(distance);

        int averageMotorTicks = 0;

        while (averageMotorTicks < targetTicks && !this.isStopRequested()) {
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

        if (stopMotors) {
            setAllMotorPowers(0);
        }
    }

    public void moveCardinalRamping(double power, double distance, int direction) {

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

        if (direction == 0 || direction == 180) {
            // Empirically determined values for strafing sideways
            distance *= ((304.8) / (304.8 - 19));
        }

        // TODO: See if needed
        double maxPower = Math.sqrt(2);
        // double maxPower = 1;

        // Distance things
        double targetTicks = distanceToEncoderTicks(distance);

        int averageMotorTicks = 0;

        // TODO: Empirical, see if this is good
        double rampupTicks = 800 * power;

        if (targetTicks / 2 < rampupTicks) {
            rampupTicks = targetTicks / 2;
        }

        // ElapsedTime rampupTimer = new ElapsedTime();

        double motorPower = 0;

        while (averageMotorTicks < targetTicks && !this.isStopRequested()) {

            // double currentTime = rampupTimer.time(TimeUnit.SECONDS);

            int lft = Math.abs(lfMotor.getCurrentPosition());
            int rft = Math.abs(rfMotor.getCurrentPosition());
            int lbt = Math.abs(lbMotor.getCurrentPosition());
            int rbt = Math.abs(rbMotor.getCurrentPosition());
            averageMotorTicks = (lft + rft + lbt + rbt) / 4;

            double powerOffsetStart = 0.1;
            double powerOffsetEnd = 0.1;

            power = Math.max(power, Math.max(powerOffsetEnd, powerOffsetStart));

            if (averageMotorTicks < rampupTicks) {
                // motorPower = power * (powerOffsetStart + (1 - powerOffsetStart) * (averageMotorTicks / rampupTicks));
                motorPower = powerOffsetStart + (power - powerOffsetStart) * (averageMotorTicks / rampupTicks);
            }
            else if (averageMotorTicks > targetTicks - rampupTicks) {
                // motorPower = power * (powerOffsetEnd + (1 - powerOffsetEnd) * (targetTicks - averageMotorTicks) / rampupTicks);
                motorPower = powerOffsetEnd + (power - powerOffsetEnd) * (targetTicks - averageMotorTicks) / rampupTicks;
            }
            else {
                motorPower = power;
            }

            lfMotor.setPower(motorPower * motorDirections[0] / maxPower);
            rfMotor.setPower(motorPower * motorDirections[1] / maxPower);
            lbMotor.setPower(motorPower * motorDirections[2] / maxPower);
            rbMotor.setPower(motorPower * motorDirections[3] / maxPower);

            telemetry.addLine("Target Ticks: " + targetTicks);
            telemetry.addLine("LF Actual: " + lft);
            telemetry.addLine("RF Actual: " + rft);
            telemetry.addLine("LB Actual: " + lbt);
            telemetry.addLine("RB Actual: " + rbt);
            telemetry.update();
        }

        setAllMotorPowers(0);
    }

    public void turnDegrees(double turnPower, double angleDelta) {

        assert Math.abs(angleDelta) <= 180;

        double startAngle = getIMUAngleConverted();

        double targetAngle = startAngle + angleDelta;

        if (targetAngle > 360) {
            targetAngle = targetAngle % 360;
        }
        else if (targetAngle < 0) {
            targetAngle = targetAngle % 360;
            targetAngle += 360;
        }

        turnPower = Math.copySign(turnPower, angleDelta);

        lfMotor.setPower(turnPower);
        rfMotor.setPower(turnPower);
        lbMotor.setPower(turnPower);
        rbMotor.setPower(turnPower);

        double currentAngle = startAngle;

        if (angleDelta > 0 && targetAngle < startAngle) {
            while (currentAngle >= startAngle || currentAngle < targetAngle && !isStopRequested()) {
                currentAngle = getIMUAngleConverted();
                telemetry.addLine("Delta: " + angleDelta + " Target: " + targetAngle + " Current: " + currentAngle);
                telemetry.update();
            }
        }
        else if (angleDelta < 0 && targetAngle > startAngle) {
            while (currentAngle <= startAngle || currentAngle > targetAngle && !isStopRequested()) {
                currentAngle = getIMUAngleConverted();
                telemetry.addLine("Delta: " + angleDelta + " Target: " + targetAngle + " Current: " + currentAngle);
                telemetry.update();
            }
        }
        else if (angleDelta > 0) {
            while (currentAngle < targetAngle && !isStopRequested()) {
                currentAngle = getIMUAngleConverted();
                telemetry.addLine("Delta: " + angleDelta + " Target: " + targetAngle + " Current: " + currentAngle);
                telemetry.update();
            }
        }
        else {
            while (currentAngle > targetAngle && !isStopRequested()) {
                currentAngle = getIMUAngleConverted();
                telemetry.addLine("Delta: " + angleDelta + " Target: " + targetAngle + " Current: " + currentAngle);
                telemetry.update();
            }
        }

        setAllMotorPowers(0);
    }

    // public void makeStraight(double power) {
    public void makeStraight() {

        double currentAngle = getIMUAngleConverted();

        double[] potentialValues = {0, 90, 180, 270};

        double[] diffs = new double[potentialValues.length];

        for (int i = 0; i < potentialValues.length; i++) {
            diffs[i] = Math.abs(getAngleDifference(currentAngle, potentialValues[i]));
        }

        ArrayList<Double> diffsArrayList = new ArrayList<>();
        for(double diff : diffs) {
            diffsArrayList.add(diff);
        }

        double min = Collections.min(diffsArrayList);
        int minIndex = diffsArrayList.indexOf(min);

        double empiricalValue = 60;
        double power = Math.max(0.1, min / 60);

        gotoDegreesRamping(power, potentialValues[minIndex]);
    }

    public void gotoDegreesRamping(double power, double targetAngle) {

        double startAngle = getIMUAngleConverted();
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
        double angleTolerance = 0;

        if (Math.abs(signedAngleDifference) < abortTolerance) {
            return;
        }

        while (Math.abs(signedAngleDifference) > angleTolerance && !isStopRequested()) {

            currentAngle = getIMUAngleConverted();

            signedAngleDifference = getAngleDifference(currentAngle, targetAngle);
            absAngleDifference = Math.abs(signedAngleDifference);
            absAngleProgress = Math.abs(getAngleDifference(startAngle, currentAngle));

            double powerOffsetStart = 0.1;
            double powerOffsetEnd = 0.1;

            double motorPower;

            if (absAngleProgress < rampupAngle) {
                motorPower = power * (powerOffsetStart + (1 - powerOffsetStart) * (absAngleProgress / rampupAngle));
            }
            else if (absAngleDifference < rampupAngle) {
                motorPower = power * (powerOffsetEnd + (1 - powerOffsetEnd) * (absAngleDifference / rampupAngle));
            }
            else {
                motorPower = power;
            }

            motorPower = Math.copySign(motorPower, signedAngleDifference);

            setAllMotorPowers(motorPower);
        }

        setAllMotorPowers(0);
    }

    public static double getAngleDifference(double currentAngle, double targetAngle) {

        currentAngle = currentAngle % 360;
        targetAngle = targetAngle % 360;

        currentAngle = currentAngle < 0 ? currentAngle + 360 : currentAngle;
        targetAngle = targetAngle < 0 ? targetAngle + 360 : targetAngle;

        double angleDiff = targetAngle - currentAngle;

        if (Math.abs(angleDiff) <= 180) {
            return angleDiff;
        }
        else {
            if (angleDiff > 0) {
                return angleDiff - 360;
            }
            else {
                return 360 + angleDiff;
            }
        }
    }

}
