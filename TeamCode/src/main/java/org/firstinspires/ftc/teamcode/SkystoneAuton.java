package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;

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

import java.util.ArrayList;
import java.util.Collections;

import static org.firstinspires.ftc.teamcode.SkystoneTeleOp.*;

@Autonomous(name = "Skystone Auton")
public class SkystoneAuton extends LinearOpMode {

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
        initSharedPreferences();

        while (!this.isStarted()) {
            telemetry.addData("Status", "Waiting for start");
            telemetry.addData("Runtime", runtime.toString());
            telemetry.addData("IMU calibration status", imu.getCalibrationStatus().toString());
            telemetry.update();
        }

        // ----- RUN SECTION -----

        mainAuton();
        // angleHoldingTest();
        // angleForcingTest1();
        // angleForcingTest2();

        while (opModeIsActive()) {
            telemetry.addData("Runtime", runtime.toString());
            telemetry.update();
            idle();
        }
    }

    // ----- TESTS -----

    public void angleHoldingTest() {
        // holdAngle(0.85, 100, 0);
        // holdAngle(0.85, 300, 0);
        int count = 1;
        for (double i = 0.3; i <= 1; i += 0.1) {
            int angle = (90 * count) % 360;
            holdAngle(i, 80, angle);
            count++;
        }
    }

    public void angleForcingTest1() {
        for (int i = 0; i < 3; i++) {
            gotoDegreesRampingv2(1, 90, true, "cw");
            sleep(3000);
            gotoDegreesRampingv2(1, 0, true, "ccw");
            sleep(3000);
        }
    }

    public void angleForcingTest2() {
        gotoDegreesRampingv2(1, 90);
        sleep(3000);
        gotoDegreesRampingv2(1, 0);
        sleep(3000);
        gotoDegreesRampingv2(1, 90);
        sleep(3000);
        gotoDegreesRampingv2(1, 0);
        sleep(3000);
        gotoDegreesRampingv2(1, 90);
        sleep(3000);
        gotoDegreesRampingv2(1, 180);
        sleep(3000);
        gotoDegreesRampingv2(1, 270);
        sleep(3000);
        gotoDegreesRampingv2(1, 0);
        sleep(3000);
    }

    // ----- META-METHODS -----

    public void mainAuton() {
        if (autonType == AutonType.TWOSKYSTONES) {
            bothBlocksAuton();
        }
        else if (autonType == AutonType.FOUNDATION) {
            plateAuton();
        }
    }

    public void plateAuton() {

        double bigpow = 0.85;
        double medpow = 0.6;
        double littlepow = 0.1;

        plateServosUp();
        // Strafe sideways to start
        moveCardinal(bigpow, inchesToCm(8), 180);

        // Move forwarads and get plate
        moveCardinal(bigpow, inchesToCm(26), 90);
        moveCardinal(littlepow, inchesToCm(5), 90);
        plateServosDown();
        moveCardinal(littlepow, inchesToCm(1.5), 90);
        sleep(500);

        // Pull plate back
        moveCardinal(medpow, inchesToCm(6), 270);

        // Rotate with plate
        if (allianceColor == allianceColor.BLUE) {
            gotoDegreesRampingv2(medpow, 270);
        }
        else if (allianceColor == allianceColor.RED) {
            gotoDegreesRampingv2(medpow, 90);
        }

        // Push plate towards center of field
        moveCardinal(medpow, inchesToCm(2), 90);

        // Keep rotating
        gotoDegreesRampingv2(medpow, 180);
        if (allianceColor == allianceColor.BLUE) {
            gotoDegreesRampingv2(medpow, 90);
        }
        else if (allianceColor == allianceColor.RED) {
            gotoDegreesRampingv2(medpow, 270);
        }

        // Push plate forwards
        moveCardinal(medpow, inchesToCm(4), 90);

        // Strafe towards wall with plate
        moveCardinal(medpow, inchesToCm(29), 180);
        plateServosUp();

        // Strafe towards wall without plate
        moveCardinal(medpow, inchesToCm(8), 180);

        // Go to center of field
        moveCardinal(medpow, inchesToCm(36), 270);
    }

    public void bothBlocksAuton() {

        double bigpow = 0.85;
        double medpow = 0.7;
        double pow = 0.35;
        double tinypow = 0.2;

        moveCardinal(medpow, inchesToCm(26), 270);

        ArrayList values = new ArrayList<Double>();

        // Scan blocks
        for (int i = 0; i < 3; i++) {
            sleep(500);
            values.add(getHSV()[2]);
            if (i < 2) {
                moveCardinal(pow, inchesToCm(8), 180);
            }
        }

        int minIndex = values.indexOf(Collections.min(values));

        telemetry.addData("Values", values);
        telemetry.addData("Min Index", minIndex);
        telemetry.update();

        // Move back to the first black block
        double extraDistOne = 0;
        moveCardinal(pow, inchesToCm(extraDistOne + (values.size() - minIndex - 1) * 8), 0);

        double blockGetPart1Dist = 2;
        double blockGetPart2Dist = 3.5; // This used to be 3
        double backupDistance = 7.5; // TODO: THIS WAS GREAT ON BLUE AT 5
        /*
        double backupDistance;
        if (allianceColor == AllianceColor.BLUE) {
            backupDistance = 5;
        }
        else if (allianceColor == AllianceColor.RED) {
            backupDistance = 7.5;
        }
        */

        double middleDistance = 23;
        double otherSideDistance1 = 18; // This used to be 21
        double otherSideDistance2 = 15; // This used to be 18
        double totalOtherSideDistance1 = middleDistance + otherSideDistance1;
        double totalOtherSideDistance2 = middleDistance + otherSideDistance2;

        double blockSize = 8;

        // Get block and back up
        moveCardinal(tinypow, inchesToCm(blockGetPart1Dist), 270);
        deploySkystoneGrabber();
        moveCardinal(tinypow, inchesToCm(blockGetPart2Dist), 270);
        moveCardinal(bigpow, inchesToCm(blockGetPart1Dist + blockGetPart2Dist + backupDistance), 90);

        // Go to other side of field and release block
        moveCardinal(bigpow, inchesToCm(totalOtherSideDistance1 + minIndex * blockSize), 0);
        retractSkystoneGrabber();

        // double extraDistTwo = 1;
        double extraDistTwo = 0;

        // Go back to other block
        moveCardinal(bigpow, inchesToCm(totalOtherSideDistance1 + (3 + minIndex) * blockSize + extraDistTwo), 180);
        moveCardinal(bigpow, inchesToCm(backupDistance), 270);

        // Get block and back up
        moveCardinal(tinypow, inchesToCm(blockGetPart1Dist), 270);
        deploySkystoneGrabber();
        moveCardinal(tinypow, inchesToCm(blockGetPart2Dist), 270);
        moveCardinal(bigpow, inchesToCm(blockGetPart1Dist + blockGetPart2Dist + backupDistance), 90);

        // Go to other side of field and release block
        moveCardinal(bigpow, inchesToCm(totalOtherSideDistance2 + (3 + minIndex) * blockSize + extraDistTwo), 0);
        retractSkystoneGrabber();

        // Park
        moveCardinal(bigpow, inchesToCm(otherSideDistance2), 180);
    }

    // ----- ANDROID SHARED PREFERENCES -----

    private static SharedPreferences sharedPrefs;

    private static AllianceColor allianceColor;
    private static AutonType autonType;

    public void initSharedPreferences() {
        sharedPrefs = PreferenceManager.getDefaultSharedPreferences(this.hardwareMap.appContext);

        String color = sharedPrefs.getString("alliance_color", "ERROR");
        String type = sharedPrefs.getString("auton_type", "ERROR");

        if (color.equals("BLUE")) {
            allianceColor = AllianceColor.BLUE;
        }
        else if (color.equals("RED")) {
            allianceColor = AllianceColor.RED;
        }

        if (type.equals("TWOSKYSTONES")) {
            autonType = AutonType.TWOSKYSTONES;
        }
        else if (type.equals("FOUNDATION")) {
            autonType = AutonType.FOUNDATION;
        }
    }

    private enum AllianceColor {
        BLUE, RED
    }

    private enum AutonType {
        TWOSKYSTONES, FOUNDATION
    }

    // ----- SERVO STUFF -----

    private Servo blockServoLeft;
    private Servo blockServoRight;
    private Servo plateServoLeft;
    private Servo plateServoRight;
    private Servo autonGrabberLeft;
    private Servo autonGrabberRight;

    public void initServos() {
        blockServoLeft = hardwareMap.servo.get("blockServoLeft");
        blockServoRight = hardwareMap.servo.get("blockServoRight");
        plateServoLeft = hardwareMap.servo.get("plateServoLeft");
        plateServoRight = hardwareMap.servo.get("plateServoRight");
        autonGrabberLeft = hardwareMap.servo.get("autonGrabberLeft");
        autonGrabberRight = hardwareMap.servo.get("autonGrabberRight");

        retractBothSkystoneGrabbers();
        plateServosDown();
        blockServoJamOpen();
    }

    private void blockServoJamOpen() {
        blockServoLeft.setPosition(blockServoLeftAutonPosition);
        blockServoRight.setPosition(blockServoRightAutonPosition);
    }

    private void plateServosUp() {
        plateServoLeft.setPosition(plateServoLeftUp);
        plateServoRight.setPosition(plateServoRightUp);
    }

    private void plateServosDown() {
        plateServoLeft.setPosition(plateServoLeftDown);
        plateServoRight.setPosition(plateServoRightDown);
    }

    public void retractBothSkystoneGrabbers() {
        autonGrabberLeft.setPosition(autonGrabberLeftPassive);
        autonGrabberRight.setPosition(autonGrabberRightPassive);
    }

    public void deploySkystoneGrabber() {
        if (allianceColor == AllianceColor.BLUE) {
            autonGrabberLeft.setPosition(autonGrabberLeftActive);
        }
        else if (allianceColor == AllianceColor.RED) {
            autonGrabberRight.setPosition(autonGrabberRightActive);
        }
    }

    public void retractSkystoneGrabber() {
        if (allianceColor == AllianceColor.BLUE) {
            autonGrabberLeft.setPosition(autonGrabberLeftPassive);
        }
        else if (allianceColor == AllianceColor.RED) {
            autonGrabberRight.setPosition(autonGrabberRightPassive);
        }
    }

    // ----- COLOR SENSOR STUFF -----

    public ColorSensor leftColorColorSensor;
    public DistanceSensor leftColorDistanceSensor;

    public ColorSensor rightColorColorSensor;
    public DistanceSensor rightColorDistanceSensor;

    public void initColorSensors() {
        leftColorColorSensor = hardwareMap.get(ColorSensor.class, "leftColorSensor");
        leftColorDistanceSensor = hardwareMap.get(DistanceSensor.class, "leftColorSensor");

        rightColorColorSensor = hardwareMap.get(ColorSensor.class, "rightColorSensor");
        rightColorDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightColorSensor");
    }

    public float[] getHSV() {

        // This couldn't possibly be bad design
        DistanceSensor distanceSensor = null;
        ColorSensor colorSensor = null;

        if (allianceColor == AllianceColor.BLUE) {
            distanceSensor = leftColorDistanceSensor;
            colorSensor = leftColorColorSensor;
        }
        else if (allianceColor == AllianceColor.RED) {
            distanceSensor = rightColorDistanceSensor;
            colorSensor = rightColorColorSensor;
        }

        double dist = distanceSensor.getDistance(DistanceUnit.CM);

        double a = colorSensor.alpha();
        double r = colorSensor.red();
        double g = colorSensor.green();
        double b = colorSensor.blue();

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

    public DistanceSensor leftDistanceSensor;
    public DistanceSensor rightDistanceSensor;

    public DistanceUnit distanceUnit = DistanceUnit.CM;

    public void initDistanceSensors() {
        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");
    }

    public double getLeftDistance() {
        return leftDistanceSensor.getDistance(distanceUnit);
    }

    public double getRightDistance() {
        return rightDistanceSensor.getDistance(distanceUnit);
    }

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

    // ----- MOTOR STUFF -----

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

    // ----- LINEAR MOVEMENT -----

    public void moveCardinal(double power, double distance, int direction) {

        holdAngle(power, distance, direction);

        /*
        // Empirically determine a reasonable number of rampup ticks for the given power
        double rampupTicks = 600 * power;
        moveCardinal(power, distance, direction, true, rampupTicks);
        */
    }

    public void moveCardinal(double power, double distance, int direction, boolean ramping, double rampupTicks) {

        if (allianceColor == AllianceColor.RED) {
            direction = (int) reflectAngle(direction);
        }

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

        // Empirically determined values for strafing sideways
        if (direction == 0 || direction == 180) {
            distance *= ((304.8) / (304.8 - 19));
        }

        // Used to be Math.sqrt(2) to make max speed consistent with movement at 45 degrees,
        // but we aborted that plan, giving us a higher possible cardinal max speed
        double maxPower = 1;

        // Distance things
        double targetTicks = distanceToEncoderTicks(distance);

        int averageMotorTicks = 0;

        if (targetTicks / 2 < rampupTicks) {
            rampupTicks = targetTicks / 2;
        }

        double motorPower = 0;

        while (averageMotorTicks < targetTicks && !this.isStopRequested()) {

            int lft = Math.abs(lfMotor.getCurrentPosition());
            int rft = Math.abs(rfMotor.getCurrentPosition());
            int lbt = Math.abs(lbMotor.getCurrentPosition());
            int rbt = Math.abs(rbMotor.getCurrentPosition());
            averageMotorTicks = (lft + rft + lbt + rbt) / 4;

            double powerOffsetStart = 0.08;
            double powerOffsetEnd = 0.08;

            if (ramping) {
                power = Math.max(power, Math.max(powerOffsetEnd, powerOffsetStart));
            }

            if (ramping && averageMotorTicks < rampupTicks) {
                motorPower = powerOffsetStart + (power - powerOffsetStart) * (averageMotorTicks / rampupTicks);
                // motorPower = power * (powerOffsetStart + (1 - powerOffsetStart) * (averageMotorTicks / rampupTicks));
            }
            else if (ramping && averageMotorTicks > targetTicks - rampupTicks) {
                motorPower = powerOffsetEnd + (power - powerOffsetEnd) * (targetTicks - averageMotorTicks) / rampupTicks;
                // motorPower = power * (powerOffsetEnd + (1 - powerOffsetEnd) * (targetTicks - averageMotorTicks) / rampupTicks);
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

    // ----- ROTATIONAL MOVEMENT -----

    public void makeStraight() {
        double mult90 = getNearestMultipleOf90();
        double power = Math.max(0.1, getAngleDifference(getIMUAngleConverted(), mult90) / 50);
        gotoDegreesRamping(power, mult90, true);
    }

    public void gotoDegreesRamping(double power, double targetAngle) {
        gotoDegreesRamping(power, targetAngle, false);
    }

    public void gotoDegreesRamping(double power, double targetAngle, boolean useTimeout) {

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
        double angleTolerance = 0.5;

        if (Math.abs(signedAngleDifference) < abortTolerance) {
            return;
        }

        double timeoutTime = 2.5;

        ElapsedTime timeoutTimer = new ElapsedTime();
        timeoutTimer.reset(); // Probably not necessary

        while (Math.abs(signedAngleDifference) > angleTolerance && !isStopRequested()) {

            // Could've just &&-ed this into the loop condition, but whatever, really
            if (useTimeout && timeoutTimer.seconds() > timeoutTime) {
                break;
            }

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

    public void gotoDegreesRampingv2(double maxPower, double targetAngle) {
        gotoDegreesRampingv2(maxPower, targetAngle, false, "");
    }

    public void gotoDegreesRampingv2(double maxPower, double targetAngle, boolean forceDirection, String direction) {

        // Direction is either "ccw" or "cw", else don't force direction

        double angleTolerance = 2;
        double timeUnderToleance = 200;
        boolean inTolerance = false;

        ElapsedTime timer = new ElapsedTime();

        while (!isStopRequested()) {

            double tmpAngle = getIMUAngleConverted();
            double angle = getAngleDifference(tmpAngle, targetAngle);
            double angleDir = getAngleDifferenceInDirection(direction, tmpAngle, targetAngle);

            if (forceDirection) {
                if (Math.abs(angleDir) >= 180) {
                    angle = angleDir;
                }
            }

            if (Math.abs(angle) < angleTolerance) {
                if (!inTolerance) {
                    timer.reset();
                }
                inTolerance = true;
                if (timer.milliseconds() > timeUnderToleance) {
                    break;
                }
            }
            else {
                inTolerance = false;
            }

            double k = 0.04;
            double correction = angle * k;

            double motorPower = Math.min(correction, maxPower);

            lfMotor.setPower(motorPower);
            rfMotor.setPower(motorPower);
            lbMotor.setPower(motorPower);
            rbMotor.setPower(motorPower);
        }
        setAllMotorPowers(0);
    }

    // ----- ANGLE UTILS -----

    public double getNearestMultipleOf90() {
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

        return potentialValues[minIndex];
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

    public double getAngleDifferenceInDirection(String direction, double currentAngle, double targetAngle) {

        currentAngle = currentAngle % 360;
        targetAngle = targetAngle % 360;

        currentAngle = currentAngle < 0 ? currentAngle + 360 : currentAngle;
        targetAngle = targetAngle < 0 ? targetAngle + 360 : targetAngle;

        double angleDiff = targetAngle - currentAngle;

        if (direction.equals("ccw")) {
            if (angleDiff > 0) {
                return angleDiff;
            }
            else {
                return (360 - currentAngle) + targetAngle;
            }
        }
        else {
            if (angleDiff < 0) {
                return angleDiff;
            }
            else {
                return (targetAngle - 360) - currentAngle;
            }
        }
    }

    public static double reflectAngle(double angle) {
        angle = angle % 360;
        angle = angle < 0 ? angle + 360 : angle;
        if (0 <= angle && angle < 180) {
            return (180 - angle) % 360;
        }
        else if (180 <= angle && angle <= 360) {
            return (540 - angle) % 360;
        }
        else {
            return angle;
        }
    }

    // ----- LINEAR MOVEMENT WITH ANGLE CORRECTING -----

    public void holdAngle(double power, double distance, int direction) {

        if (allianceColor == AllianceColor.RED) {
            direction = (int) reflectAngle(direction);
        }

        double angleHold = getNearestMultipleOf90();

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
            throw new RuntimeException("Direction must be -1 or 1");
        }

        if (direction == 0 || direction == 180) {
            distance *= ((304.8) / (304.8 - 19));
        }

        double targetTicks = distanceToEncoderTicks(distance);
        int averageMotorTicks = 0;

        boolean useRamping = true;
        double rampupTicks = 500 * power;
        if (targetTicks / 2 < rampupTicks) {
            rampupTicks = targetTicks / 2;
        }

        while (averageMotorTicks < targetTicks && !isStopRequested()) {

            int lft = Math.abs(lfMotor.getCurrentPosition());
            int rft = Math.abs(rfMotor.getCurrentPosition());
            int lbt = Math.abs(lbMotor.getCurrentPosition());
            int rbt = Math.abs(rbMotor.getCurrentPosition());
            averageMotorTicks = (lft + rft + lbt + rbt) / 4;

            double motorPower = power;

            if (useRamping) {
                double powerOffsetStart = 0.05;
                double powerOffsetEnd = 0.05;

                power = Math.max(power, Math.max(powerOffsetEnd, powerOffsetStart));

                if (averageMotorTicks < rampupTicks) {
                    motorPower = powerOffsetStart + (power - powerOffsetStart) * (averageMotorTicks / rampupTicks);
                }
                else if (averageMotorTicks > targetTicks - rampupTicks) {
                    motorPower = powerOffsetEnd + (power - powerOffsetEnd) * (targetTicks - averageMotorTicks) / rampupTicks;
                }
                else {
                    motorPower = power;
                }
            }

            double tmpAngle = getIMUAngleConverted();
            double angle = getAngleDifference(tmpAngle, angleHold);

            double k = 0.04;
            double correction = angle * k;

            double[] motorPowers = {
                motorPower * motorDirections[0],
                motorPower * motorDirections[1],
                motorPower * motorDirections[2],
                motorPower * motorDirections[3],
            };

            double minPower = getMin(motorPowers);
            double maxPower = getMax(motorPowers);

            double tempMax = Math.max(Math.abs(minPower), Math.abs(maxPower));
            double tempCorrectedMax = Math.max(Math.abs(minPower + correction), Math.abs(maxPower + correction));

            if (tempCorrectedMax > 1) {
                for (int i = 0; i < motorPowers.length; i++) {
                    motorPowers[i] *= (1 - Math.abs(correction)) / tempMax;
                }
            }

            for (int i = 0; i < motorPowers.length; i++) {
                motorPowers[i] += correction;
            }

            lfMotor.setPower(motorPowers[0]);
            rfMotor.setPower(motorPowers[1]);
            lbMotor.setPower(motorPowers[2]);
            rbMotor.setPower(motorPowers[3]);

            /*
            telemetry.addData("Angle", angle);
            telemetry.addData("Correction", correction);
            telemetry.update();
             */
        }

        setAllMotorPowers(0);
    }

    public class SimpleTimer {

        private long startTime;

        public SimpleTimer() {
            reset();
        }

        public void reset() {
            startTime = System.nanoTime();
        }

        public double getElapsedNanoseconds() {
            return System.nanoTime() - startTime;
        }

        public double getElapsedSeconds() {
            return getElapsedNanoseconds() / Math.pow(10, 9);
        }

    }

    public double getMin(double[] arr) {
        double min = arr[0];
        for (int i = 1; i < arr.length; i++) {
            if (arr[i] < min) {
                min = arr[i];
            }
        }
        return min;
    }

    public double getMax(double[] arr) {
        double max = arr[0];
        for (int i = 1; i < arr.length; i++) {
            if (arr[i] > max) {
                max = arr[i];
            }
        }
        return max;
    }
}
