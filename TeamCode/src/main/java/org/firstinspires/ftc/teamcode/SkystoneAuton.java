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

@Autonomous(name = "Skystone Auton")
public class SkystoneAuton extends LinearOpMode {

    // TODO: Figure out how to reverse everything
    // TODO: Read android app settings for field position (check Seth's 2019 commits)
    // TODO: public servo autonGrabberRight
    // TODO: Set block servo to open or wherever...

    // TODO: Add optional direction forcing to angle turn method

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

        if (autonType == AutonType.TWOSKYSTONES) {
            bothBlocksAuton();
        }
        else if (autonType == AutonType.FOUNDATION) {
            plateAuton();
        }

        // sharedPrefsTest();

        while (opModeIsActive()) {
            idle();
        }
    }

    // ----- META-METHODS -----

    public void plateAuton() {

        double bigpow = 0.85;
        double medpow = 0.5;
        double pow = 0.35;

        plateServosUp();

        moveCardinal(bigpow, inchesToCm(29), 90);
        moveCardinal(pow, inchesToCm(2), 90);

        plateServosDown();

        if (allianceColor == allianceColor.BLUE) {
            gotoDegreesRamping(pow, 270);
        }
        else if (allianceColor == allianceColor.RED) {
            gotoDegreesRamping(pow, 90);
        }
        gotoDegreesRamping(pow, 180);

        // moveCardinal(bigpow, inchesToCm(12), 90);

        plateServosUp();

        moveCardinal(pow, inchesToCm(1), 270);

        makeStraight();

        moveCardinal(bigpow, inchesToCm(30), 180);

        makeStraight();

        moveCardinal(bigpow, inchesToCm(30), 90);

        makeStraight();

        moveCardinal(bigpow, inchesToCm(18), 180);
    }

    public void bothBlocksAuton() {

        double bigpow = 0.85;
        double medpow = 0.7;
        double pow = 0.35;
        double tinypow = 0.2;

        moveCardinal(medpow, inchesToCm(27), 270);

        ArrayList values = new ArrayList<Double>();

        // Scan blocks
        for (int i = 0; i < 3; i++) {

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

        double blockGetPart1Dist = 1;
        double blockGetPart2Dist = 3;
        double backupDistance = 10;

        double middleDistance = 20;
        double otherSideDistance = 15;
        double totalOtherSideDistance = middleDistance + otherSideDistance;

        double blockSize = 8;

        // Get block and back up
        moveCardinal(tinypow, inchesToCm(blockGetPart1Dist), 270);
        deploySkystoneGrabber();
        moveCardinal(tinypow, inchesToCm(blockGetPart2Dist), 270);
        moveCardinal(bigpow, inchesToCm(blockGetPart1Dist + blockGetPart2Dist + backupDistance), 90);

        // Go to other side of field and release block
        moveCardinal(bigpow, inchesToCm(totalOtherSideDistance + minIndex * blockSize), 0);
        retractSkystoneGrabber();

        makeStraight(); // Align

        double extraDistTwo = 1;

        // Go back to other block
        moveCardinal(bigpow, inchesToCm(totalOtherSideDistance + (3 + minIndex) * blockSize + extraDistTwo), 180);
        makeStraight(); // Align
        moveCardinal(bigpow, inchesToCm(backupDistance), 270);

        // Get block and back up
        moveCardinal(tinypow, inchesToCm(blockGetPart1Dist), 270);
        deploySkystoneGrabber();
        moveCardinal(tinypow, inchesToCm(blockGetPart2Dist), 270);
        moveCardinal(bigpow, inchesToCm(blockGetPart1Dist + blockGetPart2Dist + backupDistance), 90);

        makeStraight(); // Align

        // Go to other side of field and release block
        moveCardinal(bigpow, inchesToCm(totalOtherSideDistance + (3 + minIndex) * blockSize + extraDistTwo), 0);
        retractSkystoneGrabber();

        // Park
        moveCardinal(bigpow, inchesToCm(otherSideDistance), 180);
    }

    // ----- TEST META-METHODS -----

    public void sharedPrefsTest() {
        if (allianceColor == AllianceColor.BLUE) {
            moveCardinal(0.5, 12, 90);
        }
        else if (allianceColor == AllianceColor.RED) {
            moveCardinal(0.5, 12, 270);
        }

        sleep(500);

        if (autonType == AutonType.TWOSKYSTONES) {
            moveCardinal(0.5, 12, 0);
        }
        else if (autonType == AutonType.FOUNDATION) {
            moveCardinal(0.5, 12, 180);
        }
    }

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
            moveCardinal(1, inchesToCm(30), i);
            sleep(1000);
        }

        for (int i = 0; i < 360; i += 90) {
            moveCardinal(0.5, inchesToCm(12), i);
            sleep(1000);
        }

        for (int i = 0; i < 360; i += 90) {
            moveCardinal(0.3, inchesToCm(4), i);
            sleep(1000);
        }
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

    public Servo autonGrabberLeft;
    public Servo autonGrabberRight;
    public Servo blockServo;

    private Servo plateServoLeft;
    private Servo plateServoRight;

    public double autonGrabberLeftPassive = 0.74;
    public double autonGrabberLeftActive = 0.04;

    public double autonGrabberRightPassive = 0.8;
    public double autonGrabberRightActive = 0.3;

    private double blockServoClosedPosition = 0;
    private double blockServoOpenPosition = 0.5;

    private double plateServoLeftDown = 0.28;
    private double plateServoLeftUp = 0.5;
    private double plateServoRightDown = 0.82;
    private double plateServoRightUp = 0.6;

    public void initServos() {
        autonGrabberLeft = hardwareMap.servo.get("autonGrabberLeft");
        autonGrabberRight = hardwareMap.servo.get("autonGrabberRight");
        retractBothSkystoneGrabbers();

        plateServoLeft = hardwareMap.servo.get("plateServo1");
        plateServoRight = hardwareMap.servo.get("plateServo2");
        plateServosDown();

        blockServo = hardwareMap.servo.get("testServo");
        blockServoJamOpen();
    }

    private void blockServoJamOpen() {
        blockServo.setPosition(blockServoOpenPosition);
    }

    private void blockServoClosed() {
        blockServo.setPosition(blockServoClosedPosition);
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

        // TODO: Add sensor to config
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

    public DistanceSensor frontDistanceSensor;
    public DistanceSensor leftDistanceSensor;
    public DistanceSensor rightDistanceSensor;

    public DistanceUnit distanceUnit = DistanceUnit.CM;

    public void initDistanceSensors() {
        frontDistanceSensor = hardwareMap.get(DistanceSensor.class, "frontDistanceSensor");
        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");
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

        // Empirically determine a reasonable number of rampup ticks for the given power
        double rampupTicks = 600 * power;

        moveCardinal(power, distance, direction, true, rampupTicks);
    }

    public void moveCardinal(double power, double distance, int direction, boolean ramping, double rampupTicks) {

        // TODO
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

        // Empirically set a reasonable motor power for turning
        double power = Math.max(0.1, min / 50);

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
        double angleTolerance = 0.5;

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

    // ----- ANGLE UTILS -----

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
}
