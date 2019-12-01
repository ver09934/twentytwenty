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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;

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

        while (!this.isStarted()) {
            telemetry.addData("Status", "Waiting for start");
            telemetry.addData("Runtime", runtime.toString());
            telemetry.addData("IMU calibration status", imu.getCalibrationStatus().toString());
            telemetry.update();
        }

        // ----- RUN SECTION -----

        testMoveTwo();

        while (opModeIsActive()) {
            idle();
        }
    }

    // ----- META-METHODS -----

    public void testMoveTwo() {

        moveCardinal(0.5, inchesToCm(27), 270);

        sleep(1000);

        ArrayList values = new ArrayList<Double>();

        for (int i = 0; i < 3; i++) {

            values.add(getLeftHSV()[2]);

            if (i < 2) {
                moveCardinal(0.5, inchesToCm(8), 180);
            }

            sleep(1000);
        }

        int minIndex = values.indexOf(Collections.min(values));

        telemetry.addData("Values", values);
        telemetry.addData("Min Index", minIndex);
        telemetry.update();

        // moveCardinal(0.5, inchesToCm(8) * (values.size() - minIndex - 1), 0);
        moveCardinal(0.5, inchesToCm(1), 0);
        for (int i = 0; i < values.size() - minIndex - 1; i++) {
            moveCardinal(0.5, inchesToCm(8), 0);
            sleep(1000);
        }
        sleep(1000);
        moveCardinal(0.25, inchesToCm(3), 270);
        sleep(1000);
        autonGrabberLeft.setPosition(AUTON_GRABBER_LEFT_ACTIVE);
        sleep(1000);
        moveCardinal(0.5, inchesToCm(12), 90);

        telemetry.addData("Values", values);
        telemetry.addData("Min Index", minIndex);
        telemetry.update();
    }

    public void testMove() {
        moveCardinal(0.5, inchesToCm(27), 270);
        sleep(1000);
        moveCardinal(0.5, inchesToCm(41), 180);
        sleep(1000);
        moveCardinal(0.5, inchesToCm(2), 270);

        autonGrabberLeft.setPosition(AUTON_GRABBER_LEFT_ACTIVE);

        sleep(3000);

        moveCardinal(0.5, inchesToCm(12), 90);
        sleep(1000);
        moveCardinal(0.5, inchesToCm(12), 0);
        sleep(1000);
        turnDegrees(0.5, 180);
        sleep(500);
        turnDegrees(0.5, -180);
    }

    // ----- SERVO STUFF -----

    public Servo autonGrabberLeft;

    public double AUTON_GRABBER_LEFT_PASSIVE = 0.74;
    public double AUTON_GRABBER_LEFT_ACTIVE = 0.04;

    public void initServos() {
        autonGrabberLeft = hardwareMap.servo.get("autonGrabberLeft");
        autonGrabberLeft.setPosition(AUTON_GRABBER_LEFT_PASSIVE);
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

        setAllMotorPowers(0);
    }

    public void turnDegrees(double turnPower, double angleDelta) {

        assert Math.abs(angleDelta) <= 180;

        double startAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        startAngle = startAngle < 0 ? startAngle + 360 : startAngle;

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

        if (angleDelta > 0 && targetAngle < startAngle && !isStopRequested()) {
            while (currentAngle >= startAngle || currentAngle < targetAngle) {
                currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                currentAngle = currentAngle < 0 ? currentAngle + 360 : currentAngle;
                telemetry.addLine("Delta: " + angleDelta + " Target: " + targetAngle + " Current: " + currentAngle);
                telemetry.update();
            }
        }
        else if (angleDelta < 0 && targetAngle > startAngle && !isStopRequested()) {
            while (currentAngle <= startAngle || currentAngle > targetAngle) {
                currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                currentAngle = currentAngle < 0 ? currentAngle + 360 : currentAngle;
                telemetry.addLine("Delta: " + angleDelta + " Target: " + targetAngle + " Current: " + currentAngle);
                telemetry.update();
            }
        }
        else if (angleDelta > 0) {
            while (currentAngle < targetAngle && !isStopRequested()) {
                currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                currentAngle = currentAngle < 0 ? currentAngle + 360 : currentAngle;
                telemetry.addLine("Delta: " + angleDelta + " Target: " + targetAngle + " Current: " + currentAngle);
                telemetry.update();
            }
        }
        else {
            while (currentAngle > targetAngle && !isStopRequested()) {
                currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                currentAngle = currentAngle < 0 ? currentAngle + 360 : currentAngle;
                telemetry.addLine("Delta: " + angleDelta + " Target: " + targetAngle + " Current: " + currentAngle);
                telemetry.update();
            }
        }

        setAllMotorPowers(0);
    }
}
