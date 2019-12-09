package org.firstinspires.ftc.teamcode.Tools.Move;

import android.graphics.Color;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.*;

import java.util.ArrayList;
import java.util.Collections;

import static java.lang.Thread.sleep;

public class OnlineMove2020 extends OnlineMove{
    public INITS inits = new INITS();
    public Movements movements = new Movements();

    //Color sensor
    ColorSensor leftColorColorSensor;
    DistanceSensor leftColorDistanceSensor;
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

    //Auton grabber
    double AUTON_GRABBER_LEFT_PASSIVE = 0.74;
    double AUTON_GRABBER_LEFT_ACTIVE = 0.04;
    Servo autonGrabberLeft;

    //IMU
    public BNO055IMU imu_sensor;
    public double getIMUAngleConverted() {
        Orientation orientation = imu_sensor.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double angle = orientation.firstAngle;
        angle =  angle < 0 ? angle + 360 : angle;
        return angle;
    }

    //Normal Distance sensor
    public DistanceSensor frontDistanceSensor;
    public DistanceSensor leftDistanceSensor;
    public DistanceSensor rightDistanceSensor;
    public DistanceUnit distanceUnit = DistanceUnit.CM;
    public static double inchesToCm(double inches) {
        return inches * 2.54;
    }

    public OnlineMove2020(HardwareMap hardwareMap, Telemetry telemetry, ElapsedTime time) {
        super(hardwareMap, telemetry, time);
    }

    public class INITS {
        public void initAll() {
            initMotors();
            initServos();
            initIMU();
            initColorSensors();
            initDistanceSensors();
        }

        public void initMotors() {
            steering.setAllRunModes(DcMotor.RunMode.RUN_USING_ENCODER);
            steering.setAllZeroPowerBehaviors(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public void initServos() {
            autonGrabberLeft = hardwareMap.servo.get("autonGrabberLeft");
            autonGrabberLeft.setPosition(AUTON_GRABBER_LEFT_PASSIVE);
        }

        public void initIMU() {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

            parameters.mode = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled = false;

            imu_sensor = hardwareMap.get(BNO055IMU.class, "imu");
            imu_sensor.initialize(parameters);

            logger.add("Status", "Calibrating IMU", true);
        }

        public void initColorSensors() {
            leftColorColorSensor = hardwareMap.get(ColorSensor.class, "leftColorSensor");
            leftColorDistanceSensor = hardwareMap.get(DistanceSensor.class, "leftColorSensor");
        }

        public void initDistanceSensors() {
            frontDistanceSensor = hardwareMap.get(DistanceSensor.class, "frontDistanceSensor");
            leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
            rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");
        }
    }

    public class Movements {
        public void testMoveThree() {

            for (int i = 0; i < 360; i += 90) {
                moveCardinalRamping(1, inchesToCm(30), i);
                rb_time.sleep(1000);
            }

            for (int i = 0; i < 360; i += 90) {
                moveCardinalRamping(0.5, inchesToCm(12), i);
                rb_time.sleep(1000);
            }

            for (int i = 0; i < 360; i += 90) {
                moveCardinalRamping(0.3, inchesToCm(4), i);
                rb_time.sleep(1000);
            }
        }

        public void testMoveTwo() {

            double hugepow = 0.5;
            double bigpow = 0.5;
            double pow = 0.25;
            double tinypow = 0.1;

            moveCardinal(0.5, inchesToCm(27), 270, true);

            rb_time.sleep(1000);

            ArrayList values = new ArrayList<Double>();

            for (int i = 0; i < 3; i++) {

                values.add(getLeftHSV()[2]);

                if (i < 2) {
                    moveCardinal(pow, inchesToCm(8), 180, true);
                }

                rb_time.sleep(1000);
            }

            int minIndex = values.indexOf(Collections.min(values));
            logger.add("Values", values);
            logger.add("Min Index", minIndex);
            logger.update(true);

            // moveCardinal(pow, inchesToCm(2), 0, true);

            double randDist = 3;
            if (minIndex == 2) {
                moveCardinal(pow, inchesToCm(randDist), 0, true);
            }
            for (int i = 0; i < values.size() - minIndex - 1; i++) {

                if (i == 0) {
                    moveCardinal(pow, inchesToCm(8 + randDist), 0, true);
                } else {
                    moveCardinal(pow, inchesToCm(8), 0, true);
                }

                rb_time.sleep(1000);
            }
            moveCardinal(tinypow, inchesToCm(1), 270, false);
            autonGrabberLeft.setPosition(AUTON_GRABBER_LEFT_ACTIVE);
            moveCardinal(tinypow, inchesToCm(1), 270, true);
            rb_time.sleep(1000);

            moveCardinal(bigpow, inchesToCm(18), 90, true);
            rb_time.sleep(1000);
            moveCardinal(hugepow, inchesToCm(50), 0, true);
            rb_time.sleep(1000);
            autonGrabberLeft.setPosition(AUTON_GRABBER_LEFT_PASSIVE);
            rb_time.sleep(1000);
            moveCardinal(hugepow, inchesToCm(50 + 24), 180, true);
            rb_time.sleep(1000);
            moveCardinal(bigpow, inchesToCm(18 - 2), 270, true);

            moveCardinal(tinypow, inchesToCm(1), 270, false);
            autonGrabberLeft.setPosition(AUTON_GRABBER_LEFT_ACTIVE);
            moveCardinal(tinypow, inchesToCm(1), 270, true);
            rb_time.sleep(1000);

            moveCardinal(bigpow, inchesToCm(18), 90, true);
            rb_time.sleep(1000);
            moveCardinal(hugepow, inchesToCm(50 + 24), 0, true);
            rb_time.sleep(1000);
            autonGrabberLeft.setPosition(AUTON_GRABBER_LEFT_PASSIVE);
            rb_time.sleep(1000);

            logger.add("Values", values);
            logger.add("Min Index", minIndex);
            logger.update(true);
        }
        public void moveCardinal(double power, double distance, int direction, boolean stopMotors) {

            steering.resetAllEncoders();

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
            for (int motor_index = 0; motor_index < 4; motor_index++) {
                DrivingMotor motor = getMotor(motor_index);
                motor.setPower(power * motorDirections[motor_index] / maxPower );
            }

            double targetTicks = distanceToEncoderTicks(distance);
            int averageMotorTicks = 0;

            //TODO figure out way how to get stop request
            while (averageMotorTicks < targetTicks /*&& !isStopRequested()*/) {
                logger.add("", "Target Ticks: " + targetTicks);

                for (int motor_index = 0; motor_index < 4; motor_index++) {
                    DrivingMotor motor = getMotor(motor_index);
                    averageMotorTicks += motor.position() / 4;
                    logger.add("","LF Actual: " + getAllMotorNames()[motor_index]);
                }
                logger.update(true);
            }

            if (stopMotors) {
                steering.setAllPowers(0);
            }
        }

        public void moveCardinalRamping(double power, double distance, int direction) {

            steering.resetAllEncoders();

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

            // Distance things
            double targetTicks = distanceToEncoderTicks(distance);
            int averageMotorTicks = 0;

            double rampupTicks = 600;

            if (targetTicks / 2 < rampupTicks) {
                rampupTicks = targetTicks / 2;
            }

            // ElapsedTime rampupTimer = new ElapsedTime();

            double motorPower = 0;

            //TODO add stop request later
            while (averageMotorTicks < targetTicks /*&& !this.isStopRequested()*/) {

                // double currentTime = rampupTimer.time(TimeUnit.SECONDS);
                for (int motor_index = 0; motor_index < 4; motor_index++) {
                    DrivingMotor motor = getMotor(motor_index);
                    averageMotorTicks += motor.position() / 4;
                    logger.add("","LF Actual: " + getAllMotorNames()[motor_index]);
                }

                double powerOffsetStart = 0.1;
                double powerOffsetEnd = 0.1;

                if (averageMotorTicks < rampupTicks) {
                    motorPower = power * (powerOffsetStart + (1 - powerOffsetStart) * (averageMotorTicks / rampupTicks));
                }
                else if (averageMotorTicks > targetTicks - rampupTicks) {
                    motorPower = power * (powerOffsetEnd + (1 - powerOffsetEnd) * (targetTicks - averageMotorTicks) / rampupTicks);
                }
                else {
                    motorPower = power;
                }

                for (int motor_index = 0; motor_index < 4; motor_index++) {
                    DrivingMotor motor = getMotor(motor_index);
                    motor.setPower(power * motorDirections[motor_index] / maxPower );
                }

                logger.add("", "Target Ticks: " + targetTicks);

                for (int motor_index = 0; motor_index < 4; motor_index++) {
                    DrivingMotor motor = getMotor(motor_index);
                    averageMotorTicks += motor.position() / 4;
                    logger.add("","LF Actual: " + getAllMotorNames()[motor_index]);
                }
                logger.update(true);
            }

            steering.setAllPowers(0);
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
            for (DrivingMotor motor : getAllMotors()) {
                motor.setPower(turnPower);
            }

            double currentAngle = startAngle;

            if (angleDelta > 0 && targetAngle < startAngle ) {
                while (currentAngle >= startAngle || currentAngle < targetAngle) {
                    currentAngle = getIMUAngleConverted();
                    logger.add("", "Delta: " + angleDelta + " Target: " + targetAngle + " Current: " + currentAngle, true);
                }
            }
            else if (angleDelta < 0 && targetAngle > startAngle) {
                while (currentAngle <= startAngle || currentAngle > targetAngle) {
                    currentAngle = getIMUAngleConverted();
                    logger.add("", "Delta: " + angleDelta + " Target: " + targetAngle + " Current: " + currentAngle, true);
                }
            }
            else if (angleDelta > 0) {
                while (currentAngle < targetAngle) {
                    currentAngle = getIMUAngleConverted();
                    logger.add("", "Delta: " + angleDelta + " Target: " + targetAngle + " Current: " + currentAngle, true);
                }
            }
            else {
                while (currentAngle > targetAngle) {
                    currentAngle = getIMUAngleConverted();
                    logger.add("", "Delta: " + angleDelta + " Target: " + targetAngle + " Current: " + currentAngle, true);
                }
            }

            steering.setAllPowers(0);
        }
    }

    public static int distanceToEncoderTicks(double distance) {
        final double GEAR_RATIO = 1; // output/input teeth
        final int TICKS_PER_MOTOR_ROTATION = 1120;
        final int TICKS_PER_WHEEL_ROTATION = (int)((TICKS_PER_MOTOR_ROTATION * GEAR_RATIO) + 0.5);
        final double WHEEL_DIAMETER = 10.16; // [cm]
        final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER; // [cm]
        final double DISTANCE_PER_TICK = WHEEL_CIRCUMFERENCE / TICKS_PER_WHEEL_ROTATION;
        return (int)((distance / DISTANCE_PER_TICK) + 0.5);
    }
}
