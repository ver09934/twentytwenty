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

@Autonomous(name = "Dist Test Auton")
public class DistTestAuton extends LinearOpMode {

    private ElapsedTime runtime;

    @Override
    public void runOpMode() {

        initIMU();

        waitForStart();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        runtime = new ElapsedTime();

        initMotors();
        initDistanceSensors();

        distanceTest(0.5, 0);
    }

    // ----- META-METHODS -----

    public void distanceTest(double motorPower, int direction) {

        // NOTE: Make power small so correction can actually add something
        // NOTE: Handle max >> appears to be ~819 for right

        int[] motorDirections = {1, 1, 1, 1};

        if (direction == 0) {
            motorDirections[0] = -1;
            motorDirections[1] = -1;
        }
        else if (direction == 180) {
            motorDirections[2] = -1;
            motorDirections[3] = -1;
        }
        else {
            throw new RuntimeException("Direction must be -1 or 1");
        }

        double maxPower = 1;

        while (!isStopRequested()) {

            /*
            double d1 = getLeftDistance();
            double d2 = getRightDistance();

            double d3 = 27;

            double angle = Math.toDegrees(Math.atan2(d2 - d1, d3));
             */

            double tmpAngle = getIMUAngleConverted();
            double angle = getAngleDifference(tmpAngle, 0);

            double k = 0.012;

            double correction = angle * k;

            /*
            boolean zeroCorrect = false;
            if (d1 > 800 || d2 > 800) {
                zeroCorrect = true;
            }
            if (zeroCorrect) {
                correction = 0;
            }
             */

            lfMotor.setPower(motorPower * motorDirections[0] / maxPower + correction);
            rfMotor.setPower(motorPower * motorDirections[1] / maxPower + correction);
            lbMotor.setPower(motorPower * motorDirections[2] / maxPower + correction);
            rbMotor.setPower(motorPower * motorDirections[3] / maxPower + correction);

            // telemetry.addData("Left Distance", d1);
            // telemetry.addData("Right Distance", d2);
            telemetry.addData("Angle", angle);
            telemetry.addData("Correction", correction);
            // telemetry.addData("Zero Correct", zeroCorrect);
            telemetry.update();
        }
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
