package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name = "Skystone Auton")
public class SkystoneAuton extends LinearOpMode {

    private ElapsedTime runtime;
    BNO055IMU imu;

    @Override
    public void runOpMode() {

        // ----- INIT SECTION -----

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        runtime = new ElapsedTime();

        lfMotor = hardwareMap.dcMotor.get("lfMotor");
        rfMotor = hardwareMap.dcMotor.get("rfMotor");
        lbMotor = hardwareMap.dcMotor.get("lbMotor");
        rbMotor = hardwareMap.dcMotor.get("rbMotor");
        setAllRunModes(DcMotor.RunMode.RUN_USING_ENCODER);
        setAllZeroPowerBehaviors(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            telemetry.addData("Status", "Calibrating IMU");
            telemetry.update();
            idle();
        }

        while (!this.isStarted()) {
            telemetry.addData("Status", "Waiting for start");
            telemetry.addData("Runtime", runtime.toString());
            telemetry.addData("IMU calibration status", imu.getCalibrationStatus().toString());
            telemetry.update();
        }

        // ----- RUN SECTION -----

        moveCardinal(0.5, inchesToCm(46), 90);
        sleep(1000);
        moveCardinal(0.5, inchesToCm(40), 0);
        sleep(1000);
        moveCardinal(0.5, inchesToCm(2), 90);

        sleep(3000);

        moveCardinal(0.5, inchesToCm(10), 270);
        sleep(1000);
        moveCardinal(0.5, inchesToCm(10), 180);
        sleep(1000);
        turnDegrees(0.5, 180);
        turnDegrees(0.5, 180);
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

    // ----- MOTOR STUFF -----

    private DcMotor lfMotor;
    private DcMotor rfMotor;
    private DcMotor lbMotor;
    private DcMotor rbMotor;

    Telemetry telemetry;

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
