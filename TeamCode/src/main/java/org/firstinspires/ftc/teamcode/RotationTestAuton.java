package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "Rotation Test Auton")
public class RotationTestAuton extends LinearOpMode {

    private ElapsedTime runtime;

    BNO055IMU imu;

    DcMotor lfMotor;
    DcMotor rfMotor;
    DcMotor lbMotor;
    DcMotor rbMotor;

    @Override
    public void runOpMode() {

        // --- Init Section ---

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        runtime = new ElapsedTime();

        lfMotor = hardwareMap.dcMotor.get("lfMotor");
        rfMotor = hardwareMap.dcMotor.get("rfMotor");
        lbMotor = hardwareMap.dcMotor.get("lbMotor");
        rbMotor = hardwareMap.dcMotor.get("rbMotor");

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

        while (!this.isStarted()) {
            telemetry.addData("Status", "Waiting for start");
            telemetry.addData("Runtime", runtime.toString());
            telemetry.addData("IMU calibration status", imu.getCalibrationStatus().toString());
            telemetry.update();
        }

        // --- Run Section ---

        runtime.reset();

        sleep(1000);

        double turnPower = 0.5;

        // Between -180 and 180
        double angleDelta = 90;

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

        if (angleDelta > 0 && targetAngle < startAngle) {
            while (currentAngle >= startAngle || currentAngle < targetAngle) {
                currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                currentAngle = currentAngle < 0 ? currentAngle + 360 : currentAngle;
                telemetry.addLine("Delta: " + angleDelta + " Target: " + targetAngle + " Current: " + currentAngle);
                telemetry.update();
            }
        }
        else if (angleDelta < 0 && targetAngle > startAngle) {
            while (currentAngle <= startAngle || currentAngle > targetAngle) {
                currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                currentAngle = currentAngle < 0 ? currentAngle + 360 : currentAngle;
                telemetry.addLine("Delta: " + angleDelta + " Target: " + targetAngle + " Current: " + currentAngle);
                telemetry.update();
            }
        }
        else if (angleDelta > 0) {
            while (currentAngle < targetAngle) {
                currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                currentAngle = currentAngle < 0 ? currentAngle + 360 : currentAngle;
                telemetry.addLine("Delta: " + angleDelta + " Target: " + targetAngle + " Current: " + currentAngle);
                telemetry.update();
            }
        }
        else {
            while (currentAngle > targetAngle) {
                currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                currentAngle = currentAngle < 0 ? currentAngle + 360 : currentAngle;
                telemetry.addLine("Delta: " + angleDelta + " Target: " + targetAngle + " Current: " + currentAngle);
                telemetry.update();
            }
        }

        lfMotor.setPower(0);
        rfMotor.setPower(0);
        lbMotor.setPower(0);
        rbMotor.setPower(0);

        while (opModeIsActive()) {

            telemetry.addLine("Delta: " + angleDelta + " Target: " + targetAngle + " Current: " + currentAngle);

            telemetry.addData("Status", "Running");
            telemetry.addData("Runtime", runtime.toString());

            telemetry.update();
        }
    }

}
