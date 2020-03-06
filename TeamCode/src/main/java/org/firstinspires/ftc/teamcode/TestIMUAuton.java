package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.*;

@Autonomous(name = "Test IMU Auton")
public class TestIMUAuton extends LinearOpMode {

    private ElapsedTime runtime;

    BNO055IMU imu;
    // Orientation lastAngles = new Orientation();

    double globalAngle;
    double correction;

    @Override
    public void runOpMode() {

        // --- Init Section ---

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        runtime = new ElapsedTime();

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

        // --- Run Section ----

        runtime.reset();

        while (opModeIsActive()) {
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            Position position = imu.getPosition();

            double legitAngle = angles.firstAngle;
            double LEGIT_ANGLE = legitAngle < 0 ? legitAngle + 360 : legitAngle;

            telemetry.addData("Status", "Running");
            telemetry.addData("Runtime", runtime.toString());

            telemetry.addData("Position", position);
            telemetry.addData("Position units", position.unit);
            telemetry.addData("Position x", position.x);
            telemetry.addData("Position y", position.y);
            telemetry.addData("Position z", position.z);

            telemetry.addLine();
            telemetry.addLine();

            telemetry.addLine(angles.toString());
            telemetry.addData("Legit angle", legitAngle);
            telemetry.addData("LEGIT ANGLE", LEGIT_ANGLE);

            telemetry.update();
        }
    }
}
