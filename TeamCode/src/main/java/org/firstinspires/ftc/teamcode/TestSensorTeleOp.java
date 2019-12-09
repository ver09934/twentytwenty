package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Test Sensor Tele-Op", group="TeleOp OpMode")
public class TestSensorTeleOp extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DistanceSensor testSensor;

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        testSensor = hardwareMap.get(DistanceSensor.class, "lidar");
    }

    // Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {}

    // Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
        runtime.reset();
    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {

        telemetry.addData("range", String.format("%.01f mm", testSensor.getDistance(DistanceUnit.MM)));
        telemetry.addData("range", String.format("%.01f cm", testSensor.getDistance(DistanceUnit.CM)));
        telemetry.addData("range", String.format("%.01f m", testSensor.getDistance(DistanceUnit.METER)));
        telemetry.addData("range", String.format("%.01f in", testSensor.getDistance(DistanceUnit.INCH)));

        telemetry.addData("Runtime", runtime.toString());
        telemetry.update();
    }

    // Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
    }

}