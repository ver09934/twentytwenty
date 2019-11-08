package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Sensor Test Tele-Op", group="TeleOp OpMode")
public class SensorTestTeleOp extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private Servo testServo;

    // private DistanceSensor testSensor;

    private double targetPosition = 0;

    private double servoIncrement = 0.2;

    private boolean bToggleLock = false;
    private double position1 = 1;
    // private double position2 = 0.6;
    private double position2 = 0;

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        testServo = hardwareMap.servo.get("testServo");
        // testSensor = hardwareMap.get(DistanceSensor.class, "lidar");
        testServo.setPosition(position1);
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

        /*
        if (this.gamepad1.right_stick_x > 0.5) {
            if (targetPosition < 2) {
                targetPosition += servoIncrement;
            }
        }
        else if (this.gamepad1.right_stick_x < -0.5) {
            if (targetPosition > -1) {
                targetPosition -= servoIncrement;
            }
        }
        testServo.setPosition(targetPosition);
        telemetry.addData("Servo Position", testServo.getPosition());
        */

        if (this.gamepad1.b) {
            if (!bToggleLock) {
                bToggleLock = true;
                if (testServo.getPosition() == position1) {
                    testServo.setPosition(position2);
                }
                else {
                    testServo.setPosition(position1);
                }
            }
        }
        else {
            bToggleLock = false;
        }
        telemetry.addData("Servo Position", testServo.getPosition());

        /*
        telemetry.addData("range", String.format("%.01f mm", testSensor.getDistance(DistanceUnit.MM)));
        telemetry.addData("range", String.format("%.01f cm", testSensor.getDistance(DistanceUnit.CM)));
        telemetry.addData("range", String.format("%.01f m", testSensor.getDistance(DistanceUnit.METER)));
        telemetry.addData("range", String.format("%.01f in", testSensor.getDistance(DistanceUnit.INCH)));
         */

        telemetry.addData("Runtime", runtime.toString());
        telemetry.update();
    }

    // Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
    }

}