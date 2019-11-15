package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Test Servo Tele-Op", group="TeleOp OpMode")
public class TestServoTeleOp extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private Servo testServo;

    private double lowPosition = 0;
    private double highPosition = 1;

    private double positionAdjustIncrement = 0.05;

    private boolean dpadRightLeftToggleLock = false;
    private boolean dpadUpDownToggleLock = false;

    private boolean bToggleLock = false;

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        testServo = hardwareMap.servo.get("testServo");
        testServo.setPosition(lowPosition);
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

        // TODO: Right and Left Triggers: Adjust increment

        // D-Pad: Adjust low and high positions

        if (this.gamepad1.dpad_right) {
            if (!dpadRightLeftToggleLock) {
                boolean updatePosition = testServo.getPosition() == lowPosition;
                dpadRightLeftToggleLock = true;
                lowPosition += positionAdjustIncrement;
                if (updatePosition) {
                    testServo.setPosition(lowPosition);
                }
            }
        }
        else if (this.gamepad1.dpad_left) {
            if (!dpadRightLeftToggleLock) {
                boolean updatePosition = testServo.getPosition() == lowPosition;
                dpadRightLeftToggleLock = true;
                lowPosition -= positionAdjustIncrement;
                if (updatePosition) {
                    testServo.setPosition(lowPosition);
                }
            }
        }
        else {
            dpadRightLeftToggleLock = false;
        }

        if (this.gamepad1.dpad_up) {
            if (!dpadUpDownToggleLock) {
                boolean updatePosition = testServo.getPosition() == highPosition;
                dpadUpDownToggleLock = true;
                highPosition += positionAdjustIncrement;
                if (updatePosition) {
                    testServo.setPosition(highPosition);
                }
            }
        }
        else if (this.gamepad1.dpad_down) {
            if (!dpadUpDownToggleLock) {
                boolean updatePosition = testServo.getPosition() == highPosition;
                dpadUpDownToggleLock = true;
                highPosition -= positionAdjustIncrement;
                if (updatePosition) {
                    testServo.setPosition(highPosition);
                }
            }
        }
        else {
            dpadUpDownToggleLock = false;
        }

        if (this.gamepad1.b) {
            if (!bToggleLock) {
                bToggleLock = true;
                if (testServo.getPosition() == lowPosition) {
                    testServo.setPosition(highPosition);
                }
                else {
                    testServo.setPosition(lowPosition);
                }
            }
        }
        else {
            bToggleLock = false;
        }

        telemetry.addData("Position Adjust Increment", positionAdjustIncrement);
        telemetry.addData("Low Position", lowPosition);
        telemetry.addData("High Position", highPosition);
        telemetry.addData("Servo Position", testServo.getPosition());

        telemetry.addData("Runtime", runtime.toString());
        telemetry.update();
    }

    // Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
    }

}