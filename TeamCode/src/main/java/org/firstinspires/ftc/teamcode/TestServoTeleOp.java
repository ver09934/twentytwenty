package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Test Servo Tele-Op", group="TeleOp OpMode")
public class TestServoTeleOp extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Servo testServo;

    private boolean servoHigh = false;

    private double lowPosition = 0;
    private double highPosition = 1;

    private double fineAdjustIncrement = 0.01;
    private double coarseAdjustIncrement = 0.1;

    private boolean dpadRightLeftToggleLock = false;
    private boolean dpadUpDownToggleLock = false;
    private boolean bToggleLock = false;

    private boolean ayToggleLock = false;

    private double getIncrement() {
        if (this.gamepad1.left_trigger > 0.5 || this.gamepad2.right_trigger > 0.5) {
            return fineAdjustIncrement;
        }
        else {
            return coarseAdjustIncrement;
        }
    }

    private double round(double value, double places) {
        return Math.round(value * Math.pow(10, places)) / Math.pow(10, places);
    }

    String[] servos = {
        "blockServoLeft",
        "blockServoRight",
        "plateServoLeft",
        "plateServoRight",
        "autonGrabberLeft",
        "autonGrabberRight"
    };
    String servoString;

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {

        int currentIndex = 0;
        String selector = ">>>";
        String selectorSpaces = "";
        for (int i = 0; i < selector.length(); i++) {
            selectorSpaces += " ";
        }

        while (true) {

            if (this.gamepad1.y) {
                if (!ayToggleLock) {
                    ayToggleLock = true;
                    if (currentIndex == 0) {
                        currentIndex = servos.length - 1;
                    }
                    else if (currentIndex > 0) {
                        currentIndex--;
                    }
                }
            }
            else if (this.gamepad1.a) {
                if (!ayToggleLock) {
                    ayToggleLock = true;
                    if (currentIndex == servos.length - 1) {
                        currentIndex = 0;
                    }
                    else if (currentIndex < servos.length - 1) {
                        currentIndex++;
                    }
                }
            }
            else {
                ayToggleLock = false;
            }

            if (this.gamepad1.x) {
                break;
            }

            telemetry.addLine("Select a servo:");
            for (int i = 0; i < servos.length; i++) {
                String prefix;
                if (i == currentIndex) {
                    prefix = selector;
                }
                else {
                    prefix = selectorSpaces;
                }
                telemetry.addLine(prefix + " " + servos[i]);
            }
            telemetry.update();
        }

        servoString = servos[currentIndex];
        telemetry.addData("Servo selected", servoString);

        testServo = hardwareMap.servo.get(servoString);

        servoHigh = false;
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

        // D-Pad Right/Left: Adjust low position
        if (this.gamepad1.dpad_right) {
            if (!dpadRightLeftToggleLock) {
                dpadRightLeftToggleLock = true;
                lowPosition += getIncrement();
                lowPosition = round(lowPosition, 2);
                if (!servoHigh) {
                    testServo.setPosition(lowPosition);
                }
            }
        }
        else if (this.gamepad1.dpad_left) {
            if (!dpadRightLeftToggleLock) {
                dpadRightLeftToggleLock = true;
                lowPosition -= getIncrement();
                lowPosition = round(lowPosition, 2);
                if (!servoHigh) {
                    testServo.setPosition(lowPosition);
                }
            }
        }
        else {
            dpadRightLeftToggleLock = false;
        }

        // D-Pad Up/Down: Adjust high position
        if (this.gamepad1.dpad_up) {
            if (!dpadUpDownToggleLock) {
                dpadUpDownToggleLock = true;
                highPosition += getIncrement();
                highPosition = round(highPosition, 2);
                if (servoHigh) {
                    testServo.setPosition(highPosition);
                }
            }
        }
        else if (this.gamepad1.dpad_down) {
            if (!dpadUpDownToggleLock) {
                dpadUpDownToggleLock = true;
                highPosition -= getIncrement();
                highPosition = round(highPosition, 2);
                if (servoHigh) {
                    testServo.setPosition(highPosition);
                }
            }
        }
        else {
            dpadUpDownToggleLock = false;
        }

        // B Button: Toggle Servo Position
        if (this.gamepad1.b) {
            if (!bToggleLock) {
                bToggleLock = true;
                servoHigh = !servoHigh;
                if (servoHigh) {
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

        telemetry.addData("Servo", servoString);
        telemetry.addData("Fine Adjust Increment", fineAdjustIncrement);
        telemetry.addData("Coarse Adjust Increment", coarseAdjustIncrement);
        telemetry.addData("Low Position", lowPosition);
        telemetry.addData("High Position", highPosition);
        telemetry.addData("Servo Position", testServo.getPosition());
        telemetry.addData("Target Position: ", servoHigh ? highPosition : lowPosition);

        telemetry.addData("Runtime", runtime.toString());
        telemetry.update();
    }

    // Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
    }
}
