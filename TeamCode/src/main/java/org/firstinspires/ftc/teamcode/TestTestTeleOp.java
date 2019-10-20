package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Test Test Tele-Op", group="TeleOp OpMode")
public class TestTestTeleOp extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private boolean rightTriggerToggleLock = false;

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
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

        if (this.gamepad1.right_trigger > 0.5) {
            if (!rightTriggerToggleLock) {
                rightTriggerToggleLock = true;
                telemetry.addData("Trigger", "Pressed");
            }
        }
        else {
            rightTriggerToggleLock = false;
            telemetry.addData("Trigger", "Not Pressed");
        }

    }

    // Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
    }
}
