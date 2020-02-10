package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Input Test TeleOp", group="TeleOp OpMode")
public class InputTestTeleOp extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {}

    @Override
    public void init_loop() {
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Runtime", runtime.toString());
        telemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        telemetry.addData("Left X", gamepad1.left_stick_x);
        telemetry.addData("Left Y", gamepad1.left_stick_y);
        telemetry.addData("Right X", gamepad1.right_stick_x);
        telemetry.addData("Right Y", gamepad1.right_stick_y);
        telemetry.addData("Runtime", runtime.toString());
        telemetry.update();
    }
}
