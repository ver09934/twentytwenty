package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Drive Test TeleOp", group="TeleOp OpMode")
public class TestDriveTeleOp extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    public DcMotor lfMotor;
    public DcMotor rfMotor;
    public DcMotor lbMotor;
    public DcMotor rbMotor;

    double x = 0;
    double y = 0;

    double turnSpeed = 0;
    double driveSpeed = 0;

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

    @Override
    public void init() {
        lfMotor = hardwareMap.dcMotor.get("lfMotor");
        rfMotor = hardwareMap.dcMotor.get("rfMotor");
        lbMotor = hardwareMap.dcMotor.get("lbMotor");
        rbMotor = hardwareMap.dcMotor.get("rbMotor");
        setAllRunModes(DcMotor.RunMode.RUN_USING_ENCODER);
        setAllZeroPowerBehaviors(DcMotor.ZeroPowerBehavior.BRAKE);
    }

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

        /*
        Input parameters:
        - x velocity (-1 to 1)
        - y velocity (-1 to 1)
        - turning speed (-1 to 1)
        - speed scale (0 to 1)
         */

        // double maxPossiblePower = Math.sqrt(2);
        // driveAngle = Math.atan2(xInput, yInput);
        // driveSpeed = Math.sqrt(Math.pow(xInput, 2) + Math.pow(yInput, 2));
        // double x = Math.cos(driveAngle);
        // double y = Math.sin(driveAngle);

        double activation_thresh = 0.01;

        if (Math.abs(gamepad1.right_stick_x) > activation_thresh) {
            turnSpeed = gamepad1.right_stick_x;
        }
        else {
            turnSpeed = 0;
        }

        if (Math.abs(gamepad1.left_stick_x) > activation_thresh || Math.abs(gamepad1.left_stick_y) > activation_thresh) {
            x = -gamepad1.left_stick_x;
            y = gamepad1.left_stick_y;
        }
        else {
            x = 0;
            y = 0;
        }
        driveSpeed = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

        /*
        double powerLF = x - y;
        double powerRF = x + y;
        double powerLB = -x - y;
        double powerRB = -x + y;
        */

        telemetry.addData("Runtime", runtime.toString());
        telemetry.update();
    }
}
