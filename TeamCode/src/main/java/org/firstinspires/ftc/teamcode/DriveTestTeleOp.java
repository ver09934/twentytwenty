package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.DriverFunction.MAX_SPEED_RATIO;
import static org.firstinspires.ftc.teamcode.DriverFunction.MIN_SPEED_RATIO;
import static org.firstinspires.ftc.teamcode.DriverFunction.NORMAL_SPEED_RATIO;

@TeleOp(name="Drive Test Tele-Op", group="TeleOp OpMode")
public class DriveTestTeleOp extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DriverFunction driverFunction;
    private DriverFunction.Steering steering;

    private boolean bToggleLock = false;
    private boolean runGulper = false;

    DcMotor motor1;
    DcMotor motor2;

    DcMotor winchMotor;

    private final static double TURNING_SPEED_BOOST = 0.3;

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        driverFunction = new DriverFunction(hardwareMap, telemetry);
        steering = driverFunction.getSteering();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motor1 = hardwareMap.dcMotor.get("intake1");
        motor2 = hardwareMap.dcMotor.get("intake2");
        winchMotor = hardwareMap.dcMotor.get(""); // TODO

        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        winchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor1.setDirection(DcMotorSimple.Direction.REVERSE);

        motor1.setPower(0);
        motor2.setPower(0);
        winchMotor.setPower(0);
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

        // --- gulper motors ---

        if (this.gamepad1.b) {
            if (!bToggleLock) {
                bToggleLock = true;
                runGulper = !runGulper;
            }
            telemetry.addData("B Pressed", "True");
        }
        else {
            bToggleLock = false;
            telemetry.addData("B Pressed", "False");
        }

        if (runGulper) {
            telemetry.addData("Gulpers", "True");
            motor1.setPower(1);
            motor2.setPower(1);
        }
        else {
            telemetry.addData("Gulpers", "False");
            motor1.setPower(0);
            motor2.setPower(0);
        }

        // --- winch motor ---

        double winchMotorSpeed = 0.5;
        if (this.gamepad1.left_bumper) {
            winchMotor.setPower(winchMotorSpeed);
        }
        else if (this.gamepad1.right_bumper) {
            winchMotor.setPower(-winchMotorSpeed);
        }
        else {
            winchMotor.setPower(0);
        }

        // ----- Gamepad 1: Driving Functions -----

        // D-Pad: Compass rose drive
        if (this.gamepad1.dpad_right) {
            steering.moveDegrees(180, MIN_SPEED_RATIO);
        }
        if (this.gamepad1.dpad_up) {
            steering.moveDegrees(270, MIN_SPEED_RATIO);
        }
        if (this.gamepad1.dpad_left) {
            steering.moveDegrees(0, MIN_SPEED_RATIO);
        }
        if (this.gamepad1.dpad_down) {
            steering.moveDegrees(90, MIN_SPEED_RATIO);
        }

        // Left/Right Triggers: Set driving speed ratio
        if (this.gamepad1.right_trigger > 0.5) {
            steering.setSpeedRatio(MAX_SPEED_RATIO); // Left trigger: minimum speed ratio
        }
        else if (this.gamepad1.left_trigger > 0.5) {
            steering.setSpeedRatio(MIN_SPEED_RATIO); // Right trigger: maximum speed ratio
        }
        else {
            steering.setSpeedRatio(NORMAL_SPEED_RATIO);
        }

        // Right Stick: Turn/Rotate
        if (this.gamepad1.right_stick_x > 0.1) {
            steering.turnCounterclockwise();
            steering.setSpeedRatio(Math.min(1, steering.getSpeedRatio() + TURNING_SPEED_BOOST));
        }
        else if (this.gamepad1.right_stick_x < -0.1) {
            steering.turnClockwise();
            steering.setSpeedRatio(Math.min(1, steering.getSpeedRatio() + TURNING_SPEED_BOOST));
        }

        // Left Stick: Drive/Strafe
        if (Math.abs(this.gamepad1.left_stick_x) > 0.1 || Math.abs(this.gamepad1.left_stick_y) > 0.1) {
            double angle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x);
            telemetry.addData("Angle", angle);
            steering.moveRadians(angle);
        }
        else {
            telemetry.addData("Angle", 0);
        }

        // Finish steering, putting power into hardware
        steering.finishSteering();

        // Update telemetry
        telemetry.addData("Runtime", runtime.toString());
        telemetry.update();
    }

    // Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
        driverFunction.resetAllEncoders();
    }

}
