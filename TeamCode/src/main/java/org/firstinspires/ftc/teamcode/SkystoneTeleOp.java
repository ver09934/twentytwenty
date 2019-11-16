package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.DriverFunction.MAX_SPEED_RATIO;
import static org.firstinspires.ftc.teamcode.DriverFunction.MIN_SPEED_RATIO;
import static org.firstinspires.ftc.teamcode.DriverFunction.NORMAL_SPEED_RATIO;

@TeleOp(name="Skystone Tele-Op", group="TeleOp OpMode")
public class SkystoneTeleOp extends OpMode {

    // Important things
    private ElapsedTime runtime = new ElapsedTime();
    private DriverFunction driverFunction;
    private DriverFunction.Steering steering;

    // Driving variables
    private final static double TURNING_SPEED_BOOST = 0.3;

    // DC Motors
    DcMotor motor1;
    DcMotor motor2;
    DcMotor winchMotor1;
    DcMotor winchMotor2;

    // Servos
    private Servo blockServo;

    // Toggle locks
    private boolean gamepad1XToggleLock = false;
    private boolean gamepad2XToggleLock = false;
    private boolean gamepad2BToggleLock = false;

    // Boolean state variables
    private boolean directionReverse = false;
    private boolean runGulper = false;

    // Test servo positions
    private double testServoPosition1 = 1;
    private double testServoPosition2 = 0.6;

    // Winch motor speeds
    private double winchSpeed = 0.5;
    private double winchMotor1Speed = winchSpeed;
    private double winchMotor2Speed = winchSpeed;

    // Gulper motor speeds
    private double gulperForwardPower = 1;
    private double gulperOffPower = 0;

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        driverFunction = new DriverFunction(hardwareMap, telemetry);
        steering = driverFunction.getSteering();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motor1 = hardwareMap.dcMotor.get("intake1");
        motor2 = hardwareMap.dcMotor.get("intake2");
        winchMotor1 = hardwareMap.dcMotor.get("winch1");
        winchMotor2 = hardwareMap.dcMotor.get("winch2");

        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        winchMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        winchMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winchMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winchMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        winchMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        motor1.setPower(0);
        motor2.setPower(0);
        winchMotor1.setPower(0);
        winchMotor2.setPower(0);

        blockServo = hardwareMap.servo.get("testServo");
        blockServo.setPosition(testServoPosition1);
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

        // --------------------------------------------------
        // ---------- Gamepad 1: Driving Functions ----------
        // --------------------------------------------------

        // X Button: Toggle direction reverse
        if (this.gamepad1.x) {
            if (!gamepad1XToggleLock) {
                gamepad1XToggleLock = true;
                directionReverse = !directionReverse;
            }
        }
        else {
            gamepad1XToggleLock = false;
        }
        telemetry.addData("Direction Reverse", directionReverse);

        // D-Pad: Compass rose drive
        if (!directionReverse) {
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
        }
        else {
            if (this.gamepad1.dpad_right) {
                steering.moveDegrees(0, MIN_SPEED_RATIO);
            }
            if (this.gamepad1.dpad_up) {
                steering.moveDegrees(90, MIN_SPEED_RATIO);
            }
            if (this.gamepad1.dpad_left) {
                steering.moveDegrees(180, MIN_SPEED_RATIO);
            }
            if (this.gamepad1.dpad_down) {
                steering.moveDegrees(270, MIN_SPEED_RATIO);
            }
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
            double angle;
            if (!directionReverse) {
                angle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x);
            }
            else {
                angle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x);
            }
            telemetry.addData("Angle", angle);
            steering.moveRadians(angle);
        }
        else {
            telemetry.addData("Angle", 0);
        }

        // --------------------------------------------------
        // ---------- Gamepad 2: Gunner Functions -----------
        // --------------------------------------------------

        // --- B Button: Gulper motors ---
        // TODO: Add reversing to the toggle
        if (this.gamepad2.b) {
            if (!gamepad2BToggleLock) {
                gamepad2BToggleLock = true;
                runGulper = !runGulper;
            }
        }
        else {
            gamepad2BToggleLock = false;
        }
        if (runGulper) {
            motor1.setPower(gulperForwardPower);
            motor2.setPower(gulperForwardPower);
        }
        else {
            motor1.setPower(gulperOffPower);
            motor2.setPower(gulperOffPower);
        }
        telemetry.addData("Gulpers running", runGulper);

        // --- Left/Right Bumpers: Winch motors ---
        // TODO: Add a multi-stepped run to position
        if (this.gamepad2.left_bumper) {
            winchMotor1.setPower(winchMotor1Speed);
            winchMotor2.setPower(winchMotor2Speed);
            telemetry.addData("Winch running", "Down");
        }
        else if (this.gamepad2.right_bumper) {
            winchMotor1.setPower(-winchMotor1Speed);
            winchMotor2.setPower(-winchMotor2Speed);
            telemetry.addData("Winch running", "Up");
        }
        else {
            winchMotor1.setPower(0);
            winchMotor2.setPower(0);
            telemetry.addData("Winch running", "False");
        }

        // --- X Button: Block Gripper Servo ---
        if (this.gamepad2.x) {
            if (!gamepad2XToggleLock) {
                gamepad2XToggleLock = true;
                if (blockServo.getPosition() == testServoPosition1) {
                    blockServo.setPosition(testServoPosition2);
                }
                else {
                    blockServo.setPosition(testServoPosition1);
                }
            }
        }
        else {
            gamepad2XToggleLock = false;
        }
        telemetry.addData("Block Servo Position", blockServo.getPosition());

        // --- Build Plate Clamper Servos ---
        // TODO

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
