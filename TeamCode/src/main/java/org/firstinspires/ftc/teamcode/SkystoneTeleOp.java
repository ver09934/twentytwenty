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

    /* The encoder functions available to us:
    winchMotor1.setPower(0);
    double tmp = winchMotor1.getPower();
    winchMotor1.setTargetPosition(0);
    int tmp = winchMotor1.getTargetPosition();
    boolean tmp = winchMotor1.isBusy();
    int tmp = winchMotor1.getCurrentPosition();
    */

    // TODO: Rename variables and hardware devices from 1/2 to left/right

    // Important things
    private ElapsedTime runtime = new ElapsedTime();
    private DriverFunction driverFunction;
    private DriverFunction.Steering steering;

    // Driving variables
    private final static double TURNING_SPEED_BOOST = 0.3;

    // DC Motors
    DcMotor gulperMotor1;
    DcMotor gulperMotor2;
    DcMotor winchMotor1;
    DcMotor winchMotor2;

    // Servos
    private Servo blockServo;
    private Servo plateServoLeft;
    private Servo plateServoRight;

    // Toggle locks
    private boolean gamepad1XToggleLock = false;
    private boolean gamepad2AToggleLock = false;
    private boolean gamepad2BToggleLock = false;
    private boolean gamepad2XToggleLock = false;
    private boolean gamepad2YToggleLock = false;
    private boolean gamepad2BumperToggleLock = false;
    private boolean gamepad2RightTriggerToggleLock = false;
    private boolean gamepad2LeftTriggerToggleLock = false;

    // Boolean state variables
    private boolean driveDirectionReverse = false;
    private boolean gulperReverse = false;
    private boolean runGulper = false;
    private boolean winchesPowered = false;
    private boolean plateServosUp = false;
    private boolean blockServoOpen = false;

    // Block servo positions
    private double blockServoClosedPosition = 0;
    private double blockServoOpenPosition = 0.5;

    // Winch things
    private double winchPower = 0.6;
    private double winchMotor1Power = winchPower;
    private double winchMotor2Power = winchPower;

    private int foundationHeight = 600;
    private int winchMotorStep = 750;
    private int[] winchMotorPositions = {
            0,
            foundationHeight,
            foundationHeight + winchMotorStep * 1,
            foundationHeight + winchMotorStep * 2,
            foundationHeight + winchMotorStep * 3,
            foundationHeight + winchMotorStep * 4,
            foundationHeight + winchMotorStep * 5,
            foundationHeight + winchMotorStep * 6,
            // foundationHeight + winchMotorStep * 7
            (int) ((foundationHeight + winchMotorStep * 6.8) + 0.5)
    };
    private int[] winchMotor1Offsets = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    private int[] winchMotor2Offsets = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    private int currentWinchIndex = 0;

    // Gulper motor speeds
    private double gulperForwardPower = 1;
    private double gulperReversePower = -gulperForwardPower;
    private double gulperOffPower = 0;

    // Build plate servo positions
    private double plateServoLeftDown = 0.28;
    private double plateServoLeftUp = 0.5;
    private double plateServoRightDown = 0.82;
    private double plateServoRightUp = 0.6;

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        driverFunction = new DriverFunction(hardwareMap, telemetry);
        steering = driverFunction.getSteering();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        gulperMotor1 = hardwareMap.dcMotor.get("intake1");
        gulperMotor2 = hardwareMap.dcMotor.get("intake2");
        winchMotor1 = hardwareMap.dcMotor.get("winch1");
        winchMotor2 = hardwareMap.dcMotor.get("winch2");

        gulperMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        gulperMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        winchMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        winchMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        winchMotor1.setTargetPosition(0);
        winchMotor2.setTargetPosition(0);
        winchMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        winchMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        gulperMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gulperMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winchMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        winchMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        gulperMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        winchMotor1.setDirection(DcMotorSimple.Direction.REVERSE);

        gulperMotor1.setPower(0);
        gulperMotor2.setPower(0);

        winchMotor1.setPower(0);
        winchMotor2.setPower(0);
        winchesPowered = false;

        blockServo = hardwareMap.servo.get("testServo");

        blockServoOpen = false;
        blockServo.setPosition(blockServoClosedPosition);

        plateServoLeft = hardwareMap.servo.get("plateServo1");
        plateServoRight = hardwareMap.servo.get("plateServo2");

        plateServosUp = false;
        plateServoLeft.setPosition(plateServoLeftDown);
        plateServoRight.setPosition(plateServoRightDown);
    }

    // Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {}

    // Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
        runtime.reset();
        winchMotor1.setPower(winchMotor1Power);
        winchMotor2.setPower(winchMotor2Power);
        winchesPowered = true;
        winchMotor1.setTargetPosition(winchMotorPositions[currentWinchIndex] + winchMotor1Offsets[currentWinchIndex]);
        winchMotor2.setTargetPosition(winchMotorPositions[currentWinchIndex] + winchMotor2Offsets[currentWinchIndex]);
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
                driveDirectionReverse = !driveDirectionReverse;
            }
        }
        else {
            gamepad1XToggleLock = false;
        }
        telemetry.addData("Direction Reverse", driveDirectionReverse);

        // D-Pad: Compass rose drive
        if (!driveDirectionReverse) {
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
            if (!driveDirectionReverse) {
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

        // --- A Button: Toggle Gulper Direction ---
        if (this.gamepad2.a) {
            if (!gamepad2AToggleLock) {
                gamepad2AToggleLock = true;
                gulperReverse = !gulperReverse;
            }
        }
        else {
            gamepad2AToggleLock = false;
        }

        // --- B Button: Gulper motors ---
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
            if (!gulperReverse) {
                gulperMotor1.setPower(gulperForwardPower);
                gulperMotor2.setPower(gulperForwardPower);
            }
            else {
                gulperMotor1.setPower(gulperReversePower);
                gulperMotor2.setPower(gulperReversePower);
            }
        }
        else {
            gulperMotor1.setPower(gulperOffPower);
            gulperMotor2.setPower(gulperOffPower);
        }
        telemetry.addData("Gulper Reverse", gulperReverse);
        telemetry.addData("Gulpers running", runGulper);

        // --- Y Button: Toggle Winch Power (for safety) ---
        if (this.gamepad2.y) {
            if (!gamepad2YToggleLock) {
                gamepad2YToggleLock = true;
                winchesPowered = !winchesPowered;
            }
        }
        else {
            gamepad2YToggleLock = false;
        }
        if (winchesPowered) {
            winchMotor1.setPower(winchMotor1Power);
            winchMotor2.setPower(winchMotor2Power);
        }
        else {
            winchMotor1.setPower(0);
            winchMotor2.setPower(0);
        }
        telemetry.addData("Winches Powered", winchesPowered);

        // --- Left/Right Bumpers: Winch motors ---
        if (this.gamepad2.left_bumper) {
            if (!gamepad2BumperToggleLock) {
                gamepad2BumperToggleLock = true;
                if (currentWinchIndex > 0) {
                    currentWinchIndex -= 1;
                    winchMotor1.setTargetPosition(winchMotorPositions[currentWinchIndex] + winchMotor1Offsets[currentWinchIndex]);
                    winchMotor2.setTargetPosition(winchMotorPositions[currentWinchIndex] + winchMotor2Offsets[currentWinchIndex]);
                }
            }
        }
        else if (this.gamepad2.right_bumper) {
            if (!gamepad2BumperToggleLock) {
                gamepad2BumperToggleLock = true;
                if (currentWinchIndex < winchMotorPositions.length - 1) {
                    currentWinchIndex += 1;
                    winchMotor1.setTargetPosition(winchMotorPositions[currentWinchIndex] + winchMotor1Offsets[currentWinchIndex]);
                    winchMotor2.setTargetPosition(winchMotorPositions[currentWinchIndex] + winchMotor2Offsets[currentWinchIndex]);
                }
            }
        }
        else {
            gamepad2BumperToggleLock = false;
        }

        // --- Left Trigger: Set winch all the way down ---
        if (this.gamepad2.left_trigger > 0.5) {
            if (!gamepad2LeftTriggerToggleLock) {
                gamepad2LeftTriggerToggleLock = true;
                currentWinchIndex  = 0;
                winchMotor1.setTargetPosition(winchMotorPositions[currentWinchIndex] + winchMotor1Offsets[currentWinchIndex]);
                winchMotor2.setTargetPosition(winchMotorPositions[currentWinchIndex] + winchMotor2Offsets[currentWinchIndex]);
            }
        }
        else {
            gamepad2LeftTriggerToggleLock = false;
        }

        telemetry.addData("Winch Index", currentWinchIndex);
        telemetry.addData("Winch 1 Target", winchMotor1.getTargetPosition());
        telemetry.addData("Winch 2 Target", winchMotor2.getTargetPosition());
        telemetry.addData("Winch 1 Current", winchMotor1.getCurrentPosition());
        telemetry.addData("Winch 2 Current", winchMotor2.getCurrentPosition());

        // --- X Button: Block Gripper Servo ---
        if (this.gamepad2.x) {
            if (!gamepad2XToggleLock) {
                gamepad2XToggleLock = true;
                blockServoOpen = !blockServoOpen;
            }
        }
        else {
            gamepad2XToggleLock = false;
        }
        if (blockServoOpen) {
            blockServo.setPosition(blockServoOpenPosition);
        }
        else {
            blockServo.setPosition(blockServoClosedPosition);
        }
        telemetry.addData("Block Servo Opened", blockServoOpen);
        telemetry.addData("Block Servo Position", blockServo.getPosition());

        // --- Right Trigger: Build Plate Clamper Servos ---
        if (this.gamepad2.right_trigger > 0.5) {
            if (!gamepad2RightTriggerToggleLock) {
                gamepad2RightTriggerToggleLock = true;
                plateServosUp = !plateServosUp;
            }
        }
        else {
            gamepad2RightTriggerToggleLock = false;
        }
        if (plateServosUp) {
            plateServoLeft.setPosition(plateServoLeftUp);
            plateServoRight.setPosition(plateServoRightUp);
        }
        else {
            plateServoLeft.setPosition(plateServoLeftDown);
            plateServoRight.setPosition(plateServoRightDown);
        }
        telemetry.addData("Build plate servos up", plateServosUp);

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
