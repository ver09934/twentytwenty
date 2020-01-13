package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// import static org.firstinspires.ftc.teamcode.DriverFunction.MIN_SPEED_RATIO;
import static org.firstinspires.ftc.teamcode.DriverFunction.NORMAL_SPEED_RATIO;
import static org.firstinspires.ftc.teamcode.DriverFunction.MAX_SPEED_RATIO;

@TeleOp(name="Skystone Tele-Op", group="TeleOp OpMode")
public class SkystoneTeleOp extends OpMode {

    // TODO: Release capstone (gamepad 2 x)
        // Just servo stuff
    // TODO: Adjust slide offset (gamepad 2 left stick)
        // Simply adjust winchMotorsInitialPosition, and then
        // call genWinchIndices() again
        // Make 1 step ~5-10 encoder ticks?

    // Important things
    private ElapsedTime runtime = new ElapsedTime();
    private DriverFunction driverFunction;
    private DriverFunction.Steering steering;

    // Driving variables
    private final static double TURNING_SPEED_BOOST = 0.3;
    private final static double MIN_SPEED_RATIO = 0.1;

    // DC Motors
    DcMotor gulperMotor1;
    DcMotor gulperMotor2;
    DcMotor winchMotor1;
    DcMotor winchMotor2;

    // Servos
    private Servo blockServoLeft;
    private Servo blockServoRight;
    private Servo plateServoLeft;
    private Servo plateServoRight;
    private Servo autonGrabberLeft;
    private Servo autonGrabberRight;

    // Gamepad 1 Toggle locks
    private boolean gamepad1XToggleLock = false;
    private boolean gamepad1YToggleLock = false;

    // Gamepad 2 Toggle Locks
    private boolean gamepad2BToggleLock = false;
    private boolean gamepad2DPadUpDownToggleLock = false;
    private boolean gamepad2DPadRightLeftToggleLock = false;
    private boolean gamepad2RightBumperToggleLock = false;
    private boolean gamepad2LeftBumperToggleLock = false;
    private boolean gamepad2LeftStickToggleLock = false;

    // Boolean state variables
    private boolean driveDirectionReverse = false;
    private boolean winchesPowered = false;
    private boolean plateServosUp = false;
    private boolean blockServoOpen = false;
    private boolean gulpersForwards = false;
    private boolean gulpersReverse = false;

    // Winch values
    private double winchPower = 0.6;
    private double winchMotor1Power = winchPower;
    private double winchMotor2Power = winchPower;
    private int winchMotorsInitialPosition = 0;
    private int foundationHeight = 500;
    private int originalFoundationHeight = foundationHeight;
    private int winchMotorStep = 725;
    private int[] winchMotorSteps = {
            foundationHeight,
            winchMotorStep,
            winchMotorStep,
            winchMotorStep,
            winchMotorStep,
            winchMotorStep,
            winchMotorStep,
            winchMotorStep,
    };
    private int[] winchMotor1Offsets = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    private int[] winchMotor2Offsets = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    private int[] winchMotorPositions = new int[winchMotorSteps.length + 1];
    private int currentWinchIndex = 0;

    private int winchAdjustIncrement = 50;

    // Winch methods
    private void genWinchIndices() {
        assert(winchMotorPositions.length == winchMotor1Offsets.length);
        assert(winchMotorPositions.length == winchMotor2Offsets.length);
        winchMotorPositions[0] = winchMotorsInitialPosition;
        for (int i = 1; i < winchMotorPositions.length; i++) {
            winchMotorPositions[i] = winchMotorPositions[i - 1];
            winchMotorPositions[i] += winchMotorSteps[i - 1];
        }
    }

    private void updateWinchPositions() {
        winchMotor1.setTargetPosition(winchMotorPositions[currentWinchIndex] + winchMotor1Offsets[currentWinchIndex]);
        winchMotor2.setTargetPosition(winchMotorPositions[currentWinchIndex] + winchMotor2Offsets[currentWinchIndex]);
    }

    private void unpowerWinches() {
        winchMotor1.setPower(0);
        winchMotor2.setPower(0);
        winchesPowered = false;
    }

    private void powerWinches() {
        winchMotor1.setPower(winchMotor1Power);
        winchMotor2.setPower(winchMotor2Power);
        winchesPowered = true;
    }

    // Gulper motor vars and methods
    private double gulperForwardPower = 0.9;
    private double gulperReversePower = -gulperForwardPower;
    private double gulperOffPower = 0;

    // Gulper input things
    private boolean gulpersScaling = false;
    private double gulpersInput = 0;

    // Block servo positions
    // public static final double blockServoLeftReallyClosedPosition = 0;
    public static final double blockServoLeftClosedPosition = 0.08;
    public static final double blockServoLeftOpenPosition = 0.45;
    public static final double blockServoLeftAutonPosition = 0.82;
    // public static final double blockServoRightReallyClosedPosition = 1;
    public static final double blockServoRightClosedPosition = 0.92;
    public static final double blockServoRightOpenPosition = 0.54;
    public static final double blockServoRightAutonPosition = 0.16;

    // Build plate servo positions
    public static final double plateServoLeftDown = 0.07;
    public static final double plateServoLeftUp = 0.67;
    public static final double plateServoRightDown = 0;
    public static final double plateServoRightUp = 0.53;

    // Auton grabber positions
    public static final double autonGrabberLeftPassive = 0;
    public static final double autonGrabberLeftActive = 0.5;
    public static final double autonGrabberRightPassive = 0.8;
    public static final double autonGrabberRightActive = 0.3;

    // Auton grabber method
    public void retractBothAutonGrabbers() {
        autonGrabberLeft.setPosition(autonGrabberLeftPassive);
        autonGrabberRight.setPosition(autonGrabberRightPassive);
    }

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {

        driverFunction = new DriverFunction(hardwareMap, telemetry);
        steering = driverFunction.getSteering();

        gulperMotor1 = hardwareMap.dcMotor.get("intake1");
        gulperMotor2 = hardwareMap.dcMotor.get("intake2");
        winchMotor1 = hardwareMap.dcMotor.get("winch1");
        winchMotor2 = hardwareMap.dcMotor.get("winch2");

        genWinchIndices();
        updateWinchPositions();

        gulperMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        gulperMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        winchMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        winchMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

        unpowerWinches();

        blockServoLeft = hardwareMap.servo.get("blockServoLeft");
        blockServoRight = hardwareMap.servo.get("blockServoRight");
        plateServoLeft = hardwareMap.servo.get("plateServoLeft");
        plateServoRight = hardwareMap.servo.get("plateServoRight");
        autonGrabberLeft = hardwareMap.servo.get("autonGrabberLeft");
        autonGrabberRight = hardwareMap.servo.get("autonGrabberRight");

        blockServoOpen = false;
        blockServoLeft.setPosition(blockServoLeftClosedPosition);
        blockServoRight.setPosition(blockServoRightClosedPosition);

        plateServosUp = false;
        plateServoLeft.setPosition(plateServoLeftDown);
        plateServoRight.setPosition(plateServoRightDown);

        retractBothAutonGrabbers();

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
        powerWinches();
        updateWinchPositions();
    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {

        // --------------------------------------------------
        // ---------- Gamepad 1: Driving Functions ----------
        // --------------------------------------------------

        // Y Button: Toggle build plate clamper servos
        if (this.gamepad1.y) {
            if (!gamepad1YToggleLock) {
                gamepad1YToggleLock = true;
                plateServosUp = !plateServosUp;
            }
        }
        else {
            gamepad1YToggleLock = false;
        }
        if (plateServosUp) {
            plateServoLeft.setPosition(plateServoLeftUp);
            plateServoRight.setPosition(plateServoRightUp);
        }
        else {
            plateServoLeft.setPosition(plateServoLeftDown);
            plateServoRight.setPosition(plateServoRightDown);
        }
        telemetry.addData("Build plate servos down", !plateServosUp);

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
        else if (this.gamepad1.right_bumper) {
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

        // Right/Left Bumpers: Toggle gulpers forwards/reverse
        if (this.gamepad2.right_bumper) {
            if (!gamepad2RightBumperToggleLock) {
                gamepad2RightBumperToggleLock = true;
                if (gulpersReverse) {
                    gulpersReverse = false;
                }
                gulpersForwards = !gulpersForwards;
            }
        }
        else {
            gamepad2RightBumperToggleLock = false;
        }
        if (this.gamepad2.left_bumper) {
            if (!gamepad2LeftBumperToggleLock) {
                gamepad2LeftBumperToggleLock = true;
                if (gulpersForwards) {
                    gulpersForwards = false;
                }
                gulpersReverse = !gulpersReverse;
            }
        }
        else {
            gamepad2LeftBumperToggleLock = false;
        }

        // --- Right Trigger: Take in blocks ---
        // --- Left Trigger: Push out blocks ---
        double enableGulpersThresh = 0.01;
        double maxGulperInput = 1;
        if (this.gamepad2.right_trigger > enableGulpersThresh) {
            gulpersScaling = true;
            gulpersInput = this.gamepad2.right_trigger;
        }
        else if (this.gamepad2.left_trigger > enableGulpersThresh) {
            gulpersScaling = true;
            gulpersInput = -this.gamepad2.left_trigger;
        }
        else if (this.gamepad2.right_stick_y < -enableGulpersThresh) {
            gulpersScaling = true;
            gulpersInput = this.gamepad2.right_stick_y;
        }
        else if (this.gamepad2.right_stick_y > enableGulpersThresh) {
            gulpersScaling = true;
            gulpersInput = this.gamepad2.right_stick_y;
        }
        else {
            gulpersScaling = false;
        }

        // Gulper logic
        if (gulpersScaling) {
            gulpersForwards = false;
            gulpersReverse = false;
        }

        // Write powers to gulpers
        if (gulpersScaling) {
            double power;
            if (gulpersInput > 0) {
                power = gulperForwardPower * (Math.abs(gulpersInput) / maxGulperInput);
                telemetry.addData("Gulpers", "Forwards");
            }
            else {
                power = gulperReversePower * (Math.abs(gulpersInput) / maxGulperInput);
                telemetry.addData("Gulpers", "Reverse");
            }
            gulperMotor1.setPower(power);
            gulperMotor2.setPower(power);
        }
        else if (gulpersForwards) {
            gulperMotor1.setPower(gulperForwardPower);
            gulperMotor2.setPower(gulperForwardPower);
            telemetry.addData("Gulpers", "Forwards");
        }
        else if (gulpersReverse) {
            gulperMotor1.setPower(gulperReversePower);
            gulperMotor2.setPower(gulperReversePower);
            telemetry.addData("Gulpers", "Reverse");
        }
        else {
            gulperMotor1.setPower(gulperOffPower);
            gulperMotor2.setPower(gulperOffPower);
            telemetry.addData("Gulpers", "Off");
        }

        // --- D-Pad Up/Down: Winch motors up/down ---
        if (this.gamepad2.dpad_down) {
            if (!gamepad2DPadUpDownToggleLock) {
                gamepad2DPadUpDownToggleLock = true;
                if (currentWinchIndex > 0) {
                    currentWinchIndex -= 1;
                    updateWinchPositions();
                }
            }
        }
        else if (this.gamepad2.dpad_up) {
            if (!gamepad2DPadUpDownToggleLock) {
                gamepad2DPadUpDownToggleLock = true;
                if (currentWinchIndex < winchMotorPositions.length - 1) {
                    currentWinchIndex += 1;
                    updateWinchPositions();
                }
            }
        }
        else {
            gamepad2DPadUpDownToggleLock = false;
        }

        // --- D-Pad Right/Left: Winch motors reset ---
        if (this.gamepad2.dpad_left || this.gamepad2.dpad_right) {
            if (!gamepad2DPadRightLeftToggleLock) {
                gamepad2DPadRightLeftToggleLock = true;
                currentWinchIndex  = 0;
                updateWinchPositions();
                blockServoLeft.setPosition(blockServoLeftClosedPosition);
                blockServoRight.setPosition(blockServoRightClosedPosition);
                blockServoOpen = false;
            }
        }
        else {
            gamepad2DPadRightLeftToggleLock = false;
        }

        double leftGamepadThresh = 0.4;
        if (this.gamepad2.left_stick_y > leftGamepadThresh) {
            if (!gamepad2LeftStickToggleLock) {
                gamepad2LeftStickToggleLock = true;
                foundationHeight += winchAdjustIncrement;
                genWinchIndices();
                updateWinchPositions();
            }
        }
        else if (this.gamepad2.left_stick_y < -leftGamepadThresh) {
            if (!gamepad2LeftStickToggleLock) {
                gamepad2LeftStickToggleLock = true;
                foundationHeight -= winchAdjustIncrement;
                genWinchIndices();
                updateWinchPositions();
            }
        }
        else {
            gamepad2LeftStickToggleLock = false;
        }

        // Winch position telemetry
        telemetry.addData("Winch Index", currentWinchIndex);
        telemetry.addData("Winch Offset", foundationHeight - originalFoundationHeight);
        telemetry.addData("Winch 1 Target", winchMotor1.getTargetPosition());
        telemetry.addData("Winch 2 Target", winchMotor2.getTargetPosition());
        telemetry.addData("Winch 1 Current", winchMotor1.getCurrentPosition());
        telemetry.addData("Winch 2 Current", winchMotor2.getCurrentPosition());

        // --- B Button: Block Gripper Servo ---
        if (this.gamepad2.b) {
            if (!gamepad2BToggleLock) {
                gamepad2BToggleLock = true;
                blockServoOpen = !blockServoOpen;
            }
        }
        else {
            gamepad2BToggleLock = false;
        }
        if (blockServoOpen) {
            blockServoLeft.setPosition(blockServoLeftOpenPosition);
            blockServoRight.setPosition(blockServoRightOpenPosition);
        }
        else {
            blockServoLeft.setPosition(blockServoLeftClosedPosition);
            blockServoRight.setPosition(blockServoRightClosedPosition);
        }
        telemetry.addData("Block Servo Opened", blockServoOpen);

        // Finish steering, putting power into hardware
        steering.finishSteering();

        // Motor position telemetry for testing
        telemetry.addLine("-----------------------------");
        telemetry.addData("LF Position", driverFunction.lf.getPosition());
        telemetry.addData("LB Position", driverFunction.lb.getPosition());
        telemetry.addData("RF Position", driverFunction.rf.getPosition());
        telemetry.addData("RB Position", driverFunction.rb.getPosition());
        telemetry.addLine("-----------------------------");

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
