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

    // public boolean xToggleLock = false;
    public boolean bToggleLock = false;

    // public boolean xyMethod = false;
    public boolean turnMethod = false;

    public double getMin(double[] arr) {
        double min = arr[0];
        for (int i = 1; i < arr.length; i++) {
            if (arr[i] < min) {
                min = arr[i];
            }
        }
        return min;
    }

    public double getMax(double[] arr) {
        double max = arr[0];
        for (int i = 1; i < arr.length; i++) {
            if (arr[i] > max) {
                max = arr[i];
            }
        }
        return max;
    }

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

    /*
    Input parameters:
    - x velocity (-1 to 1)
    - y velocity (-1 to 1)
    - turning speed (-1 to 1)
    - speed scale (0 to 1)

    double maxPossiblePower = Math.sqrt(2);
    driveAngle = Math.atan2(xInput, yInput);
    driveSpeed = Math.sqrt(Math.pow(xInput, 2) + Math.pow(yInput, 2));
    double x = Math.cos(driveAngle);
    double y = Math.sin(driveAngle);

    double powerLF = x - y;
    double powerRF = x + y;
    double powerLB = -x - y;
    double powerRB = -x + y;
    */

    double x = 0;
    double y = 0;
    double turnSpeed = 0;

    @Override
    public void loop() {

        /*
        if (gamepad1.x) {
            if (!xToggleLock) {
                xToggleLock = true;
                xyMethod = !xyMethod;
            }
        }
        else {
            xToggleLock = false;
        }
        */

        if (gamepad1.b) {
            if (!bToggleLock) {
                bToggleLock = true;
                turnMethod = !turnMethod;
            }
        }
        else {
            bToggleLock = false;
        }

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

        double driveSpeed = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        double driveAngle = Math.atan2(y, x);

        if (driveSpeed > 1) {
            driveSpeed = Math.min(driveSpeed, 1);
            x = driveSpeed * Math.cos(Math.toRadians(driveAngle));
            y = driveSpeed * Math.sin(Math.toRadians(driveAngle));
        }
        
        // double maxX = Math.abs(Math.cos(Math.toRadians(driveAngle)));
        // double maxY = Math.abs(Math.sin(Math.toRadians(driveAngle)));
        // double maxPower = maxX + maxY;

        x /= Math.sqrt(2);
        y /= Math.sqrt(2);

        /*
        if (!xyMethod) {

            // Speed scaling option 1:
            // Divide by max possible speed
            x /= Math.sqrt(2);
            y /= Math.sqrt(2);

        }
        else {

            // Speed scaling option 2:
            // Scale speeds by current x and y
            x /= maxDriveSpeed;
            y /= maxDriveSpeed;

        }
        */

        double[] motorPowers = {
            x - y,
            x + y,
            -x - y,
            -x + y
        };

        if (!turnMethod) {

            // Turn incorporation option 1:
            // Scale such that max turn weight is 0.5

            for (int i = 0; i < motorPowers.length; i++) {
                motorPowers[i] += turnSpeed;
            }
            double maxSpeed = getMax(motorPowers);
            if (maxSpeed > 1) {
                for (int i = 0; i < motorPowers.length; i++) {
                    motorPowers[i] /= maxSpeed;
                }
            }

        }
        else {

            // Turn incorporation option 2:
            // Prioritize turning

            double minPower = getMin(motorPowers);
            double maxPower = getMax(motorPowers);

            double tempMax = Math.max(Math.abs(minPower), Math.abs(maxPower));
            double tempCorrectedMax = Math.max(Math.abs(minPower + turnSpeed), Math.abs(maxPower + turnSpeed));

            if (tempCorrectedMax > 1) {
                for (int i = 0; i < motorPowers.length; i++) {
                    motorPowers[i] *= (1 - Math.abs(turnSpeed)) / tempMax;
                }
            }

            for (int i = 0; i < motorPowers.length; i++) {
                motorPowers[i] += turnSpeed;
            }

        }

        lfMotor.setPower(motorPowers[0]);
        rfMotor.setPower(motorPowers[1]);
        lbMotor.setPower(motorPowers[2]);
        rbMotor.setPower(motorPowers[3]);

        // telemetry.addData("XY Method (X Button)", !xyMethod ? "1" : "2");
        telemetry.addData("Turn Method (B Button)", !turnMethod ? "1" : "2");
        telemetry.addData("Runtime", runtime.toString());
        telemetry.update();
    }
}
