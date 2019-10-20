package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class TestMethods {

    HardwareMap hardwareMap;
    Telemetry telemetry;
    DriveTrain drivetrain;
    Sensors sensors;

    TestMethods(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.sensors = new Sensors(hardwareMap);
        this.drivetrain = new DriveTrain(hardwareMap);
        drivetrain.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drivetrain.setZeroPowerBehaviors(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // -------------------- START TESTS --------------------

    public void moveForwards(double power, double distance) {
        drivetrain.resetEncoders();
        while ((drivetrain.getLfPosition() + drivetrain.getRfPosition() + drivetrain.getLbPosition() + drivetrain.getRbPosition()) / 4.0 < distance) {
            drivetrain.setAllPowers(power);
        }
        drivetrain.setAllPowers(0);
    }

    public static void moveAlongWall(double walldist, double stopdist) {}
    public static void getDistance(int distIndex) {}
    public static void move(double angle, double velocity) {}

    // -------------------- END TESTS --------------------

    static class Sensors {

        private DistanceSensor sideBack;
        private DistanceSensor sideFront;
        private DistanceSensor back;
        private DistanceSensor front;

        private DistanceUnit unit = DistanceUnit.CM;

        Sensors(HardwareMap hardwareMap) {
            sideBack = hardwareMap.get(DistanceSensor.class, "sideBack");
            sideFront = hardwareMap.get(DistanceSensor.class, "sideFront");
            back = hardwareMap.get(DistanceSensor.class, "back");
            front = hardwareMap.get(DistanceSensor.class, "front");
        }

        public void setDistanceUnit(DistanceUnit unit) {
            this.unit = unit;
        }

        public double getSideBackDistance() {
            return sideBack.getDistance(unit);
        }

        public double getSideFrontDistance() {
            return sideFront.getDistance(unit);
        }

        public double getBackDistance() {
            return back.getDistance(unit);
        }

        public double getFrontDistance() {
            return front.getDistance(unit);
        }
    }

    static class DriveTrain {

        MotorWrapper lfMotor;
        MotorWrapper rfMotor;
        MotorWrapper lbMotor;
        MotorWrapper rbMotor;

        public DriveTrain(HardwareMap hardwareMap) {
            lfMotor = new MotorWrapper(hardwareMap.dcMotor.get("lfMotor"));
            rfMotor = new MotorWrapper(hardwareMap.dcMotor.get("rfMotor"));
            lbMotor = new MotorWrapper(hardwareMap.dcMotor.get("lbMotor"));
            rbMotor = new MotorWrapper(hardwareMap.dcMotor.get("rbMotor"));
        }

        public void setRunMode(DcMotor.RunMode runMode) {
            lfMotor.setMode(runMode);
            rfMotor.setMode(runMode);
            lbMotor.setMode(runMode);
            rbMotor.setMode(runMode);
        }

        public void resetEncoders() {
            lfMotor.resetEncoder();
            rfMotor.resetEncoder();
            lbMotor.resetEncoder();
            rbMotor.resetEncoder();
        }

        public void setZeroPowerBehaviors(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
            lfMotor.setZeroPowerBehavior(zeroPowerBehavior);
            rfMotor.setZeroPowerBehavior(zeroPowerBehavior);
            lbMotor.setZeroPowerBehavior(zeroPowerBehavior);
            rbMotor.setZeroPowerBehavior(zeroPowerBehavior);
        }

        public void setAllPowers(double power) {
            lfMotor.setPower(power);
            rfMotor.setPower(power);
            lbMotor.setPower(power);
            rbMotor.setPower(power);
        }

        public int getLfPosition() {
            return lfMotor.getCurrentPosition();
        }
        public int getRfPosition() {
            return rfMotor.getCurrentPosition();
        }
        public int getLbPosition() {
            return lfMotor.getCurrentPosition();
        }
        public int getRbPosition() {
            return rbMotor.getCurrentPosition();
        }

        class MotorWrapper {

            DcMotor motor;

            MotorWrapper(DcMotor motor) {
                this.motor = motor;
            }

            public void setMode(DcMotor.RunMode runMode) {
                motor.setMode(runMode);
            }

            public void resetEncoder() {
                DcMotor.RunMode currentMode = motor.getMode();
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(currentMode);
            }

            public int getCurrentPosition() {
                return motor.getCurrentPosition();
            }

            public void setPower(double power) {
                motor.setPower(power);
            }

            public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
                motor.setZeroPowerBehavior(zeroPowerBehavior);
            }
        }
    }
}
