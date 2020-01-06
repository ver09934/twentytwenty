package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.*;

import java.util.ArrayList;
import java.util.Collections;

@Autonomous(name = "Skystone Auton")
public class SkystoneAuton extends LinearOpMode {
    // TODO: Figure out how to reverse everything
    // TODO: Read android app settings for field position (check Seth's 2019 commits)
    // TODO: public servo autonGrabberRight
    // TODO: Set block servo to open or wherever...

    // TODO: Add optional direction forcing to angle turn method

    private ElapsedTime runtime;
    Motors motors = new Motors(this);
    Servos servos = new Servos(this, allianceColor);
    Sensors sensors = new Sensors(this, allianceColor);
    Movement movement = new Movement(this, motors, allianceColor, sensors);


    @Override
    public void runOpMode() {

        // ----- INIT SECTION -----

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        runtime = new ElapsedTime();

        motors.initMotors();
        servos.initServos();
        sensors.initColorSensors();
        sensors.initIMU();
        initSharedPreferences();

        while (!this.isStarted()) {
            telemetry.addData("Status", "Waiting for start");
            telemetry.addData("Runtime", runtime.toString());
            telemetry.addData("IMU calibration status", sensors.imu.getCalibrationStatus().toString());
            telemetry.update();
        }

        // ----- RUN SECTION -----

        if (autonType == AutonType.TWOSKYSTONES) {
            bothBlocksAuton();
        } else if (autonType == AutonType.FOUNDATION) {
            plateAuton();
        }

        // sharedPrefsTest();

        while (opModeIsActive()) {
            idle();
        }
    }

    // ----- META-METHODS -----

    public void plateAuton() {

        double bigpow = 0.85;
        double medpow = 0.5;
        double pow = 0.35;

        servos.plateServosUp();

        movement.moveCardinal(bigpow, motors.inchesToCm(12), 180);

        movement.moveCardinal(bigpow, motors.inchesToCm(29), 90);
        movement.moveCardinal(pow, motors.inchesToCm(2), 90);

        servos.plateServosDown();

        if (allianceColor == allianceColor.BLUE) {
            movement.gotoDegreesRamping(pow, 270);
        } else if (allianceColor == allianceColor.RED) {
            movement.gotoDegreesRamping(pow, 90);
        }
        movement.gotoDegreesRamping(pow, 180);

        // moveCardinal(bigpow, inchesToCm(12), 90);

        servos.plateServosUp();

        movement.moveCardinal(pow, motors.inchesToCm(1), 270);

        movement.makeStraight();

        movement.moveCardinal(bigpow, motors.inchesToCm(30), 180);

        movement.makeStraight();

        movement.moveCardinal(bigpow, motors.inchesToCm(30), 90);

        movement.makeStraight();

        movement.moveCardinal(bigpow, motors.inchesToCm(18), 180);
    }

    public void bothBlocksAuton() {

        double bigpow = 0.925;
        double medpow = 0.7;
        double pow = 0.35;
        double tinypow = 0.2;

        movement.moveCardinal(medpow, motors.inchesToCm(27), 270);

        ArrayList values = new ArrayList<Double>();

        // Scan blocks
        for (int i = 0; i < 3; i++) {

            values.add(sensors.getHSV()[2]);

            if (i < 2) {
                movement.moveCardinal(pow, motors.inchesToCm(8), 180);
            }
        }

        int minIndex = values.indexOf(Collections.min(values));

        telemetry.addData("Values", values);
        telemetry.addData("Min Index", minIndex);
        telemetry.update();

        // Move back to the first black block
        double extraDistOne = 0;
        movement.moveCardinal(pow, motors.inchesToCm(extraDistOne + (values.size() - minIndex - 1) * 8), 0);

        double blockGetPart1Dist = 1;
        double blockGetPart2Dist = 3;
        double backupDistance = 10;

        double middleDistance = 20;
        double otherSideDistance = 20;
        double totalOtherSideDistance = middleDistance + otherSideDistance;

        double blockSize = 8;

        // Get block and back up
        movement.moveCardinal(tinypow, motors.inchesToCm(blockGetPart1Dist), 270);
        servos.deploySkystoneGrabber();
        movement.moveCardinal(tinypow, motors.inchesToCm(blockGetPart2Dist), 270);
        movement.moveCardinal(bigpow, motors.inchesToCm(blockGetPart1Dist + blockGetPart2Dist + backupDistance), 90);

        // Go to other side of field and release block
        movement.moveCardinal(bigpow, motors.inchesToCm(totalOtherSideDistance + minIndex * blockSize), 0);
        servos.retractSkystoneGrabber();

        movement.makeStraight(); // Align

        double extraDistTwo = 1;

        // Go back to other block
        movement.moveCardinal(bigpow, motors.inchesToCm(totalOtherSideDistance + (3 + minIndex) * blockSize + extraDistTwo), 180);
        movement.makeStraight(); // Align
        movement.moveCardinal(bigpow, motors.inchesToCm(backupDistance), 270);

        // Get block and back up
        movement.moveCardinal(tinypow, motors.inchesToCm(blockGetPart1Dist), 270);
        servos.deploySkystoneGrabber();
        movement.moveCardinal(tinypow, motors.inchesToCm(blockGetPart2Dist), 270);
        movement.moveCardinal(bigpow, motors.inchesToCm(blockGetPart1Dist + blockGetPart2Dist + backupDistance), 90);

        movement.makeStraight(); // Align

        // Go to other side of field and release block
        movement.moveCardinal(bigpow, motors.inchesToCm(totalOtherSideDistance + (3 + minIndex) * blockSize + extraDistTwo), 0);
        servos.retractSkystoneGrabber();

        // Park
        movement.moveCardinal(bigpow, motors.inchesToCm(otherSideDistance), 180);
    }

    // ----- TEST META-METHODS -----

    public void sharedPrefsTest() {
        if (allianceColor == AllianceColor.BLUE) {
            movement.moveCardinal(0.5, 12, 90);
        } else if (allianceColor == AllianceColor.RED) {
            movement.moveCardinal(0.5, 12, 270);
        }

        sleep(500);

        if (autonType == AutonType.TWOSKYSTONES) {
            movement.moveCardinal(0.5, 12, 0);
        } else if (autonType == AutonType.FOUNDATION) {
            movement.moveCardinal(0.5, 12, 180);
        }
    }

    public void testMoveOne() {
        for (int j = 0; j < 1; j++) {
            for (int i = 0; i < 360; i += 90) {
                movement.gotoDegreesRamping(0.5, i);
                sleep(1000);
            }
        }

        while (opModeIsActive()) {
            // makeStraight(0.5);
            movement.makeStraight();
            sleep(5000);
        }
    }

    public void testMoveTwo() {
        for (int i = 0; i < 360; i += 90) {
            movement.moveCardinal(1, motors.inchesToCm(30), i);
            sleep(1000);
        }

        for (int i = 0; i < 360; i += 90) {
            movement.moveCardinal(0.5, motors.inchesToCm(12), i);
            sleep(1000);
        }

        for (int i = 0; i < 360; i += 90) {
            movement.moveCardinal(0.3, motors.inchesToCm(4), i);
            sleep(1000);
        }
    }

    // ----- ANDROID SHARED PREFERENCES -----

    private static SharedPreferences sharedPrefs;

    private static AllianceColor allianceColor;
    private static AutonType autonType;

    public void initSharedPreferences() {
        sharedPrefs = PreferenceManager.getDefaultSharedPreferences(this.hardwareMap.appContext);

        String color = sharedPrefs.getString("alliance_color", "ERROR");
        String type = sharedPrefs.getString("auton_type", "ERROR");

        if (color.equals("BLUE")) {
            allianceColor = AllianceColor.BLUE;
        } else if (color.equals("RED")) {
            allianceColor = AllianceColor.RED;
        }

        if (type.equals("TWOSKYSTONES")) {
            autonType = AutonType.TWOSKYSTONES;
        } else if (type.equals("FOUNDATION")) {
            autonType = AutonType.FOUNDATION;
        }
    }

    public enum AllianceColor {
        BLUE, RED
    }

    public enum AutonType {
        TWOSKYSTONES, FOUNDATION
    }


}
