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

    @Override
    public void runOpMode() {

        // ----- INIT SECTION -----

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        runtime = new ElapsedTime();

        initMotors();
        initServos();
        initColorSensors();
        initIMU();
        initSharedPreferences();

        while (!this.isStarted()) {
            telemetry.addData("Status", "Waiting for start");
            telemetry.addData("Runtime", runtime.toString());
            telemetry.addData("IMU calibration status", imu.getCalibrationStatus().toString());
            telemetry.update();
        }

        // ----- RUN SECTION -----

        if (autonType == AutonType.TWOSKYSTONES) {
            bothBlocksAuton();
        }
        else if (autonType == AutonType.FOUNDATION) {
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

        plateServosUp();

        moveCardinal(bigpow, inchesToCm(12), 180);

        moveCardinal(bigpow, inchesToCm(29), 90);
        moveCardinal(pow, inchesToCm(2), 90);

        plateServosDown();

        if (allianceColor == allianceColor.BLUE) {
            gotoDegreesRamping(pow, 270);
        }
        else if (allianceColor == allianceColor.RED) {
            gotoDegreesRamping(pow, 90);
        }
        gotoDegreesRamping(pow, 180);

        // moveCardinal(bigpow, inchesToCm(12), 90);

        plateServosUp();

        moveCardinal(pow, inchesToCm(1), 270);

        makeStraight();

        moveCardinal(bigpow, inchesToCm(30), 180);

        makeStraight();

        moveCardinal(bigpow, inchesToCm(30), 90);

        makeStraight();

        moveCardinal(bigpow, inchesToCm(18), 180);
    }

    public void bothBlocksAuton() {

        double bigpow = 0.925;
        double medpow = 0.7;
        double pow = 0.35;
        double tinypow = 0.2;

        moveCardinal(medpow, inchesToCm(27), 270);

        ArrayList values = new ArrayList<Double>();

        // Scan blocks
        for (int i = 0; i < 3; i++) {

            values.add(getHSV()[2]);

            if (i < 2) {
                moveCardinal(pow, inchesToCm(8), 180);
            }
        }

        int minIndex = values.indexOf(Collections.min(values));

        telemetry.addData("Values", values);
        telemetry.addData("Min Index", minIndex);
        telemetry.update();

        // Move back to the first black block
        double extraDistOne = 0;
        moveCardinal(pow, inchesToCm(extraDistOne + (values.size() - minIndex - 1) * 8), 0);

        double blockGetPart1Dist = 1;
        double blockGetPart2Dist = 3;
        double backupDistance = 10;

        double middleDistance = 20;
        double otherSideDistance = 20;
        double totalOtherSideDistance = middleDistance + otherSideDistance;

        double blockSize = 8;

        // Get block and back up
        moveCardinal(tinypow, inchesToCm(blockGetPart1Dist), 270);
        deploySkystoneGrabber();
        moveCardinal(tinypow, inchesToCm(blockGetPart2Dist), 270);
        moveCardinal(bigpow, inchesToCm(blockGetPart1Dist + blockGetPart2Dist + backupDistance), 90);

        // Go to other side of field and release block
        moveCardinal(bigpow, inchesToCm(totalOtherSideDistance + minIndex * blockSize), 0);
        retractSkystoneGrabber();

        makeStraight(); // Align

        double extraDistTwo = 1;

        // Go back to other block
        moveCardinal(bigpow, inchesToCm(totalOtherSideDistance + (3 + minIndex) * blockSize + extraDistTwo), 180);
        makeStraight(); // Align
        moveCardinal(bigpow, inchesToCm(backupDistance), 270);

        // Get block and back up
        moveCardinal(tinypow, inchesToCm(blockGetPart1Dist), 270);
        deploySkystoneGrabber();
        moveCardinal(tinypow, inchesToCm(blockGetPart2Dist), 270);
        moveCardinal(bigpow, inchesToCm(blockGetPart1Dist + blockGetPart2Dist + backupDistance), 90);

        makeStraight(); // Align

        // Go to other side of field and release block
        moveCardinal(bigpow, inchesToCm(totalOtherSideDistance + (3 + minIndex) * blockSize + extraDistTwo), 0);
        retractSkystoneGrabber();

        // Park
        moveCardinal(bigpow, inchesToCm(otherSideDistance), 180);
    }

    // ----- TEST META-METHODS -----

    public void sharedPrefsTest() {
        if (allianceColor == AllianceColor.BLUE) {
            moveCardinal(0.5, 12, 90);
        }
        else if (allianceColor == AllianceColor.RED) {
            moveCardinal(0.5, 12, 270);
        }

        sleep(500);

        if (autonType == AutonType.TWOSKYSTONES) {
            moveCardinal(0.5, 12, 0);
        }
        else if (autonType == AutonType.FOUNDATION) {
            moveCardinal(0.5, 12, 180);
        }
    }

    public void testMoveOne() {
        for (int j = 0; j < 1; j++) {
            for (int i = 0; i < 360; i += 90) {
                gotoDegreesRamping(0.5, i);
                sleep(1000);
            }
        }

        while (opModeIsActive()) {
            // makeStraight(0.5);
            makeStraight();
            sleep(5000);
        }
    }

    public void testMoveTwo() {
        for (int i = 0; i < 360; i += 90) {
            moveCardinal(1, inchesToCm(30), i);
            sleep(1000);
        }

        for (int i = 0; i < 360; i += 90) {
            moveCardinal(0.5, inchesToCm(12), i);
            sleep(1000);
        }

        for (int i = 0; i < 360; i += 90) {
            moveCardinal(0.3, inchesToCm(4), i);
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
        }
        else if (color.equals("RED")) {
            allianceColor = AllianceColor.RED;
        }

        if (type.equals("TWOSKYSTONES")) {
            autonType = AutonType.TWOSKYSTONES;
        }
        else if (type.equals("FOUNDATION")) {
            autonType = AutonType.FOUNDATION;
        }
    }

    private enum AllianceColor {
        BLUE, RED
    }

    private enum AutonType {
        TWOSKYSTONES, FOUNDATION
    }





}
