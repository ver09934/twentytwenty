package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Servos {
    // ----- SERVO STUFF -----

    public Servo autonGrabberLeft;
    public Servo autonGrabberRight;
    public Servo blockServo;

    private Servo plateServoLeft;
    private Servo plateServoRight;

    // TODO: THE IMPORTANT VALUES
    public static double autonGrabberLeftPassive = 0;
    public static double autonGrabberLeftActive = 0.5;

    public static double autonGrabberRightPassive = 0.8;
    public static double autonGrabberRightActive = 0.3;

    private static double blockServoClosedPosition = 0;
    private static double blockServoOpenPosition = 0.5;

    private static double plateServoLeftDown = 0.28;
    private static double plateServoLeftUp = 0.5;
    private static double plateServoRightDown = 0.82;
    private static double plateServoRightUp = 0.6;

    HardwareMap hardwareMap;
    LinearOpMode op;
    Servos(LinearOpMode op) {
        hardwareMap = op.hardwareMap;
        this.op = op;
    }

    public void initServos() {
        autonGrabberLeft = hardwareMap.servo.get("autonGrabberLeft");
        autonGrabberRight = hardwareMap.servo.get("autonGrabberRight");
        retractBothSkystoneGrabbers();

        plateServoLeft = hardwareMap.servo.get("plateServo1");
        plateServoRight = hardwareMap.servo.get("plateServo2");
        plateServosDown();

        blockServo = hardwareMap.servo.get("testServo");
        blockServoJamOpen();
    }

    private void blockServoJamOpen() {
        blockServo.setPosition(blockServoOpenPosition);
    }

    private void blockServoClosed() {
        blockServo.setPosition(blockServoClosedPosition);
    }

    private void plateServosUp() {
        plateServoLeft.setPosition(plateServoLeftUp);
        plateServoRight.setPosition(plateServoRightUp);
    }

    private void plateServosDown() {
        plateServoLeft.setPosition(plateServoLeftDown);
        plateServoRight.setPosition(plateServoRightDown);
    }

    public void retractBothSkystoneGrabbers() {
        autonGrabberLeft.setPosition(autonGrabberLeftPassive);
        autonGrabberRight.setPosition(autonGrabberRightPassive);
    }

    public void deploySkystoneGrabber() {
        if (allianceColor == AllianceColor.BLUE) {
            autonGrabberLeft.setPosition(autonGrabberLeftActive);
        } else if (allianceColor == AllianceColor.RED) {
            autonGrabberRight.setPosition(autonGrabberRightActive);
        }
    }

    public void retractSkystoneGrabber() {
        if (allianceColor == AllianceColor.BLUE) {
            autonGrabberLeft.setPosition(autonGrabberLeftPassive);
        } else if (allianceColor == AllianceColor.RED) {
            autonGrabberRight.setPosition(autonGrabberRightPassive);
        }
    }


}