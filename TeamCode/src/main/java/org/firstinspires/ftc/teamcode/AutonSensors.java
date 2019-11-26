package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class AutonSensors {

    // NOTE: No need to pass in telemetry, just to any needed from the main auton

    // TODO: Just put distance sensors in an array, getDistance method takes index
    // That couldn't possibly be more confusing...
    // "But look at how many lines I saved!"
    // Do you have any idea how many lines of boilerplate go into an Android app?
    // I think I might be going insane

    private DistanceSensor sideBack;
    private DistanceSensor sideFront;
    private DistanceSensor back;
    private DistanceSensor front;

    private DistanceUnit unit = DistanceUnit.CM;

    AutonSensors(HardwareMap hardwareMap) {
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
