package org.firstinspires.ftc.teamcode;


import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.*;

public class Sensors {
    // ----- COLOR SENSOR STUFF -----

    public ColorSensor leftColorColorSensor;
    public DistanceSensor leftColorDistanceSensor;

    public ColorSensor rightColorColorSensor;
    public DistanceSensor rightColorDistanceSensor;

    HardwareMap hardwareMap;
    LinearOpMode op;
    Telemetry telemetry;
    SkystoneAuton.AllianceColor color;

    Sensors(LinearOpMode op, SkystoneAuton.AllianceColor color) {
        hardwareMap = op.hardwareMap;
        this.op = op;
        this.telemetry = op.telemetry;
        this.color = color;
    }

    public void initColorSensors() {
        leftColorColorSensor = hardwareMap.get(ColorSensor.class, "leftColorSensor");
        leftColorDistanceSensor = hardwareMap.get(DistanceSensor.class, "leftColorSensor");

        // TODO: Add sensor to config
        rightColorColorSensor = hardwareMap.get(ColorSensor.class, "rightColorSensor");
        rightColorDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightColorSensor");
    }

    public float[] getHSV() {

        // This couldn't possibly be bad design
        DistanceSensor distanceSensor = null;
        ColorSensor colorSensor = null;

        if (color == color.BLUE) {
            distanceSensor = leftColorDistanceSensor;
            colorSensor = leftColorColorSensor;
        } else if (color == color.RED) {
            distanceSensor = rightColorDistanceSensor;
            colorSensor = rightColorColorSensor;
        }

        double dist = distanceSensor.getDistance(DistanceUnit.CM);

        double a = colorSensor.alpha();
        double r = colorSensor.red();
        double g = colorSensor.green();
        double b = colorSensor.blue();

        float hsvValues[] = {0F, 0F, 0F};
        final double SCALE_FACTOR = 255;

        Color.RGBToHSV(
                (int) (r * SCALE_FACTOR),
                (int) (g * SCALE_FACTOR),
                (int) (b * SCALE_FACTOR),
                hsvValues
        );

        return hsvValues;
    }

// ----- DISTANCE SENSOR STUFF -----

    public DistanceSensor frontDistanceSensor;
    public DistanceSensor leftDistanceSensor;
    public DistanceSensor rightDistanceSensor;

    public DistanceUnit distanceUnit = DistanceUnit.CM;

    public void initDistanceSensors() {
        frontDistanceSensor = hardwareMap.get(DistanceSensor.class, "frontDistanceSensor");
        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");
    }

// ----- IMU STUFF -----

    public BNO055IMU imu;

    public void initIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu = op.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Status", "Calibrating IMU");
        telemetry.update();

        while (!op.isStopRequested() && !imu.isGyroCalibrated()) {
            op.sleep(50);
            op.idle();
        }
    }

    public double getIMUAngleConverted() {
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double angle = orientation.firstAngle;
        angle = angle < 0 ? angle + 360 : angle;
        return angle;
    }
}