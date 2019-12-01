package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Color Sensor Test Auton")
public class TestColorAuton extends LinearOpMode {

    private ElapsedTime runtime;

    ColorSensor colorSensor;
    DistanceSensor distanceSensor;

    @Override
    public void runOpMode() {

        runtime = new ElapsedTime();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        runtime = new ElapsedTime();

        colorSensor = hardwareMap.get(ColorSensor.class, "leftColorSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "leftColorSensor");

        while (!this.isStarted()) {
            telemetry.addData("Status", "Waiting for start");
            telemetry.addData("Runtime", runtime.toString());
            telemetry.update();
        }

        while (opModeIsActive()) {

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

            telemetry.addData("Distance [cm]", dist);
            telemetry.addLine("-------------------");
            telemetry.addData("Alpha", a);
            telemetry.addData("Red", r);
            telemetry.addData("Green", g);
            telemetry.addData("Blue ", b);
            telemetry.addLine("-------------------");
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.addData("Sat", hsvValues[1]);
            telemetry.addData("Val", hsvValues[2]);
            telemetry.update();
        }
    }
}
