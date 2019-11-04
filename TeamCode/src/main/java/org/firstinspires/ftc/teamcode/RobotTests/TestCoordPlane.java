package org.firstinspires.ftc.teamcode.RobotTests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Driving.DriverFunction;
import org.firstinspires.ftc.teamcode.Navigation.Game.Field;
import org.firstinspires.ftc.teamcode.Navigation.Game.Robot;
import org.firstinspires.ftc.teamcode.Navigation.Geometry.Coord;


@Autonomous(name = "CoordPlaneTest")
public class TestCoordPlane extends LinearOpMode {
    private ElapsedTime runtime;
    private DriverFunction driverFunction;
    private DriverFunction.Steering steering;

    @Override
    public void runOpMode() {

        // --- Init ---
        Coord[] field_points = new Coord[]{new Coord(-50, 50), new Coord(50, 50), new Coord(-50, 50), new Coord(-50, -50)};
        Field field = new Field(field_points, 3.6576);
        Robot robot = new Robot(field);

        // Initial telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.moveTo(new Coord(10, 10));

        // Initialize utilities
        runtime = new ElapsedTime();
        driverFunction = new DriverFunction(hardwareMap, telemetry);
        steering = driverFunction.getSteering();


        // Reset encoders
        driverFunction.resetAllEncoders();
        while (!this.isStarted()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
        // waitForStart();

        // --- Start ---

        // Potentially reset the encoders here

        runtime.reset();

        telemetry.addData("Status", "Started");
        telemetry.update();

        // LANDING

    }
}
