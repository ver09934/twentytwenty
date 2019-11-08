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
    private DriverFunction driverFunction;
    private DriverFunction.Steering steering;

    @Override
    public void runOpMode() {

        // --- Init ---
        Coord[] field_points = new Coord[]{new Coord(-50, 50), new Coord(50, 50), new Coord(-50, 50), new Coord(-50, -50)};
        Field field = new Field(field_points, 3.6576);
        driverFunction = new DriverFunction(hardwareMap, telemetry);
        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        Robot robot = new Robot(field, driverFunction, telemetry, elapsedTime);
        steering = driverFunction.getSteering();

        // Initial telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Reset encoders
        driverFunction.resetAllEncoders();
        while (!this.isStarted()) {
            telemetry.addData("Status", "Run Time: ");
            telemetry.update();
        }
        // waitForStart();

        // --- Start ---

        telemetry.addData("Status", "Test move...");
        telemetry.update();
        steering.moveDegrees(0);
        steering.finishSteering();
        sleep(200); // 600
        steering.stopAllMotors();

        telemetry.addData("Status", "Starting drive");
        telemetry.update();
        robot.moveTo(new Coord(50, 50), 0.5);


        // Potentially reset the encoders here

        elapsedTime.reset();

        telemetry.addData("Status", "Started");
        telemetry.update();

        // LANDING

    }
}
