package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Skystone Auton")
public class SkystoneAuton extends LinearOpMode {

    private ElapsedTime runtime;

    @Override
    public void runOpMode() {

        // --- Init Section ---

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        runtime = new ElapsedTime();

        while (!this.isStarted()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
        // waitForStart();

        // --- Run Section ---

        runtime.reset();

        AutonDrivetrain autonDrivetrain = new AutonDrivetrain(hardwareMap, telemetry);

        autonDrivetrain.moveCardinal(0.5, 304.8, 0);
        // autonDrivetrain.moveDistance(0.5, 243.84, 60);

        /*
        for (int angle : new int[]{0, 90, 180, 270}) {
            autonDrivetrain.moveCardinal(0.5, 200, angle);
            sleep(1000);
        }

        sleep(3000);

        for (int angle = 0; angle < 360; angle += 45) {
            autonDrivetrain.moveDistance(0.5, 150, angle);
            sleep(1000);
        }

        while (opModeIsActive()) {
            telemetry.addData("Status", "Run time: " + runtime.toString());
            telemetry.update();
        }
         */

    }

    // --- Other Methods ---

    // TODO: Implement
    public static void moveAlongWall(double walldist, double stopdist) {

        /*
        if (percentWideWallDistanceDifference > tolerance) {
            setSidewaysWallDistance(targetSideWallDistance)
        }
        then, implement move towards wall
         */

        /*
        Ideas:
        if walldist is way off; then
            strafe to get walldist approx
        fi
        PID loop to stopdist
         */
    }
}
