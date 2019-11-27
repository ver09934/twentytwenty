package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Skystone Auton")
public class SkystoneAuton extends LinearOpMode {

    private ElapsedTime runtime;

    // --- "Main" Method ---

    @Override
    public void runOpMode() {

        /*
        Some of the control flow options:
        waitForStart();
        while (!this.isStarted()) {}
        while (opModeIsActive()) {}
         */

        // --- Init Section ---

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        runtime = new ElapsedTime();

        while (!this.isStarted()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }

        // --- Run Section ---

        runtime.reset();

        AutonDrivetrain autonDrivetrain = new AutonDrivetrain(hardwareMap);

        for (int angle : new int[]{0, 90, 180, 270}) {
            autonDrivetrain.moveCardinal(1, 50, angle);
        }

        sleep(5000);

        for (int angle = 0; angle < 360; angle+=45) {
            autonDrivetrain.moveDistance(1, 30, angle);
        }

        while (opModeIsActive()) {
            telemetry.addData("Status", "Run time: " + runtime.toString());
            telemetry.update();
        }

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
