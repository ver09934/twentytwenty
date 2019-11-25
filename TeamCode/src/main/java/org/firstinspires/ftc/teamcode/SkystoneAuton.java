package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
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

        while (opModeIsActive()) {
            telemetry.addData("Status", "Run time: " + runtime.toString());
            telemetry.update();
        }

    }

    // --- Other Methods ---

    public static void moveAlongWall(double walldist, double stopdist) {
        // TODO
        /*
        Ideas:
        if walldist is way off; then
            strafe to get walldist approx
        fi
        PID loop to stopdist
         */
    }
}
