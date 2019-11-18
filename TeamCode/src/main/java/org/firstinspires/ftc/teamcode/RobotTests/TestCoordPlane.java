package org.firstinspires.ftc.teamcode.RobotTests;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Driving.DriverFunction;
import org.firstinspires.ftc.teamcode.Navigation.Game.Field;
import org.firstinspires.ftc.teamcode.Navigation.Game.Robot;
import org.firstinspires.ftc.teamcode.Navigation.Geometry.Coord;
import org.firstinspires.ftc.teamcode.Tools.Logger.LoggerTools;
import org.firstinspires.ftc.teamcode.Tools.Logger.OfflineLoggerTools;
import org.firstinspires.ftc.teamcode.Tools.Move.MoveTools;
import org.firstinspires.ftc.teamcode.Tools.Move.OnlineMove;

import java.util.logging.Logger;


@Autonomous(name = "CoordPlaneTest")
public class TestCoordPlane extends LinearOpMode {
    private DriverFunction driverFunction;
    private DriverFunction.Steering steering;

    @Override
    public void runOpMode() {
        // --- Init ---
        Coord[] field_points = new Coord[]{new Coord(-50, 50), new Coord(50, 50), new Coord(-50, 50), new Coord(-50, -50)};
        Field field = new Field(field_points, 3.6576);

        LoggerTools logger = new OfflineLoggerTools();
        LoggerTools.RobotTime time = logger.getRobotTimeClass();
        MoveTools move = new OnlineMove(hardwareMap, telemetry);
        MoveTools.Steering steering = move.getSteeringClass();

        time.reset();
        Robot robot = new Robot(logger, move);

        sleep(500);

        // Initial telemetry
        logger.add("Status", "Initialized", true);

        sleep(500);

        // Reset encoders
        move.resetAllEncoders();
        while (!this.isStarted()) {
            logger.add("Status", "Run Time: ", true);
        }

        // --- Start ---
        time.sleep(500);
        logger.add("Status", "Test move...", true);
        steering.moveSeconds(1, 0, 1);


        logger.add("Status", "Starting drive", true);
        robot.moveTo(new Coord(50, 50), 0.5);


        // Potentially reset the encoders here
        time.reset();
        robot.time.reset();

        logger.add("Status: ", "Ending program...", true);

        // LANDING

    }
}
