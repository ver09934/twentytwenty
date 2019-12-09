package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Navigation.Game.Field;
import org.firstinspires.ftc.teamcode.Navigation.Geometry.Coord;
import org.firstinspires.ftc.teamcode.Tools.Logger.LoggerTools;
import org.firstinspires.ftc.teamcode.Tools.Move.MoveTools;
import org.firstinspires.ftc.teamcode.Tools.Move.OnlineMove2020;

@Autonomous(name = "Skystone Auton")
public class SkystoneAuton extends LinearOpMode {

    Coord[] field_points = new Coord[]{new Coord(-50, 50), new Coord(50, 50), new Coord(-50, 50), new Coord(-50, -50)};
    Field field = new Field(field_points, 3.6576);
    OnlineMove2020 move;
    LoggerTools logger;
    LoggerTools.RobotTime runtime;

    @Override
    public void runOpMode() {
        // ----- INIT SECTION -----
        move = new OnlineMove2020(hardwareMap, telemetry, new ElapsedTime());
        logger = move.getLogger();
        runtime = logger.getRobotTimeClass();
        MoveTools.Steering steering = move.getSteeringClass();

        // Waits for the IMU to init
        move.inits.initAll();
        while (!isStopRequested() && !move.imu_sensor.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        while (!this.isStarted()) {
            telemetry.addData("Status", "Waiting for start");
            telemetry.addData("Runtime", runtime.toString());
            telemetry.addData("IMU calibration status", move.imu_sensor.getCalibrationStatus().toString());
            telemetry.update();
        }

        // ----- RUN SECTION -----
        move.movements.testMoveThree();

        while (opModeIsActive()) {
            idle();
        }
    }

    /*
    // TODO: This method
    public void makeStraight() {
    }

    // TODO: This method
    public void gotoDegreesRamping(double power, double degrees) {


        TODO:
        make something nice where we use the sign of the time derivative of the angle
        to let us use angle wrapping to have an absolute angle, because this would make me very happy

        If there is not time for this (which of course there won't be), we can just reuse the logic from
        the hastily created method known as "turnDegrees"

        Must be a bit careful, such as if we are at 359, time derivative is negative, and then we jump to zero
        Therefore must do some tracking over time --> will be good fun


        double currentAngle = getIMUAngleConverted();
*/
}


