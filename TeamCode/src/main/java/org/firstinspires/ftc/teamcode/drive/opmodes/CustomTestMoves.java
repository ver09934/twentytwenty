package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.HelperClasses.Field;
import org.firstinspires.ftc.teamcode.drive.HelperClasses.Geometry.Coord;
import org.firstinspires.ftc.teamcode.drive.HelperClasses.Robot;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;

import java.util.ArrayList;

@Config
@Autonomous(group = "drive")
public class CustomTestMoves extends LinearOpMode {
    public static double CURRENT_ANGLE = 90; // deg
    public static double HALF_ROBOT_L = 9; // half the width/length of the robot
    public static double GRABBER_OFFSET_FROM_CENTER_LEN = 6 + 1.5;
    public static double BLOCK_MOVE_FORWARD_DIST_GRAB = 1.5;
    Field field = new Field(hardwareMap);
    Robot rb = field.getRb();
    /*
        FIELD NOTES:
            1 block is 8 x 4 inches
            The offset from one end of the robot to the nearest grabber is 1.5
            The center of the block is at 4 inches
            The block is 2 tiles into the field
            There are 6 blocks
            1 tile is 2 x 2 feet
            The field is 12 x 12 feet
            robot dimensions 18 x 18 x 14 inches

        FUNCTIONS NEEDED:
            generate path to block
            get block position
            get block from cv
            Robot size
        CLASSES
            Quadrant class (converts coord based on quadrant)
    */
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        if (isStopRequested()) return;

        testZigZag();

        //int DISTANCE = 12;
        //trajectory = drive.trajectoryBuilder().forward(DISTANCE).build();       //Move in a straight line 12 inches forward
        //trajectory = drive.trajectoryBuilder().strafeLeft(DISTANCE).build();    //Move in a straight line 12 inches left
        //trajectory = drive.trajectoryBuilder().back(DISTANCE).build();          //Move in a straight line 12 inches at back
        //trajectory = drive.trajectoryBuilder().strafeRight(DISTANCE).build();   //Move in a straight line 12 inches at right
        //drive.followTrajectorySync(trajectory);

        //sleep(2000);
        //drive.turnSync(Math.toRadians(45));         //Rotate 45 degrees
        //drive.turnSync(Math.toRadians(90));         //Rotate 90 degrees
        //drive.turnSync(Math.toRadians(180));        //Rotate 180 degrees
        //drive.turnSync(Math.toRadians(270));        //Rotate 270 degrees

        // Start point is at robot start in Q3
        // Coord start = Robot.START_OFFSET;
        //Follow spline heading 0
        //trajectory = drive.trajectoryBuilder().splineTo(new Pose2d(30, 30, 0)).build();
        //Follow spline heading 90
        //Follow spline heading 180
        //Follow spline heading 270


    }

    public void testZigZag() {
        // Test straight lines
        Coord startPoint = new Coord(-72 + Robot.HALF_L, -72 + Robot.HALF_L);
        Trajectory trajectory;
        rb.updatePosition(startPoint);
        int forwardDist = 3;
        for (int currDist = 0; currDist < 24 - Robot.HALF_L; currDist += forwardDist) {
            rb.drive.trajectoryBuilder().forward(forwardDist);
            rb.updatePosition(rb.position.getX() + forwardDist, rb.position.getY());
            double distance = Field.TILE_LENGTH * 6 - Robot.HALF_L;
            if (currDist % 2 == 0) {
                trajectory = rb.drive.trajectoryBuilder().strafeLeft(distance).build();
                rb.updatePosition(0, distance);
            } else {
                trajectory = rb.drive.trajectoryBuilder().strafeRight(distance).build();
                rb.updatePosition(0, -1*distance);
            }
            rb.drive.followTrajectorySync(trajectory);
        }
    }

    public void testSplineCorners() {
        Coord startPoint = new Coord(-72 + HALF_ROBOT_L, -72 + HALF_ROBOT_L);
        rb.drive.setPoseEstimate(new Pose2d(startPoint.getX(), startPoint.getY(), Math.toRadians(0)));
        double heading = 0;
        int numTrials = 5;

        Trajectory trajectory;
        rb.updatePosition(startPoint);
        Coord corner1 = new Coord(-72 + HALF_ROBOT_L, 72 - HALF_ROBOT_L);
        Coord corner2 = new Coord(-24 - HALF_ROBOT_L, 72 - HALF_ROBOT_L);
        Coord corner3 = new Coord(-24 - HALF_ROBOT_L, -72 + HALF_ROBOT_L);
        Coord corner4 = new Coord(-72 + HALF_ROBOT_L, -72 + HALF_ROBOT_L);
        Coord center = new Coord(-48, 0);

        ArrayList<Coord[]> movements = new ArrayList<>();
        Coord[] moveA = new Coord[] {corner1, corner3, center};
        Coord[] moveB = new Coord[] {corner4, corner2, center};
        movements.add(moveA);
        movements.add(moveB);


        for (int round = 0; round < numTrials; round++) {
            for (Coord[] moves : movements) {
                for (Coord coord : moves) {
                    trajectory = rb.drive.trajectoryBuilder().splineTo(new Pose2d(coord.getX(), coord.getY(), heading)).build();
                    rb.drive.followTrajectorySync(trajectory);
                }
                sleep(3000);
            }

        }
        Pose2d poseEstimate = rb.drive.getPoseEstimate();
        telemetry.addData("Robot position estimate X: ", poseEstimate.getX());
        telemetry.addData("Robot position estimate Y: ", poseEstimate.getY());
        telemetry.addData("Desired X: ", center.getX());
        telemetry.addData("Desired Y: ", center.getY());
        double percErrorX = 100 * (poseEstimate.getX() - center.getX())/center.getY();
        double percErrorY = 100* (poseEstimate.getX() - center.getX())/center.getY();
        telemetry.addData("Accumulated Percent Error X: ", percErrorX);
        telemetry.addData("Accumulated Percent Error Y: ", percErrorY);

    }
}
