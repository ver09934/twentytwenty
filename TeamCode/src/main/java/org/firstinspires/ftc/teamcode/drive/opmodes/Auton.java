package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.PathBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.HelperClasses.Field;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;

@Config
@Autonomous(group = "drive")
public class Auton extends LinearOpMode {
    public static double CURRENT_ANGLE = 90; // deg
    public static double HALF_ROBOT_L = 9; // half the width/length of the robot
    public static double GRABBER_OFFSET_FROM_CENTER_LEN = 6 + 1.5;
    public static double BLOCK_MOVE_FORWARD_DIST_GRAB = 1.5;

    Field field = new Field(3);
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
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);
        waitForStart();
        // TODO see how to do actions while moving
        if (isStopRequested()) return;
        // THESE ARE ALL IN TERMS OF THE Q4 coordinates in the path designer
        // Starting coord offset = (-15, -64)
        // Starting cv coord = (-36, -48)

        // Start at starting coord offset
        // Go to CV scanning coord
        // Get block index
        // Get block cord
        // Move to block coord center
        // Engage sucker
        // Move forward BLOCK_MOVE_FORWARD_DIST_GRAB
        // If block grabbed turn off sucker
        // Update block positions for the ones that were nudged
        // Move to build zone
        // Spit block
        // Get coordinate for next block
        Path path = new PathBuilder(new Pose2d())


    }
}
