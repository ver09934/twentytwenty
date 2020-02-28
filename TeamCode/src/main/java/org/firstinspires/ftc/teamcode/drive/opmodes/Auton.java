package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.HelperClasses.Field;
import org.firstinspires.ftc.teamcode.drive.HelperClasses.Geometry.Coord;
import org.firstinspires.ftc.teamcode.drive.HelperClasses.Robot;

@Config
@Autonomous(group = "drive")
public class Auton extends LinearOpMode {
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
    */
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        if (isStopRequested()) return;
        // Go to CV scanning coord

        // Get block index
        for (int skystoneNum = 0; skystoneNum < 2; skystoneNum++) {
            int skystoneIndex = field.skystoneIndexes[skystoneNum];
            rb.mvNextToBlock(skystoneIndex);
            rb.mvToBlock(skystoneIndex);
            rb.gulpBlock();
            rb.moveToBridgePos();
            rb.ejectBlock();
        }

        // Get block cord
        // Move to block coord center
        // Engage sucker
        // Move forward BLOCK_MOVE_FORWARD_DIST_GRAB
        // If block grabbed turn off sucker
        // Update block positions for the ones that were nudged
        // Move to build zone
        // Spit block
        // Get coordinate for next block



    }
}
