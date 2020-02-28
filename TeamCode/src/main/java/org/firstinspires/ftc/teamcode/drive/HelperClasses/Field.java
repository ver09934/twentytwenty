package org.firstinspires.ftc.teamcode.drive.HelperClasses;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.HelperClasses.Geometry.Coord;
import org.firstinspires.ftc.teamcode.drive.HelperClasses.Geometry.LineSegment;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;

import java.util.ArrayList;

public class Field {
    public static final Coord parkPosition = new Coord(0, 24 + Robot.HALF_L);
    public static final Coord startPosition = new Coord(-24 + Robot.HALF_L, 72 - Robot.HALF_L);
    public static double TILE_LENGTH = 24;

    public ArrayList<Block> blocks = new ArrayList<>();
    public int[] skystoneIndexes = new int[2];
    Robot rb;

    public Field(HardwareMap hardwareMap) {
        rb = new Robot(hardwareMap, this, new Coord(0, 0));
    }

    public void setBlocks() {
        for (int index = 0; index < 6; index++) {
            blocks.add(Block.fromStartPoint(index));
        }
    }

    public void setSkystones(int firstSkystone) {
        firstSkystone = firstSkystone - 1;
        int secondSkystone = firstSkystone + 3;
        this.skystoneIndexes = new int[]{firstSkystone, secondSkystone};

        Block skystone1 = blocks.get(firstSkystone);
        Block skystone2 = blocks.get(secondSkystone);
        skystone1.setSkystone();
        skystone2.setSkystone();
    }

    /*
       public Coord coordinate(Coord coord) {
           for (int rotations = 0; rotations < numRotations; rotations++) {
               coord = this.rotate90(coord);
           }
           return coord;
       }

       public static Coord reflectOverY(Coord coord) {
           return new Coord(-1 * coord.get_x(), coord.get_y());
       }

       void getNumRotationsToDesired(int desiredQuadrant) {
           int[] quadrants = new int[]{1, 2, 3, 4};

           int curr_quad = this.coordinatesQuadrant;
           while (coordinatesQuadrant != desiredQuadrant) {
               numRotations = (numRotations + 1) % 4 ;
           }
       }


       public void setNumRotations(int numRotations) {
           this.numRotations = numRotations;
       }

       public Coord rotate90(Coord coord) {
           return new Coord(-1 * coord.get_y(), coord.get_x());
       }
   */


    public Robot getRb() {
        return rb;
    }

    public void shiftBlocks(int[] blockIndexes, double numUnits) {
        for (int index : blockIndexes) {
            Block tempBlock = blocks.get(index);
            // Gets the first edge midpoint of the block and then compare to the grabber midpoint and get the dist
            Block blockBeforeDesired;
            if (index == 0) {
                blockBeforeDesired = Block.fromStartPoint(-1);
            } else {
                blockBeforeDesired = blocks.get(index - 1);
            }

            LineSegment backEdge = blockBeforeDesired.getEdges()[0];
            Coord edgeMidpt = backEdge.getPt1().midpoint(backEdge.getPt2());
            double distToMidpt = rb.grab.midptDistance(edgeMidpt);
            tempBlock.translate(new Coord(distToMidpt, 0));
        }
    }


}
