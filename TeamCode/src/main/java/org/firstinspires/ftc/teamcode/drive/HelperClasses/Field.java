package org.firstinspires.ftc.teamcode.drive.HelperClasses;

import org.firstinspires.ftc.teamcode.drive.HelperClasses.Geometry.Coord;
import org.firstinspires.ftc.teamcode.drive.HelperClasses.Geometry.LineSegment;

import java.util.ArrayList;

public class Field {
    int coordinatesQuadrant;
    public static double TILE_LENGTH = 24;
    public ArrayList<Block> blocks = new ArrayList<>();
    public int[] skystoneIndexes = new int[2];
    Robot rb;

    public Field(int coordinatesQuadrant) {
        this.coordinatesQuadrant = coordinatesQuadrant;
    }

    public void setRb(Robot rb) {
        this.rb = rb;
    }

    public void setBlocks() {
        for (int index = 0; index < 6; index++) {
            blocks.add(Block.fromStartPoint(index));
        }
    }

    public void setSkystones(int firstSkystone) {
        firstSkystone = firstSkystone - 1;
        int secondSkystone = firstSkystone + 3;
        this.skystoneIndexes = new int[] {firstSkystone, secondSkystone};

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
    public void mvRbToBlock(int index) {
        Block bl = blocks.get(index);
        Coord midptBack = bl.getMidptOfBack();

        Coord nextToBlockCoord = new Coord(bl.getXCoordOfSide() - Robot.HALF_L, midptBack.get_y() + Robot.HALF_L);
        Trajectory trajectory = rb.drive.trajectoryBuilder().splineTo(
                new Pose2d(nextToBlockCoord.get_x(), midptBack.get_y(), Math.toRadians(270))).build();
        rb.drive.followTrajectorySync(trajectory);

        double distShift = nextToBlockCoord.xDist(midptBack);
        Coord ptBehindBlock = new Coord(midptBack.get_x(), midptBack.get_y() + Robot.HALF_L);
        trajectory = rb.drive.trajectoryBuilder().strafeLeft(distShift).build();    //Move in a straight line 12 inches left
        rb.drive.followTrajectorySync(trajectory);
        if (index == 0) {
            shiftBlocks(new int[]{0}, distShift);
        } else {
            shiftBlocks(new int[]{index - 1, index}, distShift);
        }
    }

    public Robot getRb() {
        return rb;
    }

    public void shiftBlocks(int[] blockIndexes, double numUnits) {
        for (int index : blockIndexes) {
            Block tempBlock = blocks.get(index);
            // TODO get distance to midpoint

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
