package org.firstinspires.ftc.teamcode.drive.HelperClasses;

import org.firstinspires.ftc.teamcode.drive.HelperClasses.Geometry.Coord;
import org.firstinspires.ftc.teamcode.drive.HelperClasses.Geometry.LineSegment;

import java.util.ArrayList;

public class Field {
    int coordinatesQuadrant;
    public static double TILE_LENGTH = 24;
    public ArrayList<Block> blocks = new ArrayList<>();
    Robot rb = new Robot();

    public Field(int coordinatesQuadrant) {
        this.coordinatesQuadrant = coordinatesQuadrant;
    }

    public void setBlocks() {
        for (int index = 0; index < 6; index++) {
            blocks.add(Block.fromStartPoint(index));
        }
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
    public void shiftBlocks(int[] blockIndexes, int numUnits) {
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
