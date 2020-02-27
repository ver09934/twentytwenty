package org.firstinspires.ftc.teamcode.drive.HelperClasses;

import org.firstinspires.ftc.teamcode.drive.HelperClasses.Geometry.Coord;
import org.firstinspires.ftc.teamcode.drive.HelperClasses.Geometry.Rectangle;

public class Block extends Rectangle {
    public static Coord FIRST_START_POINT = new Coord(-24, -20);
    public static double LENGTH = 8;
    public static double WIDTH = 4;
    public boolean isSkystone = false;


    public Block(Coord[] points) {
        super(points);
    }

    public static Block fromStartPoint(int index) {
        double x = ((Field.TILE_LENGTH * 2.0) / 6) * (index) + FIRST_START_POINT.get_x() + LENGTH / 2;
        double y = FIRST_START_POINT.get_y();

        Coord topLeft = new Coord(x, y);
        Coord topRight = new Coord(topLeft.get_x() + WIDTH, topLeft.get_y());
        Coord bottomLeft = new Coord(topLeft.get_x(), topLeft.get_y() + LENGTH);
        Coord bottomRight = new Coord(topRight.get_x(), bottomLeft.get_y());

        return new Block(new Coord[]{topLeft, topRight, bottomLeft, bottomRight});
    }

    public double getXCoordOfSide() {
        return getCorners()[0].get_x();
    }

    public Coord getMidptOfBack() {
        Coord[] corners = getCorners();
        Coord topLeft = corners[0];
        Coord topRight = corners[1];
        return topLeft.midpoint(topRight);
    }

    public void setSkystone() {
        this.isSkystone = true;
    }

}
