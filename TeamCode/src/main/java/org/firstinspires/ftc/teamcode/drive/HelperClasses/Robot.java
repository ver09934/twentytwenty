package org.firstinspires.ftc.teamcode.drive.HelperClasses;

import org.firstinspires.ftc.teamcode.drive.HelperClasses.Geometry.Coord;
import org.firstinspires.ftc.teamcode.drive.HelperClasses.Geometry.LineSegment;
import org.firstinspires.ftc.teamcode.drive.HelperClasses.Geometry.Rectangle;

public class Robot extends Rectangle {
    public Coord position;

    RobotSide deposit;
    RobotSide cam;
    RobotSide grab;
    RobotSide none;
    public RobotSide[] sides;
    public static final Coord START_OFFSET = new Coord(-15, -64);
    public static double LEN = 18.0;

    public static enum SIDE {
        CAM, GRAB, DEPOSIT, EMPTY
    }

    public Robot() {
        super(Rectangle.genCoords(START_OFFSET, LEN));
        this.setSides();
    }

    public void setSides() {
        LineSegment[] edges = this.getEdges();
        RobotSide deposit = new RobotSide(edges[0], SIDE.DEPOSIT, 90);
        RobotSide cam = new RobotSide(edges[1], SIDE.CAM, 0);
        RobotSide grab = new RobotSide(edges[2], SIDE.GRAB, 270);
        RobotSide none = new RobotSide(edges[3], SIDE.EMPTY, 180);
        sides = new RobotSide[]{deposit, cam, grab, none};
    }

}
