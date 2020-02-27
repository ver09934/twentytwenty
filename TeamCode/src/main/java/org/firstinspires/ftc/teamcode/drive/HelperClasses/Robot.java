package org.firstinspires.ftc.teamcode.drive.HelperClasses;

import org.firstinspires.ftc.teamcode.drive.HelperClasses.Geometry.Coord;
import org.firstinspires.ftc.teamcode.drive.HelperClasses.Geometry.LineSegment;
import org.firstinspires.ftc.teamcode.drive.HelperClasses.Geometry.Rectangle;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;

public class Robot extends Rectangle {
    public Coord desiredPosition;
    public double rotation;
    public SampleMecanumDriveBase drive;
    public static double HALF_L = 9; // half the width/length of the robot


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

    public Robot(SampleMecanumDriveBase drive) {
        super(Rectangle.genCoords(START_OFFSET, LEN));
        this.setSides();
        this.drive = drive;
    }

    public void setSides() {
        LineSegment[] edges = this.getEdges();
        RobotSide deposit = new RobotSide(edges[0], SIDE.DEPOSIT, 90);
        RobotSide cam = new RobotSide(edges[1], SIDE.CAM, 0);
        RobotSide grab = new RobotSide(edges[2], SIDE.GRAB, 270);
        RobotSide none = new RobotSide(edges[3], SIDE.EMPTY, 180);
        sides = new RobotSide[]{deposit, cam, grab, none};
    }

    public RobotSide getRobotSide(Robot.SIDE side) {
        for (RobotSide s : sides) {
            if (s.type == side) {
                return s;
            }
        }
        return null;
    }

    public void updatePosition(Coord newCoord) {
        this.desiredPosition = newCoord;
    }

    public void updatePosition(double xToAdd, double yToAdd) {
        this.desiredPosition.set_x(this.desiredPosition.get_x() + xToAdd);
        this.desiredPosition.set_y(this.desiredPosition.get_y() + yToAdd);
    }

    public void updateRotation(double rotationToAdd) {
        this.rotation += rotationToAdd;
    }


}
