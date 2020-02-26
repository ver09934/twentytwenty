package org.firstinspires.ftc.teamcode.drive.HelperClasses;

import org.firstinspires.ftc.teamcode.drive.HelperClasses.Geometry.Coord;
import org.firstinspires.ftc.teamcode.drive.HelperClasses.Geometry.LineSegment;

public class RobotSide extends LineSegment {
    public Robot.SIDE type;
    public double facingDir = 0;

    public RobotSide(LineSegment seg, Robot.SIDE type, double facingDir) {
        super(seg.getPt1(), seg.getPt2());
        this.type = type;
        this.facingDir = facingDir;

    }

    public Coord midPoint() {
        return this.getPt1().midpoint(this.getPt2());
    }

    public double midptDistance(Coord other) {
        return this.midPoint().distance(other);
    }
}
