package org.firstinspires.ftc.teamcode.drive.HelperClasses.Geometry;

import java.util.ArrayList;
import java.util.List;

public class Line {
    protected Coord pt1;
    protected Coord pt2;

    Line(Coord a, Coord b) {
        this.pt1 = a;
        this.pt2 = b;
    }

    public Coord getPt1() {
        return pt1;
    }

    public Coord getPt2() {
        return pt2;
    }

    //WILL NOT CHECK IF LINE IS VERTICAL
    double getSlope() {
        double diff_x = pt1.get_x() - pt2.get_x();
        double diff_y = pt1.get_y() - pt2.get_y();
        return diff_x / diff_y;
    }

    //WILL NOT CHECK IF LINE IS VERTICAL
    double getYIntercept() {
        return pt1.get_y() - getSlope() * pt1.get_x();
    }

    boolean isParallel(Line other_ln) {
        if (isUndefined(this) && isUndefined(other_ln) && pt1.get_x() != other_ln.pt1.get_x()) {
            return true;
        } else if (hasNoSlope(this) && hasNoSlope(other_ln)) {
            return true;
        } else {
            return other_ln.getSlope() == this.getSlope() && other_ln.getYIntercept() != this.getYIntercept();
        }
    }

    boolean isOn(Coord a) {
        return ((this.getSlope() * a.get_x() + this.getYIntercept()) == a.get_y());
    }

    static boolean isUndefined(Line a) {
        return a.pt1.get_x() == a.pt2.get_x();
    }

    static boolean hasNoSlope(Line a) {
        return a.pt1.get_y() == a.pt2.get_y();
    }

    static Line[] getUndefinedLines(Line[] lines) {
        List<Line> undefined = new ArrayList<>();
        for (Line line : lines) {
            if (Line.isUndefined(line)) {
                undefined.add(line);
            }
        }

        return undefined.toArray(new Line[0]);

    }

    // Make sure to test that the lines are not parallel before using
    static Coord intersection(Line A, Line B) {
        if ((A.isParallel(B) && (isUndefined(A) || hasNoSlope(A)))) {
            throw new RuntimeException("Lines do not intersect");
        } else if (isUndefined(A) || isUndefined(B)) {
            Line undefined = getUndefinedLines(new Line[]{A, B})[0];
            double y_value = A.getSlope() * undefined.pt1.get_x() + A.getYIntercept();
            return new Coord(undefined.pt1.get_x(), y_value);
        } else {
            double x = (A.getYIntercept() - B.getYIntercept()) / (B.getSlope() - A.getSlope());
            double y = B.getSlope() * (x) + B.getYIntercept();
            return new Coord(x, y);
        }
    }

    public boolean coordOnLine(Coord A) {
        if (isUndefined(this)) {
            return A.get_x() == pt1.get_x();
        } else {
            double expected_y = getSlope() * A.get_x() + getYIntercept();
            return A.get_y() == expected_y;
        }
    }


}
