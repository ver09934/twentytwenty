package dev.kalink.robotmockup.geometry;

import java.util.ArrayList;
import java.util.List;

public class Line {
    protected Coord pt1;
    protected Coord pt2;

    Line(Coord a, Coord b) {
        this.pt1 = a;
        this.pt2 = b;
    }

    //WILL NOT CHECK IF LINE IS VERTICAL
    double getSlope() {
        double diff_x = pt1.get_xcor() - pt2.get_xcor();
        double diff_y = pt1.get_ycor() - pt2.get_ycor();

        return diff_x / diff_y;
    }

    //WILL NOT CHECK IF LINE IS VERTICAL
    double getYIntercept() {
        return pt1.get_ycor() - getSlope() * pt1.get_xcor();
    }

    boolean isParallel(Line other_ln) {
        if (isUndefined(this) && isUndefined(other_ln) && pt1.get_xcor() != other_ln.pt1.get_xcor()) {
            return true;
        } else if (hasNoSlope(this) && hasNoSlope(other_ln)) {
            return true;
        } else {
            return other_ln.getSlope() == this.getSlope() && other_ln.getYIntercept() != this.getYIntercept();
        }
    }

    boolean isOn(Coord a) {
        return ((this.getSlope() * a.get_xcor() + this.getYIntercept()) == a.get_ycor());
    }

    static boolean isUndefined(Line a) {
        return a.pt1.get_xcor() == a.pt2.get_xcor();
    }

    static boolean hasNoSlope(Line a) {
        return a.pt1.get_ycor() == a.pt2.get_ycor();
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
            double y_value = A.getSlope() * undefined.pt1.get_xcor() + A.getYIntercept();
            return new Coord(undefined.pt1.get_xcor(), y_value);
        } else {
            double x = (A.getYIntercept() - B.getYIntercept()) / (B.getSlope() - A.getSlope());
            double y = B.getSlope() * (x) + B.getYIntercept();
            return new Coord(x, y);
        }
    }

    public boolean coordOnLine(Coord A) {
        if (isUndefined(this)) {
            return A.get_xcor() == pt1.get_xcor();
        } else {
            double expected_y = getSlope() * A.get_xcor() + getYIntercept();
            return A.get_ycor() == expected_y;
        }
    }


}
