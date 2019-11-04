package org.firstinspires.ftc.teamcode.Navigation.Geometry;

public class LineSegment extends Line implements Comparable<LineSegment>{
    private double length;

    LineSegment(Coord a, Coord b) {
        super(a, b);
        this.length = calculateLength();
    }

    private double calculateLength() {
        double x_part = Math.pow(pt1.get_xcor() - pt2.get_xcor(), 2);
        double y_part = Math.pow(pt1.get_ycor() - pt2.get_ycor(), 2);
        return Math.sqrt(x_part + y_part);
    }

    public double getLength() {
        return length;
    }

    // TODO Probably a better way, too lazy to fix
    public boolean onSegment(Coord a) {
        boolean in_x_bounds_1 = a.get_xcor() >= pt1.get_xcor() && a.get_xcor() <= pt2.get_xcor();
        boolean in_x_bounds_2 = a.get_xcor() <= pt1.get_xcor() && a.get_xcor() >= pt2.get_xcor();
        boolean in_y_bounds_1 = a.get_ycor() >= pt1.get_ycor() && a.get_ycor() <= pt2.get_ycor();
        boolean in_y_bounds_2 = a.get_ycor() <= pt1.get_ycor() && a.get_ycor() >= pt2.get_ycor();

        return (in_x_bounds_1 || in_x_bounds_2) && (in_y_bounds_1 || in_y_bounds_2);
    }

    static boolean segmentsIntersect(LineSegment A, LineSegment B) {
        Coord intersect = intersection(A, B);
        return A.onSegment(intersect) && B.onSegment(intersect);
    }

    static boolean lineIntersectsSeg(LineSegment A, Line B) {
        Coord intersect = intersection(A, B);
        return A.onSegment(intersect) && B.coordOnLine(intersect);
    }

    @Override
    public int compareTo(LineSegment lineSegment) {
        if (this.length > lineSegment.length) {
            return 1;
        } else if (this.length < lineSegment.length) {
            return -1;
        } else {
            return 0;
        }
    }
}