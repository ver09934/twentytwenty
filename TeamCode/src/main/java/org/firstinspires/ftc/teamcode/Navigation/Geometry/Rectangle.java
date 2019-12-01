package org.firstinspires.ftc.teamcode.Navigation.Geometry;

public class Rectangle {
    private Coord a;
    private Coord b;
    private Coord c;
    private Coord d;

    public Rectangle(Coord[] points) {
        this.a = points[0];
        this.b = points[1];
        this.c = points[2];
        this.d = points[3];
    }


    // In clockwise order
    public Coord[] getCorners() {
        return new Coord[]{a, b, c, d};
    }

    public LineSegment[] getEdges() {
        LineSegment[] sides = new LineSegment[4];
        Coord[] corners = getCorners();
        for (int i = 0; i < 4; i++) {
            sides[i] = new LineSegment(corners[i], corners[i+1]);
        }
        sides[3] = new LineSegment(corners[0], corners[3]);
        return sides;
    }

    public int numLineSegsIntersect(LineSegment line) {
        int intersect_count = 0;
        for (LineSegment side : getEdges()) {
            if (LineSegment.segmentsIntersect(side, line)) {
                intersect_count++;
            }
        }
        return intersect_count;
    }

    public int numLinesIntersect(Line line) {
        int intersect_count = 0;
        for (LineSegment side : getEdges()) {
            if (LineSegment.lineIntersectsSeg(side, line)) {
                intersect_count++;
            }
        }
        return intersect_count;
    }

    public Rectangle scaleRectangleInPlace(Rectangle A, double factor) {
        Coord[] points = getCorners();
        for (int i = 0; i < 4; i++) {
            Coord point = points[i];
            double x = point.get_xcor() * factor;
            double y = point.get_ycor() * factor;
            points[i] = new Coord(x - ((x-point.get_xcor())/2), y - ((y-point.get_ycor())/2));
        }
        return new Rectangle(points);

    }
    public boolean coordInRectangle(Coord pt) {
        boolean in_x_bounds = pt.get_xcor() < b.get_xcor() && pt.get_xcor() > a.get_xcor();
        boolean in_y_bounds = pt.get_ycor() < a.get_ycor() && pt.get_ycor() > c.get_ycor();
        return in_x_bounds && in_y_bounds;
    }

    public String toString() {
        String output = "";
        for (Coord coord : getCorners()) {
            output += coord.toString() + "\n";
        }
        return output;
    }
}
