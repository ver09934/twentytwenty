package org.firstinspires.ftc.teamcode.drive.HelperClasses.Geometry;

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

    public static Coord[] genCoords(Coord start, double len) {
        Coord topLeft = new Coord(start.getX(), start.getY());
        Coord topRight = new Coord(topLeft.getX() + len, topLeft.getY());
        Coord bottomLeft = new Coord(topLeft.getX(), topLeft.getY() + len);
        Coord bottomRight = new Coord(topRight.getX(), bottomLeft.getY());
        Coord[] points = new Coord[] {topLeft, topRight, bottomLeft, bottomRight};
        return points;
    }


    // In clockwise order
    public Coord[] getCorners() {
        return new Coord[]{a, b, c, d};
    }

    public LineSegment[] getEdges() {
        LineSegment[] sides = new LineSegment[4];
        Coord[] corners = getCorners();
        for (int i = 0; i < 3; i++) {
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
            double x = point.getX() * factor;
            double y = point.getY() * factor;
            points[i] = new Coord(x - ((x-point.getX())/2), y - ((y-point.getY())/2));
        }
        return new Rectangle(points);

    }
    public boolean coordInRectangle(Coord pt) {
        boolean in_x_bounds = pt.getX() < b.getX() && pt.getX() > a.getX();
        boolean in_y_bounds = pt.getY() < a.getY() && pt.getY() > c.getY();
        return in_x_bounds && in_y_bounds;
    }

    public String toString() {
        String output = "";
        for (Coord coord : getCorners()) {
            output += coord.toString() + "\n";
        }
        return output;
    }

    public void translate(Coord units) {
        Coord[] shiftedCords = Coord.shiftCoords(units, this.getCorners());
        this.a = shiftedCords[0];
        this.b = shiftedCords[1];
        this.c = shiftedCords[2];
        this.d = shiftedCords[3];
    }
}
