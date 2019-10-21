package dev.kalink.game.geometry;

public class Rectangle {
    private Coord a;
    private Coord b;
    private Coord c;
    private Coord d;

    Rectangle(Coord[] points) {
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
        for (Coord point : points) {
            double x = point.get_xcor() * factor;
            double y = point.get_ycor() * factor;
            point = new Coord(x - ((x-point.get_xcor())/2), y - ((y-point.get_ycor())/2));
        }
        return new Rectangle(points);

    }
}
