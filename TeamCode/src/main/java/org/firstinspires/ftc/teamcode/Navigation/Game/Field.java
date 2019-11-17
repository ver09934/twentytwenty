package org.firstinspires.ftc.teamcode.Navigation.Game;

import org.firstinspires.ftc.teamcode.Navigation.Geometry.Coord;
import org.firstinspires.ftc.teamcode.Navigation.Geometry.Rectangle;

//Import later
public class Field extends Rectangle {
    //gets units per meters (2019 field is 3.6576 meters in length)
    final double FIELD_LENGTH;
    final double UNIT_CONV_FACTOR;


    public Field(Coord[] points, double field_length) {
        super(points);
        this.FIELD_LENGTH = field_length;
        this.UNIT_CONV_FACTOR = FIELD_LENGTH / 100;
    }

    double convertToMeters(double coord_distance) {
        return coord_distance * UNIT_CONV_FACTOR;
    }

    //Sorry for the long names, please don't shoot me
    double convertToCoord(double length) {
        return length / UNIT_CONV_FACTOR;
    }

}