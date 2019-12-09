package org.firstinspires.ftc.teamcode.Navigation.Game;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Tools.Logger.LoggerTools;
import org.firstinspires.ftc.teamcode.Navigation.Geometry.*;
import org.firstinspires.ftc.teamcode.Tools.Move.MoveTools;


public class Robot {
    private Field currentField;
    private Coord position = new Coord(0, 0);
    private double direction_deg = 90;

    LoggerTools logger;
    public LoggerTools.RobotTime time;
    MoveTools driverFunction;
    MoveTools.Steering steering;

    public Robot(LoggerTools logger, MoveTools moveTools, Field field) {
        this.logger = logger;
        this.time = logger.getRobotTimeClass();
        this.driverFunction = moveTools;
        this.steering = moveTools.getSteeringClass();
        currentField = field;
    }

    private void addDegrees(double deg_add) {
        this.direction_deg = (direction_deg + deg_add) % 360;
    }

    public double getDirectionRad() {
        return Math.toRadians(direction_deg);
    }

    public void moveTo(Coord end_point, double speed) {
        logger.add("Status", "Starting movement");

        double cord_dist = position.distance(end_point);
        double angle = calcAngle(end_point);
        double actual_dist = currentField.convertToMeters(cord_dist);

        logger.add("turning relative to robot: ", String.valueOf(angle) + "degrees");
        logger.add("Actual dist:", String.valueOf(actual_dist));
        logger.update(false);

        steering.moveDistance(actual_dist, angle + direction_deg, speed);

        steering.resetAllEncoders();
        position = end_point;
        addDegrees(angle);

    }

    public double calcAngle(Coord end_pt) {
        // If a + g >= (p1, p2) dist
        // angle = 180
        Coord arb_robot_pos = Coord.shift(new Coord(position.get_xcor() + Math.cos(getDirectionRad()), position.get_ycor() + Math.sin(getDirectionRad())), position);
        double dist_to_endpt = position.distance(end_pt);
        double dist_to_arbitrary = 1.0;
        double dist_hypotenuse = arb_robot_pos.distance(end_pt);

        if (dist_hypotenuse == dist_to_arbitrary + dist_to_endpt) {
            LineSegment pos_to_end = new LineSegment(position, end_pt);
            if (pos_to_end.onSegment(arb_robot_pos)) {
                return 0;
            } else {
                return 180;
            }
        } else {
            // does law of cosines to calc angle needed to turn when a triangle is formed
            double numerator = Math.pow(dist_hypotenuse, 2) - Math.pow(dist_to_arbitrary, 2) - Math.pow(dist_to_endpt, 2);
            double denominator = -2 * (dist_to_arbitrary) * (dist_to_endpt);
            return Math.toDegrees(Math.acos(numerator / denominator));
        }

    }


}