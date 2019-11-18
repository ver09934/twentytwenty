package org.firstinspires.ftc.teamcode.Navigation.Game;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Tools.Logger.LoggerTools;
import org.firstinspires.ftc.teamcode.Navigation.Geometry.*;
import org.firstinspires.ftc.teamcode.Tools.Move.MoveTools;


public class Robot {
    private Field currentField;
    private Coord position = new Coord(0, 0);
    private double direction_deg = 0;

    LoggerTools logger;
    public LoggerTools.RobotTime time;
    MoveTools driverFunction;
    MoveTools.Steering steering;


    public Robot(LoggerTools logger, MoveTools moveTools) {
        this.logger = logger;
        this.time = logger.getRobotTimeClass();
        this.driverFunction = moveTools;
        this.steering = moveTools.getSteeringClass();
    }

    private void addDegrees(double deg_add) {
        this.direction_deg = (direction_deg + deg_add) % 360;
    }

    public void moveTo(Coord end_point, double speed) {
        logger.add("Status", "Starting movement", true);


        // Checks the point is possible
        double distance = position.distance(end_point);
        // Gets the angle for the robot to move
        double angle = Math.asin(position.yDist(end_point) / distance);
        addDegrees(angle);


        try {
            //Tells the robot to move at a speed given with the calculated angle, robot goes for a max of 100 seconds, takes 2 seconds to speed up wheels to proper speed
            steering.moveDistance(currentField.convertToMeters(distance), angle, speed);
            position = end_point;
        } catch (Exception InterruptedException) {
            logger.add("Status:", "Failed", true);
        }
        // TODO fix coord in rectangle check
        if (currentField.coordInRectangle(end_point)) {
        }
    }

    double getDistFromWall(double test_dist) {
        return test_dist;
    }
    /*// takes a coord distance to go along wall
    void followWall(double dist_from_wall, double) {
        if (getDistFromWall < set_dist) {

        }
    }


    void move(double angle, double velocity) {

    }

    void makeParallel() {
        if (!checkParallel()) {
            rotate(angleFromWall(Field));
        }
    }

    boolean checkParallel() {
        return sensor1 == sensor2;
    }

    void rotate(double angle) {

    }

    // Returns in degrees
    double angleFromWall(Field current) {
        double diff_length = Math.abs(sensor1 - sensor2);
        return Math.atan(diff_length / DIST_BTW_SENSORS);
    }*/
/*
    public void encoderDrive(double speed, double meters, double angle, double timeoutS, double rampup) throws InterruptedException {
        double COUNTS_PER_MOTOR_REV = 1120;    //Set for NevRest 20 drive. For 40's change to 1120. For 60's 1680
        double DRIVE_GEAR_REDUCTION = 1.0;     // This is the ratio between the motor axle and the wheel
        double WHEEL_DIAMETER_METERS = 0.1016;     // For figuring circumference
        double COUNTS_PER_METER = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_METERS * 3.1415);

        double Rspeed = speed;
        double Lspeed = speed;

        //initialise some variables for the subroutine
        // Ensure that the opmode is still active
        // Determine new target position, and pass to motor controller we only do this in case the encoders are not totally zero'd
        int newLeftTarget = (driverFunction.getLfPosition() + driverFunction.getLbPosition()) / 2 + (int) (meters * COUNTS_PER_METER);;
        int newRightTarget = (driverFunction.getRfPosition() + driverFunction.getRbPosition()) / 2 + (int) (meters * COUNTS_PER_METER);

        // reset the timeout time and start motion.
        time.reset();
        boolean lessThanLeftTarget = Math.abs(driverFunction.getLfPosition() + driverFunction.getLbPosition()) / 2 < newLeftTarget;
        boolean lessThanRightTarget = Math.abs(driverFunction.getRfPosition() + driverFunction.getRbPosition()) / 2 < newRightTarget;


        // keep looping while we are still active, and there is time left, and neither set of motors have reached the target
        while ((time.seconds() < timeoutS) && (lessThanLeftTarget && lessThanRightTarget)) {
            double averagePositions = (Math.abs(driverFunction.getLfPosition()) + Math.abs(driverFunction.getLbPosition()) + Math.abs(driverFunction.getRfPosition()) + Math.abs(driverFunction.getRbPosition())) / 4;
            double newLeftSpeed;
            double newRightSpeed;
            //To Avoid spinning the wheels, this will "Slowly" ramp the motors up over the amount of time you set for this SubRun
            double seconds = time.seconds();
            if (seconds < rampup) {
                double ramp = seconds / rampup;
                newLeftSpeed = Lspeed * ramp;
                newRightSpeed = Rspeed * ramp;
            }

            //Keep running until you are about two rotations out
            else if (averagePositions > (COUNTS_PER_MOTOR_REV * 2)) {
                newLeftSpeed = Lspeed;
                newRightSpeed = Rspeed;
            }
            //start slowing down as you get close to the target
            else if (averagePositions > (200) && (Lspeed * .2) > .1 && (Rspeed * .2) > .1) {
                newLeftSpeed = Lspeed * (averagePositions / 1000);
                newRightSpeed = Rspeed * (averagePositions / 1000);
            }
            //minimum speed
            else {
                newLeftSpeed = Lspeed * .2;
                newRightSpeed = Rspeed * .2;

            }

            double speedX = Math.cos(angle - Math.toRadians(45));
            double speedY = Math.sin(angle - Math.toRadians(45));


            //  Pass the seed values to the motors
            // applies the calculated speeds to work with our wheels
            driverFunction.lf.applyPower(speedX * newLeftSpeed);
            driverFunction.lb.applyPower(speedX * newLeftSpeed);
            driverFunction.rf.applyPower(speedX * newRightSpeed);
            driverFunction.rb.applyPower(speedX * newRightSpeed);

            //Recalculates the booleans
            lessThanLeftTarget = Math.abs(driverFunction.getLfPosition() + driverFunction.getLbPosition()) / 2 < newLeftTarget;
            lessThanRightTarget = Math.abs(driverFunction.getRfPosition() + driverFunction.getRbPosition()) / 2 < newRightTarget;
        }

        // Stop all motion;
        //Note: This is outside our while statement, this will only activate once the time, or distance has been met
        steering.stopAllMotors();
        steering.finishSteering();

        // show the driver how close they got to the last target
        telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
        telemetry.addData("Path2", "Running at %7d :%7d", driverFunction.getLfPosition(), driverFunction.getRfPosition());
        telemetry.update();

    }
*/


}