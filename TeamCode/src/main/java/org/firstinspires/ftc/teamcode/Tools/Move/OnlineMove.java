package org.firstinspires.ftc.teamcode.Tools.Move;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Tools.Logger.LoggerTools;
import org.firstinspires.ftc.teamcode.Tools.Logger.OnlineLogger;

import static java.lang.Thread.sleep;

public class OnlineMove implements MoveTools {
    private DrivingMotor lf; // stands for left front
    private DrivingMotor lb; // stands for left back
    private DrivingMotor rf; // stands for right front
    private DrivingMotor rb; // stands for right back

    private Steering steering = new Steering();
    public Steering getSteeringClass() {
        return steering;
    }

    private Telemetry telemetry;
    public ElapsedTime time = new ElapsedTime();
    public LoggerTools logger;
    public LoggerTools.RobotTime rb_time;
    public LoggerTools getLogger() {
        return logger;
    }

    private static final double MAX_SPEED_RATIO = 1;
    private static final double NORMAL_SPEED_RATIO = 0.5;
    private static final double MIN_SPEED_RATIO = 0.3;
    private static final double DEFAULT_SMOOTHNESS = 2;
    private static final DcMotor.RunMode DEFAULT_RUNMODE = DcMotor.RunMode.RUN_USING_ENCODER;

    public OnlineMove(HardwareMap hardwareMap, Telemetry telemetry, ElapsedTime time) {
        this(hardwareMap, telemetry, DEFAULT_SMOOTHNESS);
        this.telemetry = telemetry;
        logger = new OnlineLogger(telemetry,time);
        rb_time = logger.getRobotTimeClass();
    }
    public OnlineMove(HardwareMap hardwareMap, Telemetry telemetry,  double smoothness) {
        this.lf = new DrivingMotor(hardwareMap.dcMotor.get("lfMotor"), smoothness, DEFAULT_RUNMODE);
        this.lb = new DrivingMotor(hardwareMap.dcMotor.get("lbMotor"), smoothness, DEFAULT_RUNMODE);
        this.rf = new DrivingMotor(hardwareMap.dcMotor.get("rfMotor"), smoothness, DEFAULT_RUNMODE);
        this.rb = new DrivingMotor(hardwareMap.dcMotor.get("rbMotor"), smoothness, DEFAULT_RUNMODE);

    }

    public DrivingMotor[] getAllMotors() {
        return new DrivingMotor[] {lf, rf, rb, lb};

    }
    public DrivingMotor getMotor(String motor_name) {
        switch (motor_name) {
            case "lf":
                return lf;
            case "lb":
                return lb;
            case "rf":
                return rf;
            case "rb":
                return rb;
            default:
                return null;
        }
    }
    public DrivingMotor getMotor(int motor_index) {
        return getAllMotors()[motor_index];
    }

    public void resetAllEncoders() {
        lf.resetEncoder();
        lb.resetEncoder();
        rf.resetEncoder();
        rb.resetEncoder();
    }

    /**
     * This class wraps retractRelicSlide regular motor and adds utilities to it.
     * The main feature is that their powers are smoothed so when you "apply" power to them,
     * the power is gradually shifted.
     */
    public static class DrivingMotor implements MoveTools.DrivingMotor {
        public DcMotor motor;
        private WeightedValue acceleration;
        private DcMotor.RunMode runMode;

        public DrivingMotor(DcMotor motor, double smoothness, DcMotor.RunMode runMode) {
            this.motor = motor;
            this.runMode = runMode;
            motor.setMode(runMode);
            acceleration = new WeightedValue(smoothness);
        }

        public void applyPower(double power) {
            this.motor.setPower(acceleration.applyValue(power));
        }
        public void resetEncoder() {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(runMode);
        }
        public int position() {
            return motor.getCurrentPosition();
        }
    }

    // An inner class that manages the repeated recalculation of motor powers.
    public class Steering implements MoveTools.Steering {
        double powerLF = 0;
        double powerLB = 0;
        double powerRF = 0;
        double powerRB = 0;
        double speedRatio = NORMAL_SPEED_RATIO;

        public Steering() {
        }
        public void setSpeedRatio(double speedRatio) {
            this.speedRatio = speedRatio;
        }
        public double getSpeedRatio() {
            return speedRatio;
        }

        public void addToAllPowers(double power) {
            powerLF += power;
            powerLB += power;
            powerRF += power;
            powerRB += power;
        }
        public void setAllPowers(double power) {
            powerLF = power;
            powerLB = power;
            powerRF = power;
            powerRB = power;
        }
        public void stopAllMotors() {
            lf.applyPower(0);
            lb.applyPower(0);
            rf.applyPower(0);
            rb.applyPower(0);
        }

        /**
         * Strafe the robot in any direction at certain power
         *
         * @param angle Angle, specified in radians where 0 is right.
         * @param power The power of the strafe.
         */
        public void moveSeconds(double seconds, double angle, double power) {

            double speedX = Math.cos(angle - Math.toRadians(45));
            double speedY = Math.sin(angle - Math.toRadians(45));

            powerLF += speedX * power;
            powerRB -= speedX * power;
            powerLB += speedY * power;
            powerRF -= speedY * power;

            steering.finishSteering();
            rb_time.sleep(200); // 600
            steering.stopAllMotors();

        }
        public void moveDistance(double distance, double angle, double power) {
            moveDistance(distance, angle, power, 2, 100);
        }
        public void moveDistance(double distance, double angle, double power, double timeoutSeconds) {
            moveDistance(distance, angle, power, 2, timeoutSeconds);
        }
        public void moveDistance(double distance, double angle, double power, double rampup, double timeoutSeconds) {
            double COUNTS_PER_MOTOR_REV = 1120;    //Set for NevRest 20 drive. For 40's change to 1120. For 60's 1680
            double DRIVE_GEAR_REDUCTION = 1.0;     // This is the ratio between the motor axle and the wheel
            double WHEEL_DIAMETER_METERS = 0.1016;     // For figuring circumference
            double COUNTS_PER_METER = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_METERS * Math.PI);

            angle = Math.toRadians(angle);
            double Rspeed = power;
            double Lspeed = power;

            //initialise some variables for the subroutine
            // Determine new target position, and pass to motor controller we only do this in case the encoders are not totally zero'd
            int newLeftTarget = (lf.position() + lb.position()) / 2 + (int) (distance * COUNTS_PER_METER);
            int newRightTarget = (rf.position() + rb.position()) / 2 + (int) (distance * COUNTS_PER_METER);
            logger.add("Left target:", String.valueOf(newLeftTarget), true);
            logger.add("Right Target:", String.valueOf(newRightTarget), true);

            // reset the timeout time and start motion.
            boolean lessThanLeftTarget = Math.abs(lf.position() + lb.position()) / 2 < newLeftTarget;
            boolean lessThanRightTarget = Math.abs(rf.position() + rb.position()) / 2 < newRightTarget;
            time.reset();

            // keep looping while we are still active, and there is time left, and neither set of motors have reached the target
            while ((rb_time.time() < timeoutSeconds) && (lessThanLeftTarget && lessThanRightTarget)) {

                double averagePositions = (double) (Math.abs(lf.position()) + Math.abs(lb.position()) + Math.abs(rf.position()) + Math.abs(rb.position())) / 4;
                double newLeftSpeed;
                double newRightSpeed;
                logger.add("average pos:  ", String.valueOf(averagePositions), true);

                //To Avoid spinning the wheels, this will "Slowly" ramp the motors up over the amount of time you set for this SubRun
                double seconds = rb_time.seconds();

                // ramps up in the beginning based on the number of seconds specified
                if (seconds < rampup) {
                    double ramp = seconds / rampup;
                    newLeftSpeed = Lspeed * ramp;
                    newRightSpeed = Rspeed * ramp;

                //Keep running until you are about two rotations out
                } else if (averagePositions > (COUNTS_PER_MOTOR_REV * 2)) {
                    newLeftSpeed = Lspeed;
                    newRightSpeed = Rspeed;

                //start slowing down as you get close to the target
                } else if (averagePositions > (200) && (Lspeed * .2) > .1 && (Rspeed * .2) > .1) {
                    newLeftSpeed = Lspeed * (averagePositions / 1000);
                    newRightSpeed = Rspeed * (averagePositions / 1000);

                    //minimum speed
                } else {
                    newLeftSpeed = Lspeed * .2;
                    newRightSpeed = Rspeed * .2;

                }

                // calculates the ratio for the wheels to spin to achieve the angle of motion
                double speedX = Math.cos(Math.toRadians(angle) - Math.toRadians(45));

                double speedY = Math.sin(Math.toRadians(angle) - Math.toRadians(45));

                // Applies the calculated value to move and the angle calculation
                powerLF += speedX * newLeftSpeed;
                powerRB -= speedX * newRightSpeed;
                powerLB += speedX * newLeftSpeed;
                powerRF -= speedX * newRightSpeed;
                finishSteering();

                logger.add("Power LF", String.valueOf(powerLF));
                logger.add("Power LB", String.valueOf(powerLB));
                logger.add("Power RB", String.valueOf(powerRB));
                logger.add("Power RF", String.valueOf(powerRF));


                // Recalculates the target value booleans
                lessThanLeftTarget = Math.abs(lf.position() + lb.position()) / 2 < newLeftTarget;
                lessThanRightTarget = Math.abs(rf.position() + rb.position()) / 2 < newRightTarget;
                logger.update(true);
            }

            // Stop all motion;
            //Note: This is outside our while statement, this will only activate once the time, or distance has been met
            logger.add("Status: ", "Stopping steering", true);
        }

        /**
         * Finish up steering and actually spin the motors.
         * This method must be called after any steering operations for anything to happen.
         */
        public void finishSteering() {
            // The maximum base power.
            double maxRawPower = Math.max(Math.max(Math.abs(powerLF), Math.abs(powerLB)), Math.max(Math.abs(powerRF), Math.abs(powerRB)));

            // Actually set the powers for the motors. Dividing by maxRawPower makes the "biggest" power plus or minus 1,
            // and multiplying by speedRatio makes the maximum power equal to speedRatio.
            if (maxRawPower != 0) {
                lf.applyPower(powerLF / maxRawPower * speedRatio);
                lb.applyPower(powerLB / maxRawPower * speedRatio);
                rf.applyPower(powerRF / maxRawPower * speedRatio);
                rb.applyPower(powerRB / maxRawPower * speedRatio);
            } else {
                stopAllMotors();
            }

            setAllPowers(0);
        }


    }
}


