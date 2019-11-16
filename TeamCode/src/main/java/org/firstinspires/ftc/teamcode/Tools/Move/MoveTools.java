package org.firstinspires.ftc.teamcode.Tools.Move;


import org.firstinspires.ftc.teamcode.Tools.Logger.LoggerTools;

public interface MoveTools {
    Steering getSteering();
    LoggerTools getLogger();

    double MAX_SPEED_RATIO = 1;
    double NORMAL_SPEED_RATIO = 0.5;
    double MIN_SPEED_RATIO = 0.3;
    double DEFAULT_SMOOTHNESS = 2;

    public DrivingMotor getLf();
    public DrivingMotor getLb();
    public DrivingMotor getRf();
    public DrivingMotor getRb();

    public int lfPosition();
    public int lbPosition();
    public int rfPosition();
    public int rbPosition();

    public void getMotor();

    public interface DrivingMotor {
        void applyPower(double power);
        void resetEncoders();
        int getPosition();

        // Nested class providing a weighted value
        public class WeightedValue {

            private double value = 0;
            private double smoothness;

            public WeightedValue(double smoothness) {
                this.smoothness = smoothness;
            }

            public double applyValue(double newValue) {
                if (value < newValue) {
                    value = value + Math.min(newValue - value, smoothness);
                } else {
                    value = value - Math.min(value - newValue, smoothness);
                }
                return value;
            }
        }
    }

    public interface Steering {
        double powerLF = 0;
        double powerLB = 0;
        double powerRF = 0;
        double powerRB = 0;

        double speedRatio = NORMAL_SPEED_RATIO;
        public void setSpeedRatio(double speedRatio);
        public double getSpeedRatio();

        public void addToAllPowers(double power);
        public void stopAllMotors();

        public void moveSeconds(double seconds, double angle);
        public void moveDistance(double distance, double angle);

        public void finishSteering();
    }

}