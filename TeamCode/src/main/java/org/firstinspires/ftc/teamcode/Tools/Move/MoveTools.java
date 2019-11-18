package org.firstinspires.ftc.teamcode.Tools.Move;


import org.firstinspires.ftc.teamcode.Tools.Logger.LoggerTools;

public interface MoveTools {
    Steering getSteeringClass();
    LoggerTools getLogger();

    DrivingMotor[] getAllMotors();
    DrivingMotor getMotor(String motor_name);
    DrivingMotor getMotor(int motor_index);
    public void resetAllEncoders();


    interface DrivingMotor {
        void applyPower(double power);

        void resetEncoder();

        int position();

        // Nested class providing a weighted value
        class WeightedValue {

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

    interface Steering {

        void setSpeedRatio(double speedRatio);

        double getSpeedRatio();

        void addToAllPowers(double power);

        void setAllPowers(double power);

        void stopAllMotors();

        void moveSeconds(double seconds, double angle, double power);

        void moveDistance(double distance, double angle, double power);

        void moveDistance(double distance, double angle, double power, double rampup, double timeoutSeconds);

        void finishSteering();
    }

}