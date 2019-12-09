package org.firstinspires.ftc.teamcode.Tools.Logger;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Navigation.Game.Robot;
import org.firstinspires.ftc.teamcode.Tools.Logger.LoggerTools;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.concurrent.TimeUnit;

public class OnlineLogger implements LoggerTools {
    Telemetry telemetry;
    ElapsedTime elapsedTime;
    RobotTime rb_time = new RobotTime();


    public RobotTime getRobotTimeClass() {
        return rb_time;
    }

    public OnlineLogger(Telemetry telemetry, ElapsedTime time) {
        this.elapsedTime = time;
        this.telemetry = telemetry;
    }

    public void add(String caption, String text) {
        message_queue.add(new String[]{caption, text});
    }

    public void add(String caption, String text, boolean update) {
        message_queue.add(new String[]{caption, text});
        if (update) {
            update(false);
        }
    }
    public void add(String caption, double num) {
        add(caption, String.valueOf(num));
    }
    public void add(String caption, double num, boolean update) {
        add(caption, String.valueOf(num), update);
    }

    public void add(String caption, ArrayList values) {
        add(caption, Arrays.toString(values.toArray()));
    }

    public void update(boolean clear) {
        for (int i = 0; i < message_queue.size(); i++) {
            String[] msg = message_queue.get(i);
            telemetry.addData(msg[0], msg[1]);
        }
        telemetry.update();
        if (clear) {
            message_queue.clear();
        }
    }

    class RobotTime implements LoggerTools.RobotTime {



        public long now(TimeUnit unit) {
            return elapsedTime.now(unit);
        }

        public void reset() {
            elapsedTime.reset();
        }

        public void sleep(long milliseconds){
            try {
                Thread.sleep(milliseconds);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        public double startTime() {
            return elapsedTime.startTime();
        }

        public double time() {
            return elapsedTime.time();
        }

        public double seconds() {
            return elapsedTime.seconds();
        }

        public double milliseconds() {
            return elapsedTime.milliseconds();
        }

        public long nanoseconds() {
            return elapsedTime.nanoseconds();
        }
    }

    public void seconds() {

    }
}
