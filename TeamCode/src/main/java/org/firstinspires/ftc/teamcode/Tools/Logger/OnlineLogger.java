package org.firstinspires.ftc.teamcode.Tools.Logger;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Tools.Logger.LoggerTools;

public class OnlineLogger implements LoggerTools {
    Telemetry telemetry;
    ElapsedTime elapsedTime = new ElapsedTime();

    public OnlineLogger(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void add(String caption, String text, boolean update) {
        message_queue.add(new String[] {caption, text});
        if (update) {
            update();
        }
    }

    public void update() {
        for (int i = 0; i < message_queue.size(); i++) {
            String[] msg = message_queue.get(i);
            telemetry.addData(msg[0], msg[1]);
        }
        telemetry.update();
    }

    class time implements RobotTime {
        public double getSeconds() {
            return elapsedTime.seconds();
        }

        public void resetTime() {
            elapsedTime.reset();
        }
    }

    public void seconds() {

    }
}
