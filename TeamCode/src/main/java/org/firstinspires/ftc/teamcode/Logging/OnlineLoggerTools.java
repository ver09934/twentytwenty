package org.firstinspires.ftc.teamcode.Logging;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OnlineLoggerTools implements LoggerTools {
    Telemetry tel;
    public OnlineLoggerTools(Telemetry telemetry) {
        tel = telemetry;
    }

    public void log(String text) {
        tel.addData("Status", text);
        tel.update();
    }
}
