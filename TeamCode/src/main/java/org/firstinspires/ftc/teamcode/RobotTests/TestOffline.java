package org.firstinspires.ftc.teamcode.RobotTests;

import org.firstinspires.ftc.teamcode.Tools.Logger.LoggerTools;
import org.firstinspires.ftc.teamcode.Tools.Logger.OfflineLoggerTools;

class TestOffline {
    public static void main(String[] args) {
        LoggerTools logger = new OfflineLoggerTools();
        logger.add("Test", "123", true);
        logger.add("Test", "123", true);
        logger.add("Test", "123", false);

        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        logger.update();

    }
}