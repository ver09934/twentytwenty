package org.firstinspires.ftc.teamcode.RobotTests;

import org.firstinspires.ftc.teamcode.Tools.Logger.LoggerTools;
import org.firstinspires.ftc.teamcode.Tools.Logger.OfflineLoggerTools;

class TestOffline {
    LoggerTools logger = new OfflineLoggerTools();
    LoggerTools.RobotTime time = logger.getRobotTimeClass();

    public static void main(String[] args) {
        LoggerTools logger = new OfflineLoggerTools();
        LoggerTools.RobotTime time = logger.getRobotTimeClass();

        // Output logger tests
        TestOffline tests = new TestOffline();
        tests.messageNoStackTest();
        tests.messageStackTest();
        tests.elapseTimeTest();

    }

    void elapseTimeTest() {
        // Elapsed time test (seconds, milliseconds, nanoseconds)
        time.reset();
        logger.add(String.valueOf(time.startTime()), "");
        logger.add(String.valueOf(time.time()), "");
        logger.add("Elapsed seconds, milliseconds, nanoseconds:", "");

        for (int i = 0; i < 10; i++) {
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            logger.add("", String.valueOf(time.seconds() + " " + String.valueOf(time.milliseconds() + " " + String.valueOf(time.nanoseconds()))));
            logger.update(true);
        }
    }

    void messageStackTest() {
        // Messages stacking
        logger.add("stack1", "a");
        logger.update(false);
        logger.add("stack2", "b");
        logger.update(false);
        logger.add("stack3", "c");

        logger.update(true);
    }

    void messageNoStackTest() {
        // Messages stacking
        logger.add("stack1", "a", true);
        logger.add("stack2", "b", true);
        logger.add("stack3", "c", true);
    }
}