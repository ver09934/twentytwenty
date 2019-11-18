package org.firstinspires.ftc.teamcode.Tools.Logger;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public interface LoggerTools {
    ArrayList<String[]> message_queue = new ArrayList<>();
    RobotTime getRobotTimeClass();

    void add(String caption, String text);
    void add(String caption, String text, boolean update);


    void update(boolean clear);


    interface RobotTime {
        // Returns the current time on the clock used by the timer in the proper unit
        long now(TimeUnit unit);

        // Tells the thread to sleep
        void sleep(long milliseconds);

        // Resets the internal state of the timer to reflect the current time
        void reset();

        // Returns, in resolution-dependent units, the time at which this timer was last reset.
        double startTime();

        // Returns the duration that has elapsed since the last reset of this timer in nanoseconds
        double time();

        double seconds();

        double milliseconds();

        long nanoseconds();

    }


}