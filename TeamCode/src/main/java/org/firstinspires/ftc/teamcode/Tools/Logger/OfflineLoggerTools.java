package org.firstinspires.ftc.teamcode.Tools.Logger;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Tools.Logger.LoggerTools;

import java.util.concurrent.TimeUnit;

public class OfflineLoggerTools implements LoggerTools {
    RobotTime time = new RobotTime();

    public RobotTime getRobotTimeClass() {
        return time;
    }

    public OfflineLoggerTools() {
    }

    public void add(String caption, String text) {
        message_queue.add(new String[]{caption, text});
    }

    public void add(String caption, String text, boolean update) {
        message_queue.add(new String[]{caption, text});
        if (update) {
            update(true);
        }
    }

    public void update(boolean clear) {
        for (int i = 0; i < message_queue.size(); i++) {
            String[] msg = message_queue.get(i);
            System.out.println(msg[0] + " " + msg[1]);
        }
        if (clear) {
            message_queue.clear();
        }
    }

    class RobotTime implements LoggerTools.RobotTime {
        volatile long nanoStartTime = nsNow();
        final long SECOND_IN_NANO = (long) (1 * Math.pow(10, -9));

        public long nsNow() {
            return System.nanoTime();
        }

        // Returns the current time on the clock used by the timer in the proper unit
        public long now(TimeUnit unit) {
            return unit.convert(nsNow(), TimeUnit.NANOSECONDS);
        }

        // Resets the internal state of the timer to reflect the current time
        public void reset() {
            nanoStartTime = nsNow();
        }

        // Returns, in resolution-dependent units, the time at which this timer was last reset.
        public double startTime() {
            return nanoStartTime;
        }

        // Returns the duration that has elapsed since the last reset of this timer in nanoseconds
        public double time() {
            return (nsNow() - nanoStartTime);
        }

        public double seconds() {
            return nanoseconds() / (double) 1_000_000_000.0;
        }

        public double milliseconds() {
            return seconds() * 1000;
        }

        public long nanoseconds() {
            return (nsNow() - nanoStartTime);
        }
    }


}
