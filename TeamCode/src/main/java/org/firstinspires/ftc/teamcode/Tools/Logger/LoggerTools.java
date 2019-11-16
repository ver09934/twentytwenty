package org.firstinspires.ftc.teamcode.Tools.Logger;

import java.util.ArrayList;

public interface LoggerTools {
    ArrayList<String[]> message_queue = new ArrayList<String[]>();
    double elapsed_time = 0;

    void add(String caption, String text, boolean update);

    void update();


    interface RobotTime {

        double getSeconds();

        void resetTime();
    }


}