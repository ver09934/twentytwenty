package org.firstinspires.ftc.teamcode.Tools.Logger;

import org.firstinspires.ftc.teamcode.Tools.Logger.LoggerTools;

public class OfflineLoggerTools implements LoggerTools {


    public OfflineLoggerTools() {
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
            System.out.println(msg[0] + " " + msg[1]);
        }
        message_queue.clear();
    }


}
