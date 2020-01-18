package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Skystone Auton")
public class RedSkystoneAuton extends SkystoneAuton {

    @Override
    public void runOpMode() {
        setAutonType(AutonType.TWOSKYSTONES);
        setAllianceColor(AllianceColor.RED);
        initThings();
        mainAuton();
        endTelemetry();
    }

}
