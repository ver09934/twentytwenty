package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Foundation Auton")
public class RedFoundationAuton extends SkystoneAuton {

    @Override
    public void runOpMode() {
        setAutonType(AutonType.FOUNDATION);
        setAllianceColor(AllianceColor.RED);
        initThings();
        mainAuton();
        endTelemetry();
    }

}
