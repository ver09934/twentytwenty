package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Foundation Auton")
public class BlueFoundationAuton extends SkystoneAuton {

    @Override
    public void runOpMode() {
        setAutonType(AutonType.FOUNDATION);
        setAllianceColor(AllianceColor.BLUE);
        initThings();
        mainAuton();
        endTelemetry();
    }

}
