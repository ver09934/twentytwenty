package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Skystone Auton")
public class BlueSkystoneAuton extends SkystoneAuton {

    @Override
    public void runOpMode() {
        setAutonType(AutonType.TWOSKYSTONES);
        setAllianceColor(AllianceColor.BLUE);
        initThings();
        mainAuton();
        endTelemetry();
    }

}
