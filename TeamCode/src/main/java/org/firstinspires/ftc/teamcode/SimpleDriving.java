package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SimpleDriving {

    // Motor names are the same in hardware config
    public DrivingMotor lfMotor;
    public DrivingMotor rfMotor;
    public DrivingMotor lbMotor;
    public DrivingMotor rbMotor;

    public Telemetry telemetry;

    public SimpleDriving(HardwareMap hardwareMap, Telemetry telemetry) {
        lfMotor = new DrivingMotor(hardwareMap.dcMotor.get("lfMotor"));
        rfMotor = new DrivingMotor(hardwareMap.dcMotor.get("rfMotor"));
        lbMotor = new DrivingMotor(hardwareMap.dcMotor.get("lbMotor"));
        rbMotor = new DrivingMotor(hardwareMap.dcMotor.get("rbMotor"));
        this.telemetry = telemetry;
    }

    // Encapsulate the motor object, to allow for further functionality in the future
    public static class DrivingMotor {

        public DcMotor motor;
        public DcMotor.RunMode runMode;

        public static final DcMotor.RunMode DEFAULT_RUNMODE = DcMotor.RunMode.RUN_USING_ENCODER;

        public DrivingMotor(DcMotor motor, DcMotor.RunMode runMode) {
            this.runMode = runMode;
            this.motor = motor;
        }

        public DrivingMotor(DcMotor motor) {
            this(motor, DEFAULT_RUNMODE);
        }

    }

}
