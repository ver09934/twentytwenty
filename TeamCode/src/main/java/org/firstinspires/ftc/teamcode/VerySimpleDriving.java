package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class VerySimpleDriving {

    DcMotor lfMotor;
    DcMotor rfMotor;
    DcMotor lbMotor;
    DcMotor rbMotor;

    public VerySimpleDriving(HardwareMap hardwareMap) {
        lfMotor = hardwareMap.dcMotor.get("lfMotor");
        rfMotor = hardwareMap.dcMotor.get("rfMotor");
        lbMotor = hardwareMap.dcMotor.get("lbMotor");
        rbMotor = hardwareMap.dcMotor.get("rbMotor");
    }



}