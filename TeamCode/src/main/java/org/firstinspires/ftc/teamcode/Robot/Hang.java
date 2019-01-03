package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hang {
    public DcMotor hangMotor;

    public Hang(HardwareMap hwMap, Mode mode) {
        hangMotor = hwMap.dcMotor.get("hangMotor");
        hangMotor.setDirection(DcMotor.Direction.FORWARD);
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setMotorRunMode(mode);


    }

    public void setMotorRunMode(Mode mode) {
        if (mode == Mode.Auto) {
            hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hangMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        } else {
            hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }
    }
}