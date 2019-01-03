package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    public DcMotor intakeMotor;

    public Intake(HardwareMap hwMap, Mode mode) {
        intakeMotor = hwMap.dcMotor.get("intakeMotor");
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setMotorRunMode(mode);


    }

    public void setMotorRunMode(Mode mode) {
        if (mode == Mode.Auto) {
            intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        } else {
            intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }
    }
}
