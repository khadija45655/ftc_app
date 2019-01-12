package org.firstinspires.ftc.teamcode.RobotProcessor;

import org.firstinspires.ftc.teamcode.Robot.Intake;

public class IntakeProcessor {
    public Intake intake;

    public IntakeProcessor(Intake intake) {
        this.intake = intake;
    }

    public void intakeOn() {

        intake.intakeMotor.setPower(-1);
    }

    public void intakeOff() {

        intake.intakeMotor.setPower(0);
    }

    public void runIntake(double time) {
        intakeOn();
        //sleep(time)
        intakeOff();
    }

}
