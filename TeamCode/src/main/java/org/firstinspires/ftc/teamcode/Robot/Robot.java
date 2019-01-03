package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    public LinearOpMode currentOpMode = null;

    public DriveTrain driveTrain;
    public Hang hang;
    public Intake intake;
    public Output output;
    public Sensors sensors;

    public Robot(){

    }

    public void initBot(LinearOpMode currentOpMode,HardwareMap ahwmap, Mode mode){
        driveTrain = new DriveTrain(ahwmap, mode);
        hang = new Hang(ahwmap, mode);
        intake = new Intake(ahwmap,mode);
        output = new Output(ahwmap,mode);
        sensors = new Sensors(ahwmap,mode);
        this.currentOpMode = currentOpMode;

    }



    public boolean isRunningAutonomous() {
        return currentOpMode != null;
    }

    public boolean opModeIsActive() {
        if (isRunningAutonomous()) {
            return currentOpMode.opModeIsActive();
        }
        return true;
    }
}
