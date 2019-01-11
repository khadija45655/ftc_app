package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot implements OpModeIF {
    public LinearOpMode currentOpMode = null;
    public Telemetry telemetry = null;

    public DriveTrain driveTrain;
    public Hang hang;
    public Intake intake;
    public Output output;
    public Sensors sensors;

    public Robot(){

    }

    public Robot(LinearOpMode currentOpMode,HardwareMap ahwmap, Mode mode,Telemetry telemetry){
        this.driveTrain = new DriveTrain(ahwmap, mode);
        this.hang = new Hang(ahwmap, mode);
        this.intake = new Intake(ahwmap,mode);
        this.output = new Output(ahwmap,mode);
        this.sensors = new Sensors(ahwmap,mode);
        this.currentOpMode = currentOpMode;
        this.telemetry = telemetry;
    }

    public void initBot(LinearOpMode currentOpMode,HardwareMap ahwmap, Mode mode,Telemetry telemetry){
        this.driveTrain = new DriveTrain(ahwmap, mode);
        this.hang = new Hang(ahwmap, mode);
        this.intake = new Intake(ahwmap,mode);
        this.output = new Output(ahwmap,mode);
        this.sensors = new Sensors(ahwmap,mode);
        this.currentOpMode = currentOpMode;
        this.telemetry = telemetry;

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
