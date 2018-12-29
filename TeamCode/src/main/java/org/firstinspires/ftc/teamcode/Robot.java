package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by khadija on 12/28/2018.
 */
public class Robot {
    public RobotDrive drive;
    public Intake intake;

    public Robot(HardwareMap map){
        drive = new RobotDrive(map);
        intake = new Intake(map);
    }
}
