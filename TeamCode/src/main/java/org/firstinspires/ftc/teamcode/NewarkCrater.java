package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Mode;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.RobotProcessor.RobotProcessor;

@Autonomous(name = "Crater", group = "tensor")
public class NewarkCrater extends LinearOpMode {

    /**
     * Override this method and place your code here.
     * <p>
     * Please do not swallow the InterruptedException, as it is used in cases
     * where the op mode needs to be terminated early.
     *
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {
        RobotProcessor proc = new RobotProcessor(this,hardwareMap,Mode.Auto,telemetry);

        proc.displayINIT();

        proc.identifyLocationV2();
        telemetry.addData("location", proc.locationMineral);

        // decend off lander
        proc.descend();
        proc.driveTrainProcessor.goAngle(2.5,0,1);
        proc.driveTrainProcessor.goAngle(2.5,90 ,1);

        // turn to gold mineral
        proc.turntoGold();
        // hit gold mineral
        proc.intakeProcessor.intakeOn();
        proc.driveTrainProcessor.goAngle(23,0,1);


        proc.driveTrainProcessor.goAngle(12,180,1);
        proc.driveTrainProcessor.align(0);
        proc.intakeProcessor.intakeOff();
        proc.driveTrainProcessor.align(-4);

        //strafe to wall
        proc.driveTrainProcessor.goAngleStall(50,90,1);

        //drive to depot
        proc.driveTrainProcessor.align(45);
        proc.driveTrainProcessor.goAngle(9,0,1);
        proc.driveTrainProcessor.align(45);

        proc.driveTrainProcessor.goAngle(proc.distanceToWall(),90,1);
        proc.driveTrainProcessor.align(0);
        proc.dropMarker();
        proc.driveTrainProcessor.goAngle(8, -90, 1);
        proc.driveTrainProcessor.goAngle(8,0,1);
        proc.driveTrainProcessor.align(-45);



        proc.driveTrainProcessor.goAngle(7,90,1);


        proc.driveTrainProcessor.goAngle(45,0,1);

        proc.driveTrainProcessor.goAngle(9,90,1);
        proc.driveTrainProcessor.goAngle(25,0,1);





    }


}
