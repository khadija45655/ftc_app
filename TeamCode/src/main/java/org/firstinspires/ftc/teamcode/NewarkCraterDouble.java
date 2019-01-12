package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Mode;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.RobotProcessor.RobotProcessor;

@Autonomous(name = "CraterDouble", group = "tensor")
public class NewarkCraterDouble extends LinearOpMode {

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


        waitForStart();

        // identify location of particle
        proc.identifyLocation();
        telemetry.addData("location", proc.locationMineral);

        // decend off lander
        proc.descend();
        proc.driveTrainProcessor.goAngle(2.5,0,.3);
        proc.driveTrainProcessor.goAngle(2.5,90 ,.3);

        // turn to gold mineral
        proc.turntoGold();
        // hit gold mineral
        proc.driveTrainProcessor.goAngle(35,0,.5);
        proc.driveTrainProcessor.align(0);

        proc.driveTrainProcessor.goAngle(10,180,.5);
        proc.driveTrainProcessor.align(0);
        //strafe to wall
        proc.driveTrainProcessor.goAngle(30,-90,.5);

        //drive to depot
        proc.driveTrainProcessor.align(45);
        proc.driveTrainProcessor.goAngle(30,180,.7);
        proc.dropMarker();
        //turn toward sample
        //identify seconf particle
        //push off particle
        //strafe toward crater
        //turn
        //ram crater to park
        proc.alignForSample();

        proc.setUpToDropCrater();
        proc.dropMarker();
        proc.realignForParkCrater();

        //proc.driveTrainProcessor.turn(205);
        //proc.dropMarker();


    }

    //fpublic void kill(){
    // kill();
    // }
}
