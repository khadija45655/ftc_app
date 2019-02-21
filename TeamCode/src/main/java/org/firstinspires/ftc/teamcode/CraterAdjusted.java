package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotProcessor.RobotProcessor;
import org.firstinspires.ftc.teamcode.Robot.Mode;

/**
 * Created by khadija on 2/16/2019.
 */
@Autonomous(name = "Crater adjusted", group = "tensor")
public class CraterAdjusted extends LinearOpMode {

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


        if (isStopRequested()) {
            return;
        }



        while (opModeIsActive()){
            while (!isStopRequested()){


            }
        }
        proc.bot.output.marker.setPosition(.5);

        sleep(300);

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
        proc.driveTrainProcessor.goAngle(27,0,1);


        proc.driveTrainProcessor.goAngle(12,180,1);
        proc.driveTrainProcessor.align(90);
        proc.intakeProcessor.intakeOff();


        //strafe to wall
        proc.driveTrainProcessor.goAngleStall(proc.distanceStrafe(),0,1);
        proc.driveTrainProcessor.align(-45);

        proc.driveTrainProcessor.goAngleStall(16,90,1);

        //drive to depot
        proc.driveTrainProcessor.align(-45);


        proc.driveTrainProcessor.goAngle(proc.distanceToWall(),180,1);

        proc.dropMarker();


        proc.driveTrainProcessor.align(-45);



        proc.driveTrainProcessor.goAngle(5,90,1);


        proc.driveTrainProcessor.goAngle(45,0,1);
        proc.driveTrainProcessor.align(-45);

        proc.driveTrainProcessor.goAngle(4,90,1);
        proc.driveTrainProcessor.goAngle(30,0,1);






    }


}
