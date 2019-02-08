package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Mode;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.RobotProcessor.RobotProcessor;

@Autonomous(name = "NewarkDepot", group = "tensor")
public class NewarkDepot extends LinearOpMode {

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
        waitForStart();

        proc.identifyLocation();
        telemetry.addData("location", proc.locationMineral);

        proc.descend();
        proc.driveTrainProcessor.goAngle(2.5,0,.4);
        proc.driveTrainProcessor.goAngle(2.5,90 ,.41);
        proc.turntoGold();
        proc.intakeProcessor.intakeOn();
        proc.driveTrainProcessor.goAngle(35,0,.5);
        proc.alignForSample();
        proc.driveTrainProcessor.goAngle(proc.goToDep(),0,.5);
        proc.driveTrainProcessor.align(0);
        proc.intakeProcessor.intakeOff();
        proc.driveTrainProcessor.align(-90);


        proc.dropMarker();

        proc.driveTrainProcessor.align(-45);
        proc.driveTrainProcessor.goAngleStall(10,90,.3);
        proc.driveTrainProcessor.goAngleStall(4,-90,1);

        proc.driveTrainProcessor.align(-45);



        proc.driveTrainProcessor.goAngleStall(20,180,1);



        proc.driveTrainProcessor.goAngle(45,180,1);

        proc.driveTrainProcessor.goAngle(9,90,1);
        proc.driveTrainProcessor.goAngle(50,180,1);

        /*proc.driveTrainProcessor.goAngle(50,0,1);

        proc.driveTrainProcessor.goAngle(9,90,1);
        proc.driveTrainProcessor.goAngle(50,0,1);





        /*proc.setUpToDropDepot();
        proc.dropMarker();
        sleep(1000);
        proc.realignForParkDepot();*/

        //proc.driveTrainProcessor.turn(205);
        //proc.dropMarker();


    }

    //fpublic void kill(){
       // kill();
   // }


}
