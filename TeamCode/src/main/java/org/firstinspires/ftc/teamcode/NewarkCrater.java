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
        proc.driveTrainProcessor.goAngle(2.5,0,.3);
        proc.driveTrainProcessor.goAngle(2.5,90 ,.3);

        // turn to gold mineral
        proc.turntoGold();
        // hit gold mineral
        proc.intakeProcessor.intakeOn();
        proc.driveTrainProcessor.goAngle(27,0,.5);


        proc.driveTrainProcessor.goAngle(13,180,.5);
        proc.driveTrainProcessor.align(0);
        proc.intakeProcessor.intakeOff();
        //strafe to wall
        proc.driveTrainProcessor.goAngleStall(48,90,.6);

        //drive to depot
        proc.driveTrainProcessor.align(-45);
        proc.driveTrainProcessor.goAngle(40,165,1.0);
        proc.dropMarker();
        //turn toward sample
        //proc.driveTrainProcessor.align(-90);
        //identify seconf particle
        //proc.alignFor2ndSample();
        //push off particle
        //proc.driveTrainProcessor.goAngle(30,0,.5);
        //proc.driveTrainProcessor.goAngle(25,180,.5);

        //turn

        proc.driveTrainProcessor.goAngle(3,-90,1);

        proc.driveTrainProcessor.align(-45);
        //ram crater to park
        proc.driveTrainProcessor.goAngle(1,-90,1);

        proc.driveTrainProcessor.goAngle(50,0,1);

        proc.driveTrainProcessor.goAngle(9,90,1);
        proc.driveTrainProcessor.goAngle(50,0,1);




    }


}
