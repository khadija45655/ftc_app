package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Mode;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.RobotProcessor.RobotProcessor;

@Autonomous(name = "Depot Adjusted", group = "tensor")
public class DepotAdjusted extends LinearOpMode {

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
        proc.bot.output.marker.setPosition(1);

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
        proc.driveTrainProcessor.align(180);
        proc.intakeProcessor.intakeOff();


        proc.dropMarker();

        proc.driveTrainProcessor.goAngle(7,180,.5);
        proc.driveTrainProcessor.goAngle(5,0,.5);


        proc.driveTrainProcessor.align(135);


        proc.driveTrainProcessor.goAngle(11,-90,1);


        proc.driveTrainProcessor.goAngleStall(40,0,1);





        proc.driveTrainProcessor.goAngle(4,-90,1);
        proc.driveTrainProcessor.goAngle(60,0,1);










    }




}
