package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

/**
 * Created by khadija on 11/30/2018.
 */
@Autonomous(name = "Crater", group = "AutoRoverRukus")
public class Crater extends Processor {

    public ElapsedTime runtime = new ElapsedTime();
    TF1 tf = new TF1();


    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        waitForStart();
        tf.tfod.activate();





        while (opModeIsActive())
        {
            //getInitialPitch();


            descend();
            //analyze();
            driveToGold();
            craterTurn();
            drive(50,0.8);
            dropMarker();
            align(-120);
            drive(60,0.8);
            stop();
        }
    }

}
