package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/**
 * Created by khadija on 12/7/2018.
 */
@Autonomous
public class TFv1 extends Processor {
    masterV vision;
    sampleRandomPos goldPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");// recommended camera direction
        parameters.vuforiaLicenseKey = "AePFtGr/////AAABmTqOHBpjPkiDumePTFCRXhlueWd30Y4KQGm4uFHSsP2Rdhtlt2kMXLayBPRbBrX7VJLJfwVMqOYsTUjI63iVCna9oEOLQfRkvkNnj9npDzSzaf59ccQXUBGaO2Ga/lt2nX5mr4yJinI6S9qO43TCW4qURaoXFEjeohvQthjAPDpA13up2yKez6Kr0B+7hTTrETsW6UfSeijS7/ylQORuo02fc9IonaKvCPhvdjlINpDh85+M8bHx6KPCNHE1+v4jmNmGTYCLBwbHWb36j4uHHkMSBN51B6uec7J5A34/LKPUYYKwaKOmrzThbOiAEIu9oieG3zSmUx7enMWFox0pBu0jNOLezwLO5nj+/4JV/Rxx";

        vision = new masterV(parameters, hardwareMap, true, masterV.TFLiteAlgorithm.INFER_NONE);
        vision.init();// enables the camera overlay. this will take a couple of seconds
        vision.enable();// enables the tracking algorithms. this might also take a little time

        waitForStart();

        vision.disable();// disables tracking algorithms. this will free up your phone's processing power for other jobs.

        goldPosition = vision.getTfLite().getLastKnownSampleOrder();

        while(opModeIsActive()){
            telemetry.addData("goldPosition was", goldPosition);// giving feedback

            switch (goldPosition){ // using for things in the autonomous program
                case LEFT:
                    telemetry.addLine("going to the left");

                    break;
                case CENTER:
                    telemetry.addLine("going straight");
                    break;
                case RIGHT:
                    telemetry.addLine("going to the right");
                    break;
                case UNKNOWN:
                    telemetry.addLine("staying put");
                    break;
            }

            telemetry.update();
        }

        vision.shutdown();
    }
}
