package org.firstinspires.ftc.teamcode.RobotProcessor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Robot.*;

import java.util.List;

import static org.firstinspires.ftc.teamcode.RobotProcessor.DriveTrainProcessor.ANTI_WINDUP;


public class RobotProcessor {
    public Robot bot;
    public DriveTrainProcessor driveTrainProcessor;
    public HangProcessor hangProcessor;
    public IntakeProcessor intakeProcessor;
    public OutputProcessor outputProcessor;
    public SensorProcessor sensorProcessor;
    public ElapsedTime runTime;

    static final double P_SAMPLE_COEFF = .018;
    static final double I_SAMPLE_COEFF = 0;
    static final double D_SAMPLE_COEFF = 0;

    static final double PIXEL_THRESHOLD = 3;
    static final double ANTI_WINDUP_PIXEL = 3;

    public int locationMineral=-1;


    public RobotProcessor() {
        this.bot = new Robot();
        this.driveTrainProcessor = new DriveTrainProcessor(bot, bot.driveTrain, bot.sensors, bot.telemetry, bot.currentOpMode);
        this.hangProcessor = new HangProcessor(this.bot.hang);
        this.intakeProcessor = new IntakeProcessor(this.bot.intake);
        this.outputProcessor = new OutputProcessor(this.bot.output);
        this.sensorProcessor = new SensorProcessor(this.bot.sensors);
        runTime = new ElapsedTime();
    }
    public RobotProcessor(LinearOpMode currentOpMode, HardwareMap ahwmap, Mode mode, Telemetry telemetry) {
        this.bot = new Robot(currentOpMode,ahwmap,mode,telemetry);
        this.driveTrainProcessor = new DriveTrainProcessor(bot, bot.driveTrain, bot.sensors, bot.telemetry, bot.currentOpMode);
        this.hangProcessor = new HangProcessor(this.bot.hang);
        this.intakeProcessor = new IntakeProcessor(this.bot.intake);
        this.outputProcessor = new OutputProcessor(this.bot.output);
        this.sensorProcessor = new SensorProcessor(this.bot.sensors);
        runTime = new ElapsedTime();
    }

    public void initProc(){
        this.driveTrainProcessor = new DriveTrainProcessor(this.bot, bot.driveTrain, bot.sensors, bot.telemetry, bot.currentOpMode);
        this.hangProcessor = new HangProcessor(this.bot.hang);
        this.intakeProcessor = new IntakeProcessor(this.bot.intake);
        this.outputProcessor = new OutputProcessor(this.bot.output);
        this.sensorProcessor = new SensorProcessor(this.bot.sensors);
        runTime = new ElapsedTime();
    }


    public void descend() {
        int intialTicks = hangProcessor.hang.hangMotor.getCurrentPosition();
        int target = -7200;
        //distance the hang needs to decend divided by the circumfrence multiplied by the pulses per rotation of a 60
        hangProcessor.hang.hangMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hangProcessor.hang.hangMotor.setTargetPosition(target + intialTicks);
        hangProcessor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hangProcessor.setPower(-1.0);

        runTime.reset();
        while (hangProcessor.hang.hangMotor.isBusy()&&bot.opModeIsActive()) {
            driveTrainProcessor.strafeLeft(.2);
            bot.telemetry.addData("location", hangProcessor.hang.hangMotor.getCurrentPosition());

            bot.telemetry.addData("target", hangProcessor.hang.hangMotor.getTargetPosition());
            bot.telemetry.update();
        }
        driveTrainProcessor.stopBotMotors();
        hangProcessor.hang.hangMotor.setPower(0);
    }

    public void identifyLocation() {

        if (bot.opModeIsActive()) {
            /* Activate Tensor Flow Object Detection. */
            if (bot.sensors.tfod != null) {
                bot.sensors.tfod.activate();
            }
            runTime.reset();
            while (locationMineral == -1&&/*runTime.milliseconds()<5000&&*/bot.opModeIsActive()) {
                if (bot.sensors.tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = bot.sensors.tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        bot.telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 3) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;

                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(bot.sensors.LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    bot.telemetry.addData("Gold Mineral Position", 1);
                                    bot.telemetry.addData("Left", "");
                                    locationMineral = 1;
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    bot.telemetry.addData("Gold Mineral Position", 3);
                                    bot.telemetry.addData("Right", "");
                                    locationMineral = 3;
                                } else {
                                    bot.telemetry.addData("Gold Mineral Position", 2);
                                    bot.telemetry.addData("Center", "");
                                    locationMineral = 2;
                                }
                            }
                        }
                        bot.telemetry.update();
                    }
                }
                if(runTime.milliseconds()>5000){
                    locationMineral = 2;
                }
            }
        }

        if (bot.sensors.tfod != null) {
            bot.sensors.tfod.shutdown();
        }
    }

    public void identifyLocationV3() {

        if (bot.opModeIsActive()) {
            /* Activate Tensor Flow Object Detection. */
            if (bot.sensors.tfod != null) {
                bot.sensors.tfod.activate();
            }
            runTime.reset();
            while (locationMineral == -1&&/*runTime.milliseconds()<5000&&*/bot.opModeIsActive()) {
                if (bot.sensors.tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = bot.sensors.tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        bot.telemetry.addData("# Object Detected", updatedRecognitions.size());

                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;

                            for (Recognition recognition : updatedRecognitions) {
                                if(recognition.getTop()<150){
                                    //ignore
                                    bot.telemetry.addData("nah","fam");
                                }
                                else if (recognition.getLabel().equals(bot.sensors.LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                    bot.telemetry.addData("gold boi","fam");

                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    bot.telemetry.addData("Gold Mineral Position", 1);
                                    bot.telemetry.addData("Left", "");
                                    locationMineral = 1;
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    bot.telemetry.addData("Gold Mineral Position", 3);
                                    bot.telemetry.addData("Right", "");
                                    locationMineral = 3;
                                } else {
                                    bot.telemetry.addData("Gold Mineral Position", 2);
                                    bot.telemetry.addData("Center", "");
                                    locationMineral = 2;
                                }
                            }

                            bot.telemetry.addData("yote","yeet");

                        bot.telemetry.update();
                    }
                }
                if(runTime.milliseconds()>1000){
                    locationMineral = 2;
                }
            }
        }

        if (bot.sensors.tfod != null) {
            bot.sensors.tfod.shutdown();
        }
    }

    public void identifyLocationV2() {

        if (bot.opModeIsActive()) {
            /* Activate Tensor Flow Object Detection. */
            if (bot.sensors.tfod != null) {
                bot.sensors.tfod.activate();
            }
            runTime.reset();
            while (locationMineral == -1&&/*runTime.milliseconds()<5000&&*/bot.opModeIsActive()) {
                if (bot.sensors.tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = bot.sensors.tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        bot.telemetry.addData("# Object Detected", updatedRecognitions.size());

                        int goldMineralX = -1000;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;

                        for (Recognition recognition : updatedRecognitions) {
                            if(recognition.getTop()<250){
                                //ignore
                                bot.telemetry.addData("nah","fam");
                            }
                            else if (recognition.getLabel().equals(bot.sensors.LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                                bot.telemetry.addData("gold boi","fam");

                            }
                        }


                        if(goldMineralX!=-1000&& goldMineralX>-100&&goldMineralX<=250)
                        {
                            locationMineral = 1;
                        }
                        if(goldMineralX!=-1000&& goldMineralX>251&&goldMineralX<=450)
                        {
                            locationMineral = 2;
                        }
                        if(goldMineralX!=-1000&& goldMineralX>451)
                        {
                            locationMineral = 3;
                        }
                        bot.telemetry.addData("yote","yeet");

                        bot.telemetry.update();
                    }
                }
                if(runTime.milliseconds()>1000){
                    locationMineral = 2;
                }
            }
        }

        if (bot.sensors.tfod != null) {
            bot.sensors.tfod.shutdown();
        }
    }
    public void turntoGold(){
        if(locationMineral==1)
        {
            driveTrainProcessor.align(30);
        }
        else if (locationMineral==3)
        {
            driveTrainProcessor.align(-30);
        }
        else
        {
            driveTrainProcessor.align(0);
        }
    }

    public void turntoGoldDepot(){
        if(locationMineral==1)
        {
            driveTrainProcessor.align(32);
        }
        else if (locationMineral==3)
        {
            driveTrainProcessor.align(-32);
        }
        else
        {
            driveTrainProcessor.align(0);
        }
    }
    public void alignForSample(){
        if(locationMineral==1)
        {
            driveTrainProcessor.align(-30);
        }
        else if (locationMineral==3)
        {
            driveTrainProcessor.align(30);
        }
        else
        {
            driveTrainProcessor.align(0);
        }
    }

    public void alignFor2ndSample(){
        if(locationMineral==1)
        {
            driveTrainProcessor.align(-30-90);
        }
        else if (locationMineral==3)
        {
            driveTrainProcessor.align(30-90);
        }
        else
        {
            driveTrainProcessor.align(0-90);
        }
    }

    public void setUpToDropDepot(){
        if (locationMineral == 1) {
            //align north, drive north (hit wall), drive south, strafe west?, align northeast
            driveTrainProcessor.align(-45);
            driveTrainProcessor.goAngle(15, 0, .2);
            driveTrainProcessor.goAngle(5,180, .3);
            driveTrainProcessor.goAngle(8, 90, .3);
            driveTrainProcessor.align(-90);
        }
        else if (locationMineral == 3) {
            //align north, drive north (not as far) (hit wall), drive south, strafe west (more) , align northeast
            driveTrainProcessor.align(-45);
            driveTrainProcessor.goAngle(20, 0, .2);
            driveTrainProcessor.goAngle(5,180, .3);
            driveTrainProcessor.goAngle(30, 90, .3);
            driveTrainProcessor.align(-90);
        }
        else {
            //align north, drive north (somewher in between) (hit wall), drive south, strafe west (in between), align northeast
            driveTrainProcessor.align(-45);
            driveTrainProcessor.goAngle(12, 0, .2);
            driveTrainProcessor.goAngle(5,180, .3);
            driveTrainProcessor.goAngle(10, 90, .3);
            driveTrainProcessor.align(-90);
        }
    }

    public void realignForParkDepot() {
        //align south, drive south forward (maybe a foot), strafe west right(hit wall), drive south forward to crater
        driveTrainProcessor.align(135);
        driveTrainProcessor.goAngle(10, 0, .3);
        driveTrainProcessor.goAngle(20, 90, .2);

        driveTrainProcessor.goAngle(20, 0, .5);
        //driveTrainProcessor.goAngle(100, 0, .5);
    }

    public void setUpToDropCrater(){
        if (locationMineral == 1) {
            //align north, drive north (hit wall), drive south, strafe west?, align northeast
            driveTrainProcessor.align(-45);
            driveTrainProcessor.goAngle(15, 0, .2);
            driveTrainProcessor.goAngle(5,180, .3);
            driveTrainProcessor.goAngle(8, 90, .3);
            driveTrainProcessor.align(-90);
        }
        else if (locationMineral == 3) {
            //align north, drive north (not as far) (hit wall), drive south, strafe west (more) , align northeast
            driveTrainProcessor.align(-45);
            driveTrainProcessor.goAngle(10, 0, .2);
            driveTrainProcessor.goAngle(5,180, .3);
            driveTrainProcessor.goAngle(12, 90, .3);
            driveTrainProcessor.align(-90);
        }
        else {
            //align north, drive north (somewher in between) (hit wall), drive south, strafe west (in between), align northeast
            driveTrainProcessor.align(-45);
            driveTrainProcessor.goAngle(12, 0, .2);
            driveTrainProcessor.goAngle(5,180, .3);
            driveTrainProcessor.goAngle(10, 90, .3);
            driveTrainProcessor.align(-90);
        }
    }

    public void realignForParkCrater() {
        //align south, drive south forward (maybe a foot), strafe west right(hit wall), drive south forward to crater
        driveTrainProcessor.turn(-225);
        driveTrainProcessor.goAngle(10, 0, .3);
        driveTrainProcessor.goAngle(20, -90, .2);

        driveTrainProcessor.goAngle(20, 0, .5);
        //driveTrainProcessor.goAngle(100, 0, .5);
    }

    public void dropMarker(){
        bot.output.marker.setPosition(0);
        bot.currentOpMode.sleep(1500);

    }
    public void displayTFOD() {
        if (bot.opModeIsActive()) {
            /* Activate Tensor Flow Object Detection. */
            activateTFOD();


        }
        while(bot.opModeIsActive()) {
            if (bot.sensors.tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = bot.sensors.tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    bot.telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 3) {

                    }
                    int goldMineralX = -1;
                    int goldMineralY = -1;
                    int goldMineralBot = -1;
                    int goldMineralTop = -1;
                    int goldMineralWidth = -1;
                    int goldMineralHeight = -1;
                    double goldMineralAngle = -1;

                    int silverMineral1X = -1;
                    int silverMineral2X = -1;
                    int pos = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(bot.sensors.LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                            goldMineralY = (int) recognition.getRight();
                            goldMineralBot = (int) recognition.getBottom();
                            goldMineralTop = (int) recognition.getTop();
                            goldMineralWidth = (int) recognition.getWidth();
                            goldMineralHeight = (int) recognition.getHeight();
                            goldMineralAngle = (int) recognition.estimateAngleToObject(AngleUnit.DEGREES);


                        } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                        } else {
                            silverMineral2X = (int) recognition.getLeft();
                        }
                    }
                    bot.telemetry.addData("GoldmineralX", goldMineralX);
                    bot.telemetry.addData("goldMineralY", goldMineralY);
                    bot.telemetry.addData("goldMineralBot", goldMineralBot);
                    bot.telemetry.addData("goldMineralTop", goldMineralTop);
                    bot.telemetry.addData("goldMineralWidth", goldMineralWidth);
                    bot.telemetry.addData("goldMineralHeight", goldMineralHeight);
                    bot.telemetry.addData("goldMineralAngle", goldMineralAngle);
                    bot.telemetry.addData("silverMineral1X", silverMineral1X);
                    bot.telemetry.addData("silverMineral2X", silverMineral2X);


                }
                bot.telemetry.update();
            }
        }
        deactivateTFOD();

    }

    public void displayINIT(){

            activateTFOD();



        while(!bot.opModeIsActive()&&!bot.currentOpMode.isStopRequested()) {
            if (bot.sensors.tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = bot.sensors.tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    bot.telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 3) {

                    }
                    int goldMineralX = -1;
                    int goldMineralY = -1;
                    int goldMineralBot = -1;
                    int goldMineralTop = -1;
                    int goldMineralWidth = -1;
                    int goldMineralHeight = -1;
                    double goldMineralAngle = -1;

                    int silverMineral1X = -1;
                    int silverMineral2X = -1;
                    int pos = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(bot.sensors.LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                            goldMineralY = (int) recognition.getRight();
                            goldMineralBot = (int) recognition.getBottom();
                            goldMineralTop = (int) recognition.getTop();
                            goldMineralWidth = (int) recognition.getWidth();
                            goldMineralHeight = (int) recognition.getHeight();
                            goldMineralAngle = (int) recognition.estimateAngleToObject(AngleUnit.DEGREES);


                        } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                        } else {
                            silverMineral2X = (int) recognition.getLeft();
                        }
                    }
                    bot.telemetry.addData("GoldmineralX", goldMineralX);
                    bot.telemetry.addData("goldMineralY", goldMineralY);
                    bot.telemetry.addData("goldMineralBot", goldMineralBot);
                    bot.telemetry.addData("goldMineralTop", goldMineralTop);
                    bot.telemetry.addData("goldMineralWidth", goldMineralWidth);
                    bot.telemetry.addData("goldMineralHeight", goldMineralHeight);
                    bot.telemetry.addData("goldMineralAngle", goldMineralAngle);
                    bot.telemetry.addData("silverMineral1X", silverMineral1X);
                    bot.telemetry.addData("silverMineral2X", silverMineral2X);


                }
                bot.telemetry.update();
            }
        }

    }


    public void turnSample(){///BRIJJD"INEFNEOFNWEDIN
            //Turn using PID
            // clockwise = negative input, counter-clockwise = positive input

            activateTFOD();
            double pixelheading = goldLocation();
            double pixelWanted = 540; // center of the camera
            double rcw = 1;
            double integral = 0;
            double previous_error = 0;
            while (rcw != 0 && bot.currentOpMode.opModeIsActive()) {


                double error = pixelWanted - goldLocation();
                double derivative = error - previous_error;
                //small margin of error for increased speed
           if (Math.abs(error) < PIXEL_THRESHOLD) {
                error = 0;
            }
            //prevents integral from growing too large
            if (Math.abs(error) < ANTI_WINDUP_PIXEL && error != 0) {
                integral += error;
            } else {
                integral = 0;
            }
            if (integral > (50 / I_SAMPLE_COEFF)) {
                integral = 50 / I_SAMPLE_COEFF;
            }
            if (error == 0) {
                derivative = 0;
            }
            rcw = P_SAMPLE_COEFF * error + I_SAMPLE_COEFF * integral + D_SAMPLE_COEFF * derivative;
            previous_error = error;
            driveTrainProcessor.accelerate(rcw);

            bot.telemetry.addData("first angle", bot.sensors.getHeading());
            //telemetry.addData("second angle", ref.secondAngle);
            //telemetry.addData("third angle", ref.thirdAngle);
            bot.telemetry.addData("target", "lol");
            bot.telemetry.addData("speed ", rcw);
            bot.telemetry.addData("error", pixelWanted - goldLocation());
            bot.telemetry.addData("angleWanted", pixelWanted);
            bot.telemetry.addData("motor power", bot.driveTrain.motorLF.getPower());
            bot.telemetry.addData("rcw", rcw);
            bot.telemetry.addData("P", P_SAMPLE_COEFF * error);
            bot.telemetry.addData("I", I_SAMPLE_COEFF * integral);
            bot.telemetry.addData("D", D_SAMPLE_COEFF * derivative);

            bot.telemetry.update();

            bot.currentOpMode.sleep(20);

            }
        deactivateTFOD();
        driveTrainProcessor.accelerate(0);

    }


    public int goldLocation(){
        int goldX = 0;
        if (bot.opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */

            if (bot.sensors.tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = bot.sensors.tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    bot.telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 3) {

                    }
                    int goldMineralX = -1;


                    int silverMineral1X = -1;
                    int silverMineral2X = -1;
                    int pos = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(bot.sensors.LABEL_GOLD_MINERAL)) {
                            goldX = goldMineralX;
                            goldMineralX = (int) recognition.getLeft();



                        } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                        } else {
                            silverMineral2X = (int) recognition.getLeft();
                        }
                    }
                    bot.telemetry.addData("GoldmineralX", goldMineralX);
                    bot.telemetry.addData("silverMineral1X", silverMineral1X);
                    bot.telemetry.addData("silverMineral2X", silverMineral2X);


                }
                bot.telemetry.update();
            }

        }

        return goldX;
    }

    public void turnSampleAngle(){
        activateTFOD();
        double heading = angleGold();
        double angle = 0; // center of the camera
        double rcw = 1;
        double integral = 0;
        double previous_error = 0;
        while (rcw != 0 && bot.currentOpMode.opModeIsActive()) {


            double error = angle - angleGold();
            double derivative = error - previous_error;
            //small margin of error for increased speed
            if (Math.abs(error) < 2) {
                error = 0;
            }
            //prevents integral from growing too large
            if (Math.abs(error) < ANTI_WINDUP && error != 0) {
                integral += error;
            } else {
                integral = 0;
            }
            if (integral > (50 / I_SAMPLE_COEFF)) {
                integral = 50 / I_SAMPLE_COEFF;
            }
            if (error == 0) {
                derivative = 0;
            }
            rcw = P_SAMPLE_COEFF * error + I_SAMPLE_COEFF * integral + D_SAMPLE_COEFF * derivative;
            previous_error = error;
            driveTrainProcessor.accelerate(rcw);

            bot.telemetry.addData("first angle", bot.sensors.getHeading());
            //telemetry.addData("second angle", ref.secondAngle);
            //telemetry.addData("third angle", ref.thirdAngle);
            bot.telemetry.addData("target", "lol");
            bot.telemetry.addData("speed ", rcw);
            bot.telemetry.addData("error", angle - angleGold());
            bot.telemetry.addData("angleWanted", angle);
            bot.telemetry.addData("motor power", bot.driveTrain.motorLF.getPower());
            bot.telemetry.addData("rcw", rcw);
            bot.telemetry.addData("P", P_SAMPLE_COEFF * error);
            bot.telemetry.addData("I", I_SAMPLE_COEFF * integral);
            bot.telemetry.addData("D", D_SAMPLE_COEFF * derivative);
            bot.telemetry.update();

            bot.currentOpMode.sleep(20);

        }
        deactivateTFOD();
        driveTrainProcessor.accelerate(0);
    }

    public void activateTFOD(){
        if (bot.sensors.tfod != null) {
            bot.sensors.tfod.activate();
        }
    }

    public void deactivateTFOD(){
        if (bot.sensors.tfod != null) {
            bot.sensors.tfod.shutdown();
        }
    }

    public int angleGold() {
        int goldAngle = 0;
        if (bot.opModeIsActive()) {
            /* Activate Tensor Flow Object Detection. */

            if (bot.sensors.tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = bot.sensors.tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    bot.telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 3) {

                    }
                    int goldMineralAngle = -1;


                    int silverMineral1X = -1;
                    int silverMineral2X = -1;
                    int pos = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(bot.sensors.LABEL_GOLD_MINERAL)) {
                            goldMineralAngle = (int) recognition.estimateAngleToObject(AngleUnit.DEGREES);
                            goldAngle = goldMineralAngle;


                        } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                        } else {
                            silverMineral2X = (int) recognition.getLeft();
                        }
                    }
                    bot.telemetry.addData("GoldmineralX", goldMineralAngle);
                    bot.telemetry.addData("silverMineral1X", silverMineral1X);
                    bot.telemetry.addData("silverMineral2X", silverMineral2X);


                }
                bot.telemetry.update();
            }

        }

        return goldAngle;
    }

    public double distanceToWall(){
        double ret;
        ret = 45;
        if (locationMineral == 1){
         ret = 44;
        }
        else if(locationMineral == 3){
            ret = 46;
        }


        return ret;
    }
    public double goToDep(){
        double ret;
        ret = 27;
        if (locationMineral == 2){
            ret = 23;
        }
        return ret;
    }
    public double distanceStrafe(){
        double ret;
        if (locationMineral == 1){
            ret = 27;
        }
        else if(locationMineral == 2 ){
            ret = 30;
        }
        else{
            ret = 38;
        }
        return ret;
    }

    public void goArc(double distance,double frontAngle, double turnAngle,double power) {
        driveTrainProcessor.resetEnc();
        driveTrainProcessor.enterEnc();

        /*
        Drives the robot in an arc, by driving two separate arcs using two pairs of wheels. One pair drives the outer arc,
        and the other drives the inner arc. Powering the outer wheels faster than the inner wheels the creates the desired arc.
        */

        double turnAngel = Math.toRadians(turnAngle);
        double radius = (distance/2)/Math.sin(Math.abs(turnAngel)/2);

        //compute how many rotations to move desired distance
        double rotations = distance / (driveTrainProcessor.OMNI_WHEEL_CIRCUMFERENCE);

        //the angle on the robot that is considered the front, in radians
        double frontAngel = Math.toRadians(frontAngle);

        //extract the sine and cosine of the frontAngle
        double xFront = Math.cos(frontAngel);
        double yFront = Math.sin(frontAngel);

        //find the ratio between the radius of the outer and inner circles, each 9 inches from the center of the robot
        double ratio = (radius - 9) / (radius + 9);

        int expRF = (int) Math.signum((yFront - xFront) * turnAngle) >>> 31;
        int expLF = (int) Math.signum((-yFront - xFront) * turnAngle) >>> 31;
        int expLB = (int) Math.signum((-yFront + xFront) * turnAngle) >>> 31;
        int expRB = (int) Math.signum((yFront + xFront) * turnAngle) >>> 31;

        double powerRF = power * Math.pow(ratio, expRF) * Math.signum(yFront - xFront);
        double powerLF = power * Math.pow(ratio, expLF) * Math.signum(-yFront - xFront);
        double powerLB = power * Math.pow(ratio, expLB) * Math.signum(-yFront + xFront);
        double powerRB = power * Math.pow(ratio, expRB) * Math.signum(yFront + xFront);

        //sets up the PID turn`
        Orientation ref = bot.sensors.imu.getAngularOrientation();
        double heading = ref.firstAngle;
        double angleWanted = turnAngle + heading;
        double integral = 0;
        double previous_error = 0;
        double rcw = 1;

        //combines the tick condition with a gyroscopic sensor condition to ensure accuracy
        while (rcw !=0 && bot.opModeIsActive()) {

            ref = bot.sensors.imu.getAngularOrientation();

            double firstAngle = ref.firstAngle;
            if(ref.firstAngle<0&&turnAngle>0){
                firstAngle = 360 + ref.firstAngle;
            }
            if(ref.firstAngle>0&&turnAngle<0){
                firstAngle = -360+ ref.firstAngle;
            }

            double error = Math.abs(angleWanted) - Math.abs(firstAngle);

            double derivative = error - previous_error;
            //small margin of error for increased speed
            if (Math.abs(error) < 3) {
                error = 0;
            }
            //prevents integral from growing too large
            if (Math.abs(error) < ANTI_WINDUP && error != 0) {
                integral += error;
            } else {
                integral = 0;
            }
            if (integral > (50 / driveTrainProcessor.I_TURN_COEFF)) {
                integral = 50 / driveTrainProcessor.I_TURN_COEFF;
            }
            if (error == 0) {
                derivative = 0;
            }
            previous_error = error;
            integral=0;
            derivative=0;
            rcw = driveTrainProcessor.P_TURN_COEFF * error + driveTrainProcessor.I_TURN_COEFF*integral + driveTrainProcessor.D_TURN_COEFF * derivative;
            //sets the power, which, due to the exponents, is either the ratio or 1. User can change power factor for lower rcws.
            bot.driveTrain.motorRF.setPower(powerRF * rcw);
            bot.driveTrain.motorLF.setPower(powerLF * rcw);
            bot.driveTrain.motorLB.setPower(powerLB * rcw);
            bot.driveTrain.motorRB.setPower(powerRB * rcw);

            bot.telemetry.addData("Path2", "Running at %7d :%7d",
                    bot.driveTrain.motorLB.getCurrentPosition(),
                    bot.driveTrain.motorLF.getCurrentPosition(),
                    bot.driveTrain.motorRB.getCurrentPosition(),
                    bot.driveTrain.motorRF.getCurrentPosition());
            bot.telemetry.addData("target", "Running at %7d :%7d",
                    bot.driveTrain.motorLB.getTargetPosition(),
                    bot.driveTrain.motorLF.getTargetPosition(),
                    bot.driveTrain.motorRB.getTargetPosition(),
                    bot.driveTrain.motorRF.getTargetPosition());
            bot.telemetry.addData("first angle", firstAngle);
            bot.telemetry.addData("second angle", ref.secondAngle);
            bot.telemetry.addData("third angle", ref.thirdAngle);
            bot.telemetry.addData("target", turnAngle);
            bot.telemetry.addData("error", angleWanted - ref.firstAngle);
            bot.telemetry.addData("angleWanted", angleWanted);
            bot.telemetry.addData("motor power", bot.driveTrain.motorLF.getPower());
            bot.telemetry.addData("rcw", rcw);
            bot.telemetry.addData("P", driveTrainProcessor.P_TURN_COEFF * error);
            bot.telemetry.addData("I", driveTrainProcessor.I_TURN_COEFF * integral);
            bot.telemetry.addData("D", driveTrainProcessor.D_TURN_COEFF * derivative);
            bot.telemetry.update();

            bot.currentOpMode.sleep(20);
        }
        driveTrainProcessor.stopBotMotors();
        driveTrainProcessor.enterEnc();
    }
}