package org.firstinspires.ftc.teamcode.RobotProcessor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Robot.*;

import java.util.List;


public class RobotProcessor {
    public Robot bot;
    DriveTrainProcessor driveTrainProcessor;
    HangProcessor hangProcessor;
    IntakeProcessor intakeProcessor;
    OutputProcessor outputProcessor;
    SensorProcessor sensorProcessor;
    ElapsedTime runTime;

    static final double P_SAMPLE_COEFF = .018;
    static final double I_SAMPLE_COEFF = 0;
    static final double D_SAMPLE_COEFF = 0;

    public RobotProcessor() {
        this.bot = new Robot();
        this.driveTrainProcessor = new DriveTrainProcessor(bot, bot.driveTrain, bot.sensors, bot.telemetry, bot.currentOpMode);
        this.hangProcessor = new HangProcessor(this.bot.hang);
        this.intakeProcessor = new IntakeProcessor(this.bot.intake);
        this.outputProcessor = new OutputProcessor(this.bot.output);
        this.sensorProcessor = new SensorProcessor(this.bot.sensors);
        runTime = new ElapsedTime();
    }


    public void descend() {
        int intialTicks = hangProcessor.hang.hangMotor.getCurrentPosition();
        int target = 10 * (int) (9.5 / (Math.PI * (1.5)) * 1680);
        //distance the hang needs to decend divided by the circumfrence multiplied by the pulses per rotation of a 60
        hangProcessor.hang.hangMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hangProcessor.hang.hangMotor.setTargetPosition(target + intialTicks);
        hangProcessor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hangProcessor.setPower(-1.0);

        runTime.reset();
        while (runTime.seconds() < 10 && hangProcessor.hang.hangMotor.isBusy()) {
            driveTrainProcessor.strafeLeft(.2);
            bot.telemetry.addData("location", hangProcessor.hang.hangMotor.getCurrentPosition());

            bot.telemetry.addData("target", hangProcessor.hang.hangMotor.getTargetPosition());
            bot.telemetry.update();
        }
        hangProcessor.hang.hangMotor.setPower(0);
    }

    public void displayLOCATION() {
        if (bot.opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (bot.sensors.tfod != null) {
                bot.sensors.tfod.activate();
            }

            while (bot.opModeIsActive()) {
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
                            int pos = -1;
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
                                    bot.telemetry.addData("Left","");
                                    pos = 1;
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    bot.telemetry.addData("Gold Mineral Position", 3);
                                    bot.telemetry.addData("Right","");
                                    pos = 3;
                                } else {
                                    bot.telemetry.addData("Gold Mineral Position", 2);
                                    bot.telemetry.addData("Center","");
                                    pos = 2;
                                }
                            }
                        }
                        bot.telemetry.update();
                    }
                }
            }
        }

        if (bot.sensors.tfod != null) {
            bot.sensors.tfod.shutdown();
        }


        }
        /*public void turn(){
            //Turn using PID
            // clockwise = negative input, counter-clockwise = positive input

            double heading = bot.sensors.getHeading();
            double angleWanted = target + heading;
            double rcw = 1;
            double integral = 0;
            double previous_error = 0;
            while (rcw != 0 && bot.currentOpMode.opModeIsActive()) {


                double error = angleWanted - bot.sensors.getHeading();
                ;

                while (error > 180 && bot.currentOpMode.opModeIsActive())
                    error -= 360;
                while (error < -180 && bot.currentOpMode.opModeIsActive())
                    error += 360;
                double derivative = error - previous_error;
                //small margin of error for increased speed
           /* if (Math.abs(error) < HEADING_THRESHOLD) {
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
            bot.telemetry.addData("target", target);
            bot.telemetry.addData("speed ", rcw);
            bot.telemetry.addData("error", angleWanted - bot.sensors.getHeading());
            bot.telemetry.addData("angleWanted", angleWanted);
            bot.telemetry.addData("motor power", bot.driveTrain.motorLF.getPower());
            bot.telemetry.addData("rcw", rcw);
            bot.telemetry.addData("P", P_SAMPLE_COEFF * error);
            bot.telemetry.addData("I", I_SAMPLE_COEFF * integral);
            bot.telemetry.addData("D", D_SAMPLE_COEFF * derivative);
            bot.telemetry.update();

            bot.currentOpMode.sleep(20);


        }
        driveTrainProcessor.accelerate(0);*/



    }

