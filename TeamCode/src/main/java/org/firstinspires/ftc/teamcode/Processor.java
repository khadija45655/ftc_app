package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_GOLD_MINERAL;

/**
 * Created by khadija on 11/30/2018.
 */
public abstract class Processor extends LinearOpMode{
    Map bot = new Map();

    static final double COUNTS_PER_MOTOR_REV = 1120;
    static final double DRIVE_GEAR_REDUCTION = 1.286;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415));
    static final double TURN_SPEED = 0.4;
    static final double ANTI_WINDUP = 2;
    static final double P_TURN_COEFF = .018;
    static final double I_TURN_COEFF = .01;
    static final double D_TURN_COEFF = .026;

    static final double HEADING_THRESHOLD = 1.5;
    static final double ENCODER_THRESHOLD = 1;


    double depotAngle = 0;
    double craterAngle = 0;
    double order = 1;

    TF1 tf = new TF1();


    public void drive(double distance,double speed) {

        double speedLeft = speed;
        double speedRight = speed;
        int leftTarget;
        int rightTarget;

        bot.motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        leftTarget = ((bot.motorLeftFront.getCurrentPosition()+bot.motorLeftBack.getCurrentPosition())/2 + (int) (distance * COUNTS_PER_INCH));
        rightTarget = ((bot.motorRightFront.getCurrentPosition()+bot.motorRightBack.getCurrentPosition())/2 + (int) (distance * COUNTS_PER_INCH));


        bot.motorLeftFront.setTargetPosition(leftTarget);
        bot.motorLeftBack.setTargetPosition(leftTarget);
        bot.motorRightFront.setTargetPosition(rightTarget);
        bot.motorRightBack.setTargetPosition(rightTarget);

        bot.motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bot.motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bot.motorLeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bot.motorRightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        bot.motorLeftFront.setPower(Range.clip(absLol((double)(bot.motorLeftFront.getCurrentPosition()-bot.motorLeftFront.getTargetPosition())/leftTarget), .1,speed));
        bot.motorRightFront.setPower(Range.clip(absLol((double)(bot.motorRightFront.getCurrentPosition()-bot.motorRightFront.getTargetPosition())/rightTarget),.1,speed));
        bot.motorLeftBack.setPower(Range.clip(absLol((double)(bot.motorLeftBack.getCurrentPosition()-bot.motorLeftBack.getTargetPosition())/leftTarget), .1,speed));
        bot.motorRightBack.setPower(Range.clip(absLol((double)(bot.motorRightBack.getCurrentPosition()-bot.motorRightBack.getTargetPosition())/rightTarget),.1,speed));
        while (bot.motorRightFront.isBusy() && bot.motorRightBack.isBusy()&& bot.motorLeftBack.isBusy()&&
                bot.motorLeftFront.isBusy()) {



        }
        bot.motorLeftFront.setPower(0);
        bot.motorRightFront.setPower(0);
        bot.motorLeftBack.setPower(0);
        bot.motorRightBack.setPower(0);
        bot.motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    /*public double getinitialPitch()
    {
        initialPitch = bot.imu.getAngularOrientation().thirdAngle;
        return initialPitch;
    }*/
    public void dropMarker() throws InterruptedException {
        bot.marker.setPosition(0.0);
        wait(2000);
        bot.marker.setPosition(1.0);
    }
    public float angularOffset() {
        Orientation initial = bot.imu.getAngularOrientation();
        float x = initial.firstAngle;
        return x;
    }
    public void align(double offset) {
        //turns to a specific angle
        double error = angularOffset();
        double diff = offset - error;
        turn(diff);
    }
    public void turn(double target) {
        //Turn using PID
        // clockwise = negative input, counter-clockwise = positive input
        Orientation ref = bot.imu.getAngularOrientation();
        double heading = ref.firstAngle;
        double angleWanted = target + heading;
        double speed = 1;
        double integral = 0;
        double previous_error = 0;
        while (speed != 0 && opModeIsActive()) {


            ref = bot.imu.getAngularOrientation();
            double error = angleWanted - ref.firstAngle;

            while (error > 180 && opModeIsActive())
                error -= 360;
            while (error < -180 && opModeIsActive())
                error += 360;
            double derivative = error - previous_error;
            //small margin of error for increased speed
            if (Math.abs(error) < HEADING_THRESHOLD) {
                error = 0;
            }
            //prevents integral from growing too large
            if (Math.abs(error) < ANTI_WINDUP && error != 0) {
                integral += error;
            } else {
                integral = 0;
            }
            if (integral > (50 / I_TURN_COEFF)) {
                integral = 50 / I_TURN_COEFF;
            }
            if (error == 0) {
                derivative = 0;
            }
            double rcw = P_TURN_COEFF * error + I_TURN_COEFF * integral + D_TURN_COEFF * derivative;
            telemetry.addData("first angle", ref.firstAngle);
            telemetry.addData("second angle", ref.secondAngle);
            telemetry.addData("third angle", ref.thirdAngle);
            telemetry.addData("target", target);
            telemetry.addData("speed ", speed);
            telemetry.addData("error", angleWanted - ref.firstAngle);
            telemetry.addData("angleWanted", angleWanted);
            telemetry.addData("motor power", bot.motorLeftFront.getPower());
            telemetry.addData("rcw", rcw);
            telemetry.addData("P", P_TURN_COEFF * error);
            telemetry.addData("I", I_TURN_COEFF * integral);
            telemetry.addData("D", D_TURN_COEFF * derivative);
            telemetry.update();
            previous_error = error;
            speed = rcw;
            accelerate(speed);
            sleep(20);


        }
        accelerate(0);
    }
    private void accelerate(double speed) {
        double clip_speed = Range.clip(speed, -1, 1);
        bot.motorLeftFront.setPower(clip_speed);
        bot.motorRightFront.setPower(clip_speed);
        bot.motorLeftBack.setPower(clip_speed);
        bot.motorRightBack.setPower(clip_speed);
    }


    /*public void turnOverLeftV2(double angle) {
        bot.motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double correction;
        double integral = 0;
        double error;
        double angleWanted = angle + heading*.9833;
        double speedRight;
        int beginEncoderValue = bot.motorLeftFront.getCurrentPosition();
        double speed;
        double sensorDelay = 200;//This Number needs to be ajusted
        boolean atLocationCheck = false;
        // int     newLeftTarget;
        //int     newRightTarget;
        // need to corret the angle to make sure the error is right
        while (!atLocationCheck) {
            error = angleWanted - heading*.9833;
            while (error > 180)
                error -= 360;
            while (error <= -180)
                error += 360;
            integral = integral + (error * sensorDelay);
            correction = Range.clip(error * P_TURN_COEFF + I_TURN_COEFF * integral, -1, 1);
            if (Math.abs(error) <= HEADING_THRESHOLD) {
                atLocationCheck = true;
                speedRight = 0;// turn over the left side
                //speedRight=0;
            } else {
                speedRight = TURN_SPEED * correction;
                //speedLeft = -speedRight;
            }
            bot.motorLeftFront.setPower(keepZero(beginEncoderValue,bot.motorLeftFront.getCurrentPosition()));
            bot.motorLeftBack.setPower(keepZero(beginEncoderValue,bot.motorLeftBack.getCurrentPosition()));
            bot.motorRightFront.setPower(speedRight);
            bot.motorRightBack.setPower(speedRight);
            telemetry.addData("Err/integral",  "%f/%f",  error, integral);
            telemetry.addData("Actual",  "%d:%d",      bot.motorLeftFront.getCurrentPosition(),
                    bot.motorRightFront.getCurrentPosition());
            telemetry.addData("start",beginEncoderValue);
            telemetry.addData("power",speedRight);
            telemetry.addData("heading",heading);
            telemetry.update();
            //bot.bot.bot.motorLeft.setPower(speedLeft);
        }
        bot.motorLeftFront.setPower(0);
        bot.motorRightFront.setPower(0);
    }
*/
    /*public void turnOverRightV2(double angle) {
        double correction;
        double integral = 0;
        double error;
        double angleWanted = angle + firstAngle*.9833;
        double speedLeft;
        int beginEncoderValue = (bot.motorRightFront.getCurrentPosition()+bot.motorRightBack.getCurrentPosition())/2;
        double speed;
        double sensorDelay = 200;//This Number needs to be ajusted
        boolean atLocationCheck = false;
        // int     newLeftTarget;
        //int     newRightTarget;
        // need to corret the angle to make sure the error is right
        while (!atLocationCheck) {
            error = angleWanted - firstAngle*.9833;
            while (error > 180)
                error -= 360;
            while (error <= -180)
                error += 360;
            integral = integral + (error * sensorDelay);
            correction = Range.clip(error * P_TURN_COEFF + I_TURN_COEFF * integral, -1, 1);
            if (Math.abs(error) <= HEADING_THRESHOLD) {
                atLocationCheck = true;
                speedLeft = 0;// turn over the left side
                //speedRight=0;
            } else {
                speedLeft = TURN_SPEED * -correction;
                //speedLeft = -speedRight;
            }
            bot.motorLeftFront.setPower(speedLeft);
            bot.motorLeftBack.setPower(speedLeft);
            bot.motorRightFront.setPower(keepZero(beginEncoderValue,bot.motorRightFront.getCurrentPosition()));
            bot.motorRightBack.setPower(keepZero(beginEncoderValue,bot.motorRightBack.getCurrentPosition()));
            telemetry.addData("Err/integral",  "%f/%f",  error, integral);
            telemetry.addData("Actual",  "%d:%d",      bot.motorLeftFront.getCurrentPosition(),
                    bot.motorRightFront.getCurrentPosition());
            telemetry.addData("start",beginEncoderValue);
            telemetry.addData("power",speedLeft);
            telemetry.addData("heading",ref.firstAngle);
            telemetry.update();
            //bot.bot.bot.motorLeft.setPower(speedLeft);
        }
        bot.motorLeftFront.setPower(0);
        bot.motorLeftBack.setPower(0);
        bot.motorRightBack.setPower(0);
        bot.motorRightFront.setPower(0);
    }
*/

    public double keepZero(int startEncoderValue,int currentEncoder) {
        double ret = 0;
        if(startEncoderValue<currentEncoder){
            ret = -.05;
        }
        if(startEncoderValue>currentEncoder){
            ret = .05;
        }
        return ret;
    }

    public double scaleValue(int difference) {
        double ret = 0;
        double newPower = 0.0;
        if (difference > 0 && difference < 10) {
            newPower = 0.95;
        }
        if (difference >= 10 && difference < 20) {
            newPower = 0.9;
        }
        if (difference >= 20 && difference < 30) {
            newPower = 0.8;
        }
        if (difference >= 30 && difference < 40) {
            newPower = 0.7;
        }
        if (difference >= 40 && difference < 50) {
            newPower = 0.6;
        }
        if (difference >= 50 && difference < 60) {
            newPower = 0.5;
        }
        if (difference >= 60 && difference < 70) {
            newPower = 0.4;
        }
        if (difference >= 70 && difference < 80) {
            newPower = 0.3;
        }
        if (difference >= 80) {
            newPower = 0.3;
        }
        ret = newPower;
        return ret;
    }


    public static double absolute(int a){

        return (a<=0) ? 0-a : a;
    }
    public double absLol(double r){
        double ret = r;
        if(r <= 0 ){
            ret = -r;
        }
        return ret;

    }

    public void driveDownHang()
    {
        bot.runtime.reset();
        while(bot.runtime.seconds()<1.5) {
            bot.hangMotor.setPower(-0.5);
        }
    }
    /*public void analyze()
    {
        if(getOrder() == 2)
        {
            depotAngle = 0;
        }
        if(getOrder() == 3) {
            turnOverRightV2(30);
            depotAngle = -45;
        }
        if(getOrder() == 1) {
            turnOverLeftV2(30);
            depotAngle = 45;
        }
    }
*/
    public void depotTurn(){

        turn(depotAngle);
    }
    public void craterTurn(){
        craterAngle = depotAngle + 90;
        turn(craterAngle);
    }
    public void descend()
    {
        ElapsedTime x = new ElapsedTime();
        x.reset();
        while(x.seconds()<8.5) {
            bot.hangMotor.setPower(0.5);
        }
    }
    public void driveToGold() {
        bot.runtime.reset();
        while (/*Math.abs(bot.imu.getAngularOrientation().thirdAngle - bot.initialPitch)>20*/bot.runtime.seconds() < 2.5) {
            bot.motorLeftFront.setPower(0.5);
            bot.motorLeftBack.setPower(0.5);
            bot.motorRightFront.setPower(0.5);
            bot.motorRightBack.setPower(0.5);
        }
    }
    public void findGold() {
        int gold = -1;
        while (opModeIsActive()) {
            if (tf.tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tf.tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 3) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                        }
                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Left");
                                gold = 0;
                            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Right");
                                gold = 2;
                            } else {
                                telemetry.addData("Gold Mineral Position", "Center");
                                gold = 1;
                            }
                        }
                    }
                    telemetry.update();
                }
            }

        }

    }
    public void whereWeAt(int pos){ //TODO: Locate where we are on the field using vuforia's position system and pictures


    }

}

