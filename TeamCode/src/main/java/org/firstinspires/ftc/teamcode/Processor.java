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
public abstract class Processor extends LinearOpMode {
    Map bot = new Map();
    static final double P_TURN_COEFF = .018;
    static final double I_TURN_COEFF = 0;
    static final double D_TURN_COEFF = 0;
    static final double HEADING_THRESHOLD = 5;
    static final double ANTI_WINDUP = 2;

    public final static double TICKSPERROTATION = 537.6;
    public static final double OMNI_WHEEL_CIRCUMFERENCE = 4 * Math.PI;
    public final static int DIAMETER_OF_WHEEL = 4;
    public static final double DRIVE_GEAR_REDUCTION = 1.286;

    ElapsedTime runTime = new ElapsedTime();
    public int order = 2;

    public void resetEnc() {
        bot.motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void enterEnc() {
        bot.motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void enterPosenc() {
        bot.motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bot.motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bot.motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bot.motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void stopBotMotors() {
        bot.motorRF.setPower(0);
        bot.motorLF.setPower(0);
        bot.motorLB.setPower(0);
        bot.motorRB.setPower(0);

    }

    public void goAngle(double dist, double angle, double power) {
        resetEnc();
        enterPosenc();
        angle = angle-180;
        double angel = Math.toRadians(angle);
        double x = Math.cos(angel);
        double y = Math.sin(angel);
        double distance = dist / (OMNI_WHEEL_CIRCUMFERENCE);
        double ticks = TICKSPERROTATION * distance;
        int ticksRF = (int) Math.round(ticks * (y - x));
        int ticksLF = (int) Math.round(ticks * (-y - x));
        int ticksLB = (int) Math.round(ticks * (-y + x));
        int ticksRB = (int) Math.round(ticks * (y + x));
        bot.motorLF.setTargetPosition(ticksLF);
        bot.motorRF.setTargetPosition(ticksRF);
        bot.motorRB.setTargetPosition(ticksRB);
        bot.motorLB.setTargetPosition(ticksLB);
        bot.motorRF.setPower(power * (y - x));
        bot.motorLF.setPower(-power * (-y - x));
        bot.motorLB.setPower(-power * (-y + x));
        bot.motorRB.setPower(power * (y + x));
        while ((bot.motorLB.isBusy() && bot.motorRB.isBusy() && bot.motorRF.isBusy() && bot.motorLF.isBusy()) && opModeIsActive()) {
            telemetry.addData("Path2", "Running at %7d :%7d",
                    bot.motorLB.getCurrentPosition(),
                    bot.motorLF.getCurrentPosition(),
                    bot.motorRB.getCurrentPosition(),
                    bot.motorRF.getCurrentPosition());
            telemetry.addData("target", "Running at %7d :%7d",
                    bot.motorLB.getTargetPosition(),
                    bot.motorLF.getTargetPosition(),
                    bot.motorRB.getTargetPosition(),
                    bot.motorRF.getTargetPosition());
            telemetry.addData("gyroHeading",bot.imu.getAngularOrientation());
            telemetry.update();
        }
        stopBotMotors();
        enterEnc();
    }

    public void driveForward(double power){
        bot.motorRF.setPower(power);
        bot.motorLF.setPower(-power);
        bot.motorLB.setPower(-power);
        bot.motorRB.setPower(power);
    }
    public void driveBackward(double power){
        bot.motorRF.setPower(-power);
        bot.motorLF.setPower(power);
        bot.motorLB.setPower(power);
        bot.motorRB.setPower(-power);
    }
    public void strafeLeft(double power){
        bot.motorRF.setPower(-power);
        bot.motorLF.setPower(-power);
        bot.motorLB.setPower(power);
        bot.motorRB.setPower(power);
    }
    public void strafeRight(double power){
        bot.motorRF.setPower(-power);
        bot.motorLF.setPower(power);
        bot.motorLB.setPower(power);
        bot.motorRB.setPower(-power);
    }
    public void turn(double target) {
        //Turn using PID
        // clockwise = negative input, counter-clockwise = positive input
        Orientation ref = bot.imu.getAngularOrientation();
        double heading = ref.firstAngle;
        double angleWanted = target + heading;
        double rcw =1;
        double speed = .2;
        double integral = 0;
        double previous_error = 0;
        while (rcw != 0 && opModeIsActive()) {


            ref = bot.imu.getAngularOrientation();
            double error = Math.signum(angleWanted - ref.firstAngle);

            if (Math.abs(error) < HEADING_THRESHOLD) {
                error = 0;
            }

            accelerate(speed*error);
        }
        accelerate(0);
    }
    public void goArc(double distance,double frontAngle, double turnAngle,double power) {
        resetEnc();
        enterEnc();

        /*
        Drives the robot in an arc, by driving two separate arcs using two pairs of wheels. One pair drives the outer arc,
        and the other drives the inner arc. Powering the outer wheels faster than the inner wheels the creates the desired arc.
        */

        double turnAngel = Math.toRadians(turnAngle);
        double radius = (distance/2)/Math.sin(Math.abs(turnAngel)/2);

        //compute how many rotations to move desired distance
        double rotations = distance / (OMNI_WHEEL_CIRCUMFERENCE);

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
        Orientation ref = bot.imu.getAngularOrientation();
        double heading = ref.firstAngle;
        double angleWanted = turnAngle + heading;
        double integral = 0;
        double previous_error = 0;
        double rcw = 1;

        //combines the tick condition with a gyroscopic sensor condition to ensure accuracy
        while (rcw !=0 && opModeIsActive()) {

            ref = bot.imu.getAngularOrientation();

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
            previous_error = error;
            integral=0;
            derivative=0;
            rcw = P_TURN_COEFF * error + I_TURN_COEFF*integral + D_TURN_COEFF * derivative;
            //sets the power, which, due to the exponents, is either the ratio or 1. User can change power factor for lower rcws.
            bot.motorRF.setPower(powerRF * rcw);
            bot.motorLF.setPower(powerLF * rcw);
            bot.motorLB.setPower(powerLB * rcw);
            bot.motorRB.setPower(powerRB * rcw);

            telemetry.addData("Path2", "Running at %7d :%7d",
                    bot.motorLB.getCurrentPosition(),
                    bot.motorLF.getCurrentPosition(),
                    bot.motorRB.getCurrentPosition(),
                    bot.motorRF.getCurrentPosition());
            telemetry.addData("target", "Running at %7d :%7d",
                    bot.motorLB.getTargetPosition(),
                    bot.motorLF.getTargetPosition(),
                    bot.motorRB.getTargetPosition(),
                    bot.motorRF.getTargetPosition());
            telemetry.addData("first angle", firstAngle);
            telemetry.addData("second angle", ref.secondAngle);
            telemetry.addData("third angle", ref.thirdAngle);
            telemetry.addData("target", turnAngle);
            telemetry.addData("error", angleWanted - ref.firstAngle);
            telemetry.addData("angleWanted", angleWanted);
            telemetry.addData("motor power", bot.motorLF.getPower());
            telemetry.addData("rcw", rcw);
            telemetry.addData("P", P_TURN_COEFF * error);
            telemetry.addData("I", I_TURN_COEFF * integral);
            telemetry.addData("D", D_TURN_COEFF * derivative);
            telemetry.update();

            sleep(20);
        }
        stopBotMotors();
        enterEnc();
    }

    private void accelerate(double speed) {
        double clip_speed = Range.clip(speed, -1, 1);
        bot.motorLF.setPower(clip_speed);
        bot.motorRF.setPower(clip_speed);
        bot.motorRB.setPower(clip_speed);
        bot.motorLB.setPower(clip_speed);
    }

    public void align(double offset) {
        //turns to a specific angle
        Orientation initial = bot.imu.getAngularOrientation();
        double error = initial.firstAngle;
        double diff = offset - error;
        turn(diff);
    }
    public void telemetryImu(){
        telemetry.addData("imu angle",bot.imu.getAngularOrientation().firstAngle);
    }

    public void telemetrymotorEncoders(){
        bot.motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bot.motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bot.motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bot.motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        telemetry.addData("motorLB encoder", bot.motorLB.getCurrentPosition());
        telemetry.addData("motorLF encoder", bot.motorLF.getCurrentPosition());
        telemetry.addData("motorRB encoder", bot.motorRB.getCurrentPosition());
        telemetry.addData("motorRF encoder", bot.motorRF.getCurrentPosition());
    }


    public void turntoGoldDepot(int order){
        if(order==1)
        {
            align(30);
        }
        else if (order==3)
        {
            align(-30);
        }
        else
        {
            align(0);
        }
    }
    public void turntoGoldCrater(int order){
        if(order==1)
        {
            align(30);
        }
        else if (order==3)
        {
            align(-30);
        }
        else
        {
            align(0);
        }
    }

    public void hitGold(){
        intakeOn();
        goAngle(30,90,.8);
        sleep(500);
        intakeOff();
    }

    public void scoreMarkerDepot(int order){
        goAngle(10,90,.8);
        align(45);
        if(order==1)
        {
            goAngle(5,0,.8);
            goAngle(8,-90,.8);
        }
        else if (order==3)
        {
            goAngle(20,0,.8);
        }
        else{
            goAngle(10,-90,.8);
            goAngle(10,0,.8);
        }
        bot.bucketServo.setPosition(1);
    }

    public void scoreMarkerCrater(int order){
        goAngle(10,-90,.8);
        align(0);
        goAngle(25,180,.8);
        align(45);
        goAngle(30,-90,.8);
        bot.bucketServo.setPosition(1);
        sleep(1000);
    }

    public void descend()
    {
        int intialTicks = bot.hangMotor.getCurrentPosition();
        int target = 10*(int)(9.5/(Math.PI* (1.5))*1680);
        //distance the hang needs to decend divided by the circumfrence multiplied by the pulses per rotation of a 60
        bot.hangMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.hangMotor.setTargetPosition(target+intialTicks);
        bot.hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bot.hangMotor.setPower(-1.0);

        runTime.reset();
        while(runTime.seconds()<10&&bot.hangMotor.isBusy()) {
            strafeLeft(.2);
            telemetry.addData("location",bot.hangMotor.getCurrentPosition());

            telemetry.addData("target", bot.hangMotor.getTargetPosition());
            telemetry.update();
        }
        bot.hangMotor.setPower(0);
    }
    public void descend2()
    {
        int intialTicks = bot.hangMotor.getCurrentPosition();
        int target = 2*-1*(int)(9.5/(Math.PI* (1.5))*1680);
        //distance the hang needs to decend divided by the circumfrence multiplied by the pulses per rotation of a 60
        bot.hangMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.hangMotor.setTargetPosition(target+intialTicks);
        bot.hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bot.hangMotor.setPower(1.0);

        runTime.reset();
        while(runTime.seconds()<10&&bot.hangMotor.isBusy()) {
            telemetry.addData("location",bot.hangMotor.getCurrentPosition());

            telemetry.addData("target", bot.hangMotor.getTargetPosition());
            telemetry.update();
        }
        bot.hangMotor.setPower(0);
    }
    public void driveAwayFromLander(){
        goAngle(1.5,-90,.3);
        goAngle(1.5,180,.3);
        align(0);

    }

    public void intakeOn(){
        bot.intakeMotor.setPower(-1);
    }

    public void intakeOff(){
        bot.intakeMotor.setPower(0);
    }

}