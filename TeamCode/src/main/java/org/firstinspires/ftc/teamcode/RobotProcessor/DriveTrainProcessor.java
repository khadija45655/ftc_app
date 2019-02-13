package org.firstinspires.ftc.teamcode.RobotProcessor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.*;
import org.firstinspires.ftc.teamcode.Robot.DriveTrain;
import org.firstinspires.ftc.teamcode.Robot.OpModeIF;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Sensors;

public class DriveTrainProcessor {

    OpModeIF opMode;
    DriveTrain driveTrain;
    Sensors sensors;
    Telemetry telemetry;
    LinearOpMode currentOpmode;

     public DriveTrainProcessor(OpModeIF opMode, DriveTrain driveTrain, Sensors sensors, Telemetry telemetry, LinearOpMode currentOpmode){
         this.opMode = opMode;
         this.driveTrain = driveTrain;
         this.sensors = sensors;
         this.telemetry = telemetry;
         this.currentOpmode = currentOpmode;
     }


    static final double P_TURN_COEFF = .025;
    static final double I_TURN_COEFF = 0.04;
    static final double D_TURN_COEFF = .03;
    static final double HEADING_THRESHOLD = 2;
    static final double ANTI_WINDUP = 4;

    public final static double TICKSPERROTATION = 1120;
    public final static double STRAFECORRECTION = 1.5;
    public static final double OMNI_WHEEL_CIRCUMFERENCE = 3.93 * Math.PI;
    public final static double DIAMETER_OF_WHEEL = 3.93;
    public static final double DRIVE_GEAR_REDUCTION = 1.778;

    public void resetEnc() {
         driveTrain.motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         driveTrain.motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         driveTrain.motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         driveTrain.motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void enterEnc() {
         driveTrain.motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         driveTrain.motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         driveTrain.motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         driveTrain.motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void enterPosenc() {
         driveTrain.motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         driveTrain.motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         driveTrain.motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         driveTrain.motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void stopBotMotors() {
         driveTrain.motorRF.setPower(0);
         driveTrain.motorLF.setPower(0);
         driveTrain.motorLB.setPower(0);
         driveTrain.motorRB.setPower(0);

    }


    public void turn(double target) {
        //Turn using PID
        // clockwise = negative input, counter-clockwise = positive input

        double heading = sensors.getHeading();
        double angleWanted = target + heading;
        double rcw = 1;
        double integral = 0;
        double previous_error = 0;
        while (rcw != 0 && currentOpmode.opModeIsActive()) {



            double error = angleWanted - sensors.getHeading();;

            while (error > 180 && currentOpmode.opModeIsActive())
                error -= 360;
            while (error < -180 && currentOpmode.opModeIsActive())
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
            rcw = P_TURN_COEFF * error + I_TURN_COEFF * integral + D_TURN_COEFF * derivative;
            previous_error = error;
            accelerate(rcw);

            telemetry.addData("first angle", sensors.getHeading());
            //telemetry.addData("second angle", ref.secondAngle);
            //telemetry.addData("third angle", ref.thirdAngle);
            telemetry.addData("target", target);
            telemetry.addData("speed ", rcw);
            telemetry.addData("error", angleWanted - sensors.getHeading());
            telemetry.addData("angleWanted", angleWanted);
            telemetry.addData("motor power", driveTrain.motorLF.getPower());
            telemetry.addData("rcw", rcw);
            telemetry.addData("P", P_TURN_COEFF * error);
            telemetry.addData("I", I_TURN_COEFF * integral);
            telemetry.addData("D", D_TURN_COEFF * derivative);
            telemetry.update();

            currentOpmode.sleep(20);


        }
        accelerate(0);
    }

    public void turn(double target, double p, double i, double d) {
        //Turn using PID
        // clockwise = negative input, counter-clockwise = positive input

        double heading = sensors.getHeading();
        double angleWanted = target + heading;
        double rcw = 1;
        double integral = 0;
        double previous_error = 0;
        while (rcw != 0 && currentOpmode.opModeIsActive()) {



            double error = angleWanted - sensors.getHeading();;

            while (error > 180 && currentOpmode.opModeIsActive())
                error -= 360;
            while (error < -180 && currentOpmode.opModeIsActive())
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
            if (integral > (50 / i)) {
                integral = 50 / i;
            }
            if (error == 0) {
                derivative = 0;
            }
            rcw = p * error + i * integral + d * derivative;
            previous_error = error;
            accelerate(rcw);

            telemetry.addData("first angle", sensors.getHeading());
            //telemetry.addData("second angle", ref.secondAngle);
            //telemetry.addData("third angle", ref.thirdAngle);
            telemetry.addData("target", target);
            telemetry.addData("speed ", rcw);
            telemetry.addData("error", angleWanted - sensors.getHeading());
            telemetry.addData("angleWanted", angleWanted);
            telemetry.addData("motor power", driveTrain.motorLF.getPower());
            telemetry.addData("rcw", rcw);
            telemetry.addData("P", p * error);
            telemetry.addData("I", i * integral);
            telemetry.addData("D", d * derivative);
            telemetry.addData("P_coe", p );
            telemetry.addData("I_coe", i );
            telemetry.addData("D_coe", d );
            telemetry.update();

            currentOpmode.sleep(20);


        }
        accelerate(0);
    }
    public void accelerate(double speed) {
        double clip_speed = Range.clip(speed, -1, 1);
         driveTrain.motorLF.setPower(clip_speed);
         driveTrain.motorRF.setPower(clip_speed);
         driveTrain.motorRB.setPower(clip_speed);
         driveTrain.motorLB.setPower(clip_speed);
    }

    public void goAngle(double dist, double angle, double power) {
        resetEnc();
        enterPosenc();
        angle = angle+90;
        double angel = Math.toRadians(angle);
        double x = Math.cos(angel);
        double y = Math.sin(angel);
        double distanceRatio = dist / (OMNI_WHEEL_CIRCUMFERENCE);
        double ticks = (TICKSPERROTATION*distanceRatio)/DRIVE_GEAR_REDUCTION;
        int ticksRF = (int) Math.round(ticks * (y - x));
        int ticksLF = (int) Math.round(ticks * (-y - x));
        int ticksLB = (int) Math.round(ticks * (-y + x));
        int ticksRB = (int) Math.round(ticks * (y + x));
         driveTrain.motorLF.setTargetPosition(ticksLF);
         driveTrain.motorRF.setTargetPosition(ticksRF);
         driveTrain.motorRB.setTargetPosition(ticksRB);
         driveTrain.motorLB.setTargetPosition(ticksLB);
         driveTrain.motorRF.setPower(power * (y - x));
         driveTrain.motorLF.setPower(-power * (-y - x));
         driveTrain.motorLB.setPower(-power * (-y + x));
         driveTrain.motorRB.setPower(power * (y + x));
        while (( driveTrain.motorLB.isBusy() &&
                 driveTrain.motorRB.isBusy() &&
                 driveTrain.motorRF.isBusy() &&
                 driveTrain.motorLF.isBusy()) &&
                 opMode.opModeIsActive()) {
             telemetry.addData("Path2", "Running at motorLB %7d motorLF :%7d motorRB %7d motorRF %7d",
                     driveTrain.motorLB.getCurrentPosition(),
                     driveTrain.motorLF.getCurrentPosition(),
                     driveTrain.motorRB.getCurrentPosition(),
                     driveTrain.motorRF.getCurrentPosition());
             telemetry.addData("target", "Running at motorLB %7d motorLF :%7d motorRB %7d motorRF %7d",
                     driveTrain.motorLB.getTargetPosition(),
                     driveTrain.motorLF.getTargetPosition(),
                     driveTrain.motorRB.getTargetPosition(),
                     driveTrain.motorRF.getTargetPosition());
             telemetry.addData("gyroHeading", sensors.imu.getAngularOrientation());
             telemetry.update();
        }
        stopBotMotors();
        telemetry.addData("path reached", sensors.imu.getAngularOrientation());
        telemetry.update();
        currentOpmode.sleep(50);
        enterEnc();
    }
    public void goAngleStall(double dist, double angle, double power) {
        resetEnc();
        enterPosenc();
        angle = angle+90;
        double angel = Math.toRadians(angle);
        double x = Math.cos(angel);
        double y = Math.sin(angel);
        double stall = -1;
        int count = 0;
        double distanceRatio = dist / (OMNI_WHEEL_CIRCUMFERENCE);
        double ticks = (TICKSPERROTATION*distanceRatio)/DRIVE_GEAR_REDUCTION;
        int ticksRF = (int) Math.round(ticks * (y - x));
        int ticksLF = (int) Math.round(ticks * (-y - x));
        int ticksLB = (int) Math.round(ticks * (-y + x));
        int ticksRB = (int) Math.round(ticks * (y + x));
        driveTrain.motorLF.setTargetPosition(ticksLF);
        driveTrain.motorRF.setTargetPosition(ticksRF);
        driveTrain.motorRB.setTargetPosition(ticksRB);
        driveTrain.motorLB.setTargetPosition(ticksLB);
        driveTrain.motorRF.setPower(power * (y - x));
        driveTrain.motorLF.setPower(-power * (-y - x));
        driveTrain.motorLB.setPower(-power * (-y + x));
        driveTrain.motorRB.setPower(power * (y + x));
        while (( driveTrain.motorLB.isBusy() &&
                driveTrain.motorRB.isBusy() &&
                driveTrain.motorRF.isBusy() &&
                driveTrain.motorLF.isBusy()) &&
                opMode.opModeIsActive()) {
            stall = driveTrain.motorRF.getCurrentPosition();
            telemetry.addData("Path2", "Running at motorLB %7d motorLF :%7d motorRB %7d motorRF %7d",
                    driveTrain.motorLB.getCurrentPosition(),
                    driveTrain.motorLF.getCurrentPosition(),
                    driveTrain.motorRB.getCurrentPosition(),
                    driveTrain.motorRF.getCurrentPosition());
            telemetry.addData("target", "Running at motorLB %7d motorLF :%7d motorRB %7d motorRF %7d",
                    driveTrain.motorLB.getTargetPosition(),
                    driveTrain.motorLF.getTargetPosition(),
                    driveTrain.motorRB.getTargetPosition(),
                    driveTrain.motorRF.getTargetPosition());
            telemetry.addData("gyroHeading", sensors.imu.getAngularOrientation());
            telemetry.update();
            if(stall == driveTrain.motorRF.getCurrentPosition()){
                count++;
            }
            if(count>10){
                break;
            }

            stall = driveTrain.motorRF.getCurrentPosition();
        }
        stopBotMotors();
        telemetry.addData("path reached", sensors.imu.getAngularOrientation());
        telemetry.update();
        currentOpmode.sleep(20);
        enterEnc();
    }

    public void align(double offset) {
        //turns to a specific angle

        double error =  sensors.getHeading();
        double diff = offset - error + 90;
        turn(diff);
    }
    public void driveForward(double power){
        driveTrain.motorRF.setPower(power);
        driveTrain.motorLF.setPower(-power);
        driveTrain.motorLB.setPower(-power);
        driveTrain.motorRB.setPower(power);
    }
    public void driveBackward(double power){
        driveTrain.motorRF.setPower(-power);
        driveTrain.motorLF.setPower(power);
        driveTrain.motorLB.setPower(power);
        driveTrain.motorRB.setPower(-power);
    }
    public void strafeLeft(double power){
        driveTrain.motorRF.setPower(-power);
        driveTrain.motorLF.setPower(-power);
        driveTrain.motorLB.setPower(power);
        driveTrain.motorRB.setPower(power);
    }
    public void strafeRight(double power){
        driveTrain.motorRF.setPower(-power);
        driveTrain.motorLF.setPower(power);
        driveTrain.motorLB.setPower(power);
        driveTrain.motorRB.setPower(-power);
    }
    public void strafeLeftInches(double power, double inches){
        resetEnc();
        enterPosenc();

        double angel = Math.toRadians(180);
        double x = Math.cos(angel);
        double y = Math.sin(angel);
        double distanceRatio = inches / (OMNI_WHEEL_CIRCUMFERENCE);
        double ticks = ((TICKSPERROTATION*distanceRatio)/DRIVE_GEAR_REDUCTION)*STRAFECORRECTION;
        int ticksRF = (int) Math.round(ticks * (y - x));
        int ticksLF = (int) Math.round(ticks * (-y - x));
        int ticksLB = (int) Math.round(ticks * (-y + x));
        int ticksRB = (int) Math.round(ticks * (y + x));
        driveTrain.motorLF.setTargetPosition(ticksLF);
        driveTrain.motorRF.setTargetPosition(ticksRF);
        driveTrain.motorRB.setTargetPosition(ticksRB);
        driveTrain.motorLB.setTargetPosition(ticksLB);
        driveTrain.motorRF.setPower(power * (y - x));
        driveTrain.motorLF.setPower(-power * (-y - x));
        driveTrain.motorLB.setPower(-power * (-y + x));
        driveTrain.motorRB.setPower(power * (y + x));
        while (( driveTrain.motorLB.isBusy() &&
                driveTrain.motorRB.isBusy() &&
                driveTrain.motorRF.isBusy() &&
                driveTrain.motorLF.isBusy()) &&
                opMode.opModeIsActive()) {
            telemetry.addData("Path2", "Running at motorLB %7d motorLF :%7d motorRB %7d motorRF %7d",
                    driveTrain.motorLB.getCurrentPosition(),
                    driveTrain.motorLF.getCurrentPosition(),
                    driveTrain.motorRB.getCurrentPosition(),
                    driveTrain.motorRF.getCurrentPosition());
            telemetry.addData("target", "Running at motorLB %7d motorLF :%7d motorRB %7d motorRF %7d",
                    driveTrain.motorLB.getTargetPosition(),
                    driveTrain.motorLF.getTargetPosition(),
                    driveTrain.motorRB.getTargetPosition(),
                    driveTrain.motorRF.getTargetPosition());
            telemetry.addData("gyroHeading", sensors.imu.getAngularOrientation());
            telemetry.update();
        }
        stopBotMotors();
        telemetry.addData("path reached", sensors.imu.getAngularOrientation());
        telemetry.update();
        currentOpmode.sleep(20);
        enterEnc();
    }
    public void strafeRightInches(double power, double inches){
        resetEnc();
        enterPosenc();

        double angel = Math.toRadians(0);
        double x = Math.cos(angel);
        double y = Math.sin(angel);
        double distanceRatio = inches / (OMNI_WHEEL_CIRCUMFERENCE);
        double ticks = ((TICKSPERROTATION*distanceRatio)/DRIVE_GEAR_REDUCTION)*STRAFECORRECTION;
        int ticksRF = (int) Math.round(ticks * (y - x));
        int ticksLF = (int) Math.round(ticks * (-y - x));
        int ticksLB = (int) Math.round(ticks * (-y + x));
        int ticksRB = (int) Math.round(ticks * (y + x));
        driveTrain.motorLF.setTargetPosition(ticksLF);
        driveTrain.motorRF.setTargetPosition(ticksRF);
        driveTrain.motorRB.setTargetPosition(ticksRB);
        driveTrain.motorLB.setTargetPosition(ticksLB);
        driveTrain.motorRF.setPower(power * (y - x));
        driveTrain.motorLF.setPower(-power * (-y - x));
        driveTrain.motorLB.setPower(-power * (-y + x));
        driveTrain.motorRB.setPower(power * (y + x));
        while (( driveTrain.motorLB.isBusy() &&
                driveTrain.motorRB.isBusy() &&
                driveTrain.motorRF.isBusy() &&
                driveTrain.motorLF.isBusy()) &&
                opMode.opModeIsActive()) {
            telemetry.addData("Path2", "Running at motorLB %7d motorLF :%7d motorRB %7d motorRF %7d",
                    driveTrain.motorLB.getCurrentPosition(),
                    driveTrain.motorLF.getCurrentPosition(),
                    driveTrain.motorRB.getCurrentPosition(),
                    driveTrain.motorRF.getCurrentPosition());
            telemetry.addData("target", "Running at motorLB %7d motorLF :%7d motorRB %7d motorRF %7d",
                    driveTrain.motorLB.getTargetPosition(),
                    driveTrain.motorLF.getTargetPosition(),
                    driveTrain.motorRB.getTargetPosition(),
                    driveTrain.motorRF.getTargetPosition());
            telemetry.addData("gyroHeading", sensors.imu.getAngularOrientation());
            telemetry.update();
        }
        stopBotMotors();
        telemetry.addData("path reached", sensors.imu.getAngularOrientation());
        telemetry.update();
        currentOpmode.sleep(20);
        enterEnc();
    }
}
