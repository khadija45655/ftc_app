
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

/**
 * created by andrew on 4/7/18.
 */

public abstract class Processor extends LinearOpMode {
    Map bot = new Map();
    ElapsedTime runtime = new ElapsedTime();
    public final static double TICKSPERROTATION = 537.6;
    static final double P_TURN_COEFF = .018;
    static final double I_TURN_COEFF = .01;
    static final double D_TURN_COEFF = .026;
    public final static int DIAMETEROFWHEEL = 4;
    static final double HEADING_THRESHOLD = 1.5;
    static final double ANTI_WINDUP = 2;

    static final double OMNI_WHEEL_CIRCUMFERENCE = 4 * Math.PI;

    static final double COUNTS_PER_MOTOR_REV = 1120;
    static final double DRIVE_GEAR_REDUCTION = 1.286;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415));

    static final int RAISE_POSTION = 1200;







    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
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
            telemetry.addData("motor power", bot.motorLF.getPower());
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
        bot.motorLF.setPower(clip_speed);
        bot.motorRF.setPower(clip_speed);
        bot.motorRB.setPower(clip_speed);
        bot.motorLB.setPower(clip_speed);
    }






    public void align(double offset) {
        //turns to a specific angle
        double error = angularOffset();
        double diff = offset - error;
        turn(diff);
    }

    public float angularOffset() {
        Orientation initial = bot.imu.getAngularOrientation();
        float x = initial.firstAngle;
        return x;
    }


    public void goAngle(double dist, double angle) {
        resetEnc();
        enterPosenc();
        double angel = Math.PI * angle / 180;
        double x = Math.cos(angel);
        double y = Math.sin(angel);
        double distance = dist / (OMNI_WHEEL_CIRCUMFERENCE);
        double ticks = TICKSPERROTATION * distance;
        int ticksRF = (int) Math.round(ticks * Math.signum(y - x));
        int ticksLF = (int) Math.round(ticks * Math.signum(-y - x));
        int ticksLB = (int) Math.round(ticks * Math.signum(-y + x));
        int ticksRB = (int) Math.round(ticks * Math.signum(y + x));
        bot.motorLF.setTargetPosition(ticksLF);
        bot.motorRF.setTargetPosition(ticksRF);
        bot.motorRB.setTargetPosition(ticksRB);
        bot.motorLB.setTargetPosition(ticksLB);
        bot.motorRF.setPower(.5 * (y - x));
        bot.motorLF.setPower(.5 * (-y - x));
        bot.motorLB.setPower(.5 * (-y + x));
        bot.motorRB.setPower(.5 * (y + x));
        while (
                (bot.motorLB.isBusy() && bot.motorRB.isBusy() && bot.motorRF.isBusy() && bot.motorLF.isBusy() && opModeIsActive())) {

            // Display it for the driver.

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
            telemetry.update();
        }

        stopBotMotors();

        sleep(250);
        enterEnc();
    }

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

    public void goAnglePower(double dist, double angle, double power) {
        resetEnc();
        enterPosenc();
        double angel = Math.PI * angle / 180;
        double x = Math.cos(angel);
        double y = Math.sin(angel);
        double distance = dist / (OMNI_WHEEL_CIRCUMFERENCE);
        double ticks = TICKSPERROTATION * distance;
        int ticksRF = (int) Math.round(ticks * Math.signum(y - x));
        int ticksLF = (int) Math.round(ticks * Math.signum(-y - x));
        int ticksLB = (int) Math.round(ticks * Math.signum(-y + x));
        int ticksRB = (int) Math.round(ticks * Math.signum(y + x));
        bot.motorLF.setTargetPosition(ticksLF);
        bot.motorRF.setTargetPosition(ticksRF);
        bot.motorRB.setTargetPosition(ticksRB);
        bot.motorLB.setTargetPosition(ticksLB);
        bot.motorRF.setPower(power * (y - x));
        bot.motorLF.setPower(power * (-y - x));
        bot.motorLB.setPower(power * (-y + x));
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
            telemetry.update();
        }

        stopBotMotors();

        sleep(250);
        enterEnc();
    }





}