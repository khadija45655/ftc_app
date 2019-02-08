package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "TeleTest",group = "A")
public class TeleTest extends OpMode {
    TeleMap bot = new TeleMap();
    double xpow;
    double ypow;
    double zpow;
    int count = 0;
    boolean up;
    boolean down;
    double bucketPower = 0;
    public static final int BUCKET_UP_POSITION = 1250;
    public static final double RESTING_UP_POWER = -.2;
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        //initalizes hardware map
        bot.init(hardwareMap);
    }

    public void readGamePad() {
        //assigns joystick values to variables
        zpow = gamepad1.right_stick_x;
        ypow = -gamepad1.left_stick_y;
        xpow = gamepad1.left_stick_x;

        //creates a deadzone for left stick y
        if (Math.abs(ypow) < .1) {
            ypow = 0;

        }
        //creates a deadzone for left stick x
        if (Math.abs(xpow) < .1) {
            xpow = 0;

        }
    }



    public void flipGamePad() {

    }

    @Override
    public void loop() {
        //takes the joystick values and converts to motor speeds through holonomic calculations
        readGamePad();


        double theta = Math.atan2(ypow, xpow); //angle of joystick
        double power = Math.pow(Math.max(Math.abs(xpow), Math.abs(ypow)), 2); //logarithmic drive
        double zpower = Math.pow(Math.abs(zpow), 2);
        // offset of pi/4 makes wheels strafe correctly at cardinal and intermediate directions
        double x = Math.cos(theta);
        double y = Math.sin(theta);

        double z = Math.signum(zpow);

        bot.motorLF.setPower(power * (-y - x) - zpower * z);
        bot.motorRF.setPower(power * (y - x) - zpower * z);
        bot.motorRB.setPower(power * (y + x) - zpower * z);
        bot.motorLB.setPower(power * (-y + x) - zpower * z);

        telemetry.addData("xpow", xpow);
        telemetry.addData("ypow", ypow);
        telemetry.addData("zpow", zpow);
        telemetry.addData("power", power);
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("motorLF", bot.motorLF.getPower());
        telemetry.addData("motorRF", bot.motorRF.getPower());
        telemetry.addData("motorLB", bot.motorLB.getPower());
        telemetry.addData("motorRB", bot.motorRB.getPower());
        telemetry.addData("bucketMotor", bot.bucketMotor.getPower());
        telemetry.addData("bucketMotor",bot.bucketMotor.getCurrentPosition());
        telemetry.addData("count", count);
        telemetry.addData("Path2", "Running at motorLB %7d motorLF :%7d motorRB %7d motorRF %7d",
                bot.motorLB.getCurrentPosition(),
                bot.motorLF.getCurrentPosition(),
                bot.motorRB.getCurrentPosition(),
                bot.motorRF.getCurrentPosition());
        //telemetry.addData("limit1",bot.limit1.getVoltage());
        //telemetry.addData("limit2",bot.limit2.getVoltage());
        telemetry.update();




        if(gamepad1.a){

            up = true;
            down = false;
        }
        if(gamepad2.b){

            up = false;
            down = true;

        }
        if(gamepad2.y){

            up = false;
            down = false;
        }
        if(up) {
            bucketPower = -1;
        }
        if(down){
            bucketPower = .5;
        }
        if(!up&&bot.bucketMotor.getCurrentPosition()<0){
            bucketPower = 0;
            down = false;


        }
        if(!down&&bot.bucketMotor.getCurrentPosition()>BUCKET_UP_POSITION){
            bucketPower = RESTING_UP_POWER;
            up = false;

        }
        bot.bucketMotor.setPower(-bucketPower);

        if(gamepad1.right_trigger>0){
            bot.hangMotor.setPower(-gamepad1.right_trigger);
        }
        else if(gamepad1.left_trigger>0){
            bot.hangMotor.setPower(gamepad1.left_trigger);
        }
        else{
            bot.hangMotor.setPower(0);
        }
        if(gamepad1.dpad_up){
            bot.intakeMotor.setPower(1);
        }
        if(gamepad1.dpad_down){
            bot.intakeMotor.setPower(-1);
        }
        if(gamepad1.dpad_left){
            bot.intakeMotor.setPower(0);

        }

    }
}