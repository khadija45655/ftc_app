package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Tele",group = "A")
public class Tele extends OpMode{
    TeleMap bot = new TeleMap();
    double xpow;
    double ypow;
    double zpow;
    int count = 0;
    boolean flipped;
    boolean bird;
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
    public void birdGamePad(){
        //assigns joystick values to variables
        zpow = gamepad1.right_stick_x;
        ypow = -gamepad1.left_stick_x;
        xpow = -gamepad1.left_stick_y;

        //creates a deadzone for left stick y
        if (Math.abs(ypow) < .1) {
            ypow = 0;

        }
        //creates a deadzone for left stick x
        if (Math.abs(xpow) < .1) {
            xpow = 0;

        }
    }
    public void flipGamePad(){
        //assigns joystick values to variables
        zpow = gamepad1.right_stick_x;
        ypow = gamepad1.left_stick_y;
        xpow = -gamepad1.left_stick_x;

        //creates a deadzone for left stick y
        if (Math.abs(ypow) < .1) {
            ypow = 0;

        }
        //creates a deadzone for left stick x
        if (Math.abs(xpow) < .1) {
            xpow = 0;

        }
    }
    @Override
    public void loop() {
        //takes the joystick values and converts to motor speeds through holonomic calculations
        readGamePad();
        if(gamepad1.right_bumper){
            flipped = true;
        }
        if(gamepad1.left_bumper){


            flipped = false;
        }
        if(flipped){
            flipGamePad();
        }

        double theta = Math.atan2(ypow, xpow); //angle of joystick
        double power = Math.pow(Math.max(Math.abs(xpow),Math.abs(ypow)),2); //logarithmic drive
        double zpower = Math.pow(Math.abs(zpow),2);
        // offset of pi/4 makes wheels strafe correctly at cardinal and intermediate directions
        double x = Math.cos(theta);
        double y= Math.sin(theta);

        double z = Math.signum(zpow);

        bot.motorLF.setPower(power * (-y-x) - zpower*z);
        bot.motorRF.setPower(power * (y-x) - zpower*z);
        bot.motorRB.setPower(power * (y+x) - zpower*z);
        bot.motorLB.setPower(power * (-y+x) - zpower*z);

        telemetry.addData("xpow",xpow);
        telemetry.addData("ypow",ypow);
        telemetry.addData("zpow",zpow);
        telemetry.addData("power",power);
        telemetry.addData("x",x);
        telemetry.addData("y",y);
        telemetry.addData("motorLF",bot.motorLF.getPower());
        telemetry.addData("motorRF",bot.motorRF.getPower());
        telemetry.addData("motorLB",bot.motorLB.getPower());
        telemetry.addData("motorRB",bot.motorRB.getPower());
        telemetry.addData("hangMotor",bot.hangMotor.getPower());
        telemetry.addData("count",count);
        telemetry.addData("Path2", "Running at motorLB %7d motorLF :%7d motorRB %7d motorRF %7d",
                bot.motorLB.getCurrentPosition(),
                bot.motorLF.getCurrentPosition(),
                bot.motorRB.getCurrentPosition(),
                bot.motorRF.getCurrentPosition());
        //telemetry.addData("limit1",bot.limit1.getVoltage());
        //telemetry.addData("limit2",bot.limit2.getVoltage());
        telemetry.update();

        if(gamepad2.a)
        {
            bot.intakeMotor.setPower(-1);
        }

        if(gamepad2.b)
        {
            bot.intakeMotor.setPower(0);
        }
        if(gamepad2.y) {
            bot.intakeMotor.setPower(1);
        }

        bot.bucketMotor.setPower(gamepad2.right_stick_y);

        //triggers return -1.0 when up and 1.0 when down
        if(gamepad1.right_trigger>0){
            bot.hangMotor.setPower(-gamepad1.right_trigger);
        }
        else if(gamepad1.left_trigger>0){
            bot.hangMotor.setPower(gamepad1.left_trigger);
        }
        else{
            bot.hangMotor.setPower(0);
        }

        if(gamepad2.dpad_up){
            bot.bucketServo.setPosition(1);
        }
        if(gamepad2.dpad_down){
            bot.bucketServo.setPosition(0);
        }
        /*
        if(bot.limit1.getVoltage()>2||bot.limit2.getVoltage()>2&&runtime.milliseconds()>500){
            count++;
            runtime.reset();
        }
        if(count > 1){
            bot.intakeMotor.setPower(1);
            count=0;
        }
        */
    }
}