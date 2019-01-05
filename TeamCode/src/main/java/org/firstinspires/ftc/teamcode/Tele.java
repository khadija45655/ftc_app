package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Tele",group = "A")
public class Tele extends OpMode{
    TeleMap bot = new TeleMap();
    double xpow;
    double ypow;
    double zpow;
    int count = 0;
    @Override
    public void init() {
        //initalizes hardware map
        bot.init(hardwareMap);
    }

    public void readGamePad() {
        //assigns joystick values to variables
        zpow = gamepad1.right_stick_x;
        ypow = gamepad1.left_stick_y;
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
    @Override
    public void loop() {

        //takes the joystick values and converts to motor speeds through holonomic calculations
        readGamePad();

        double theta = Math.atan2(ypow, xpow); //angle of joystick
        double power = Math.pow(Math.max(Math.abs(xpow),Math.abs(ypow)),2); //logarithmic drive
        // offset of pi/4 makes wheels strafe correctly at cardinal and intermediate directions
        double cos = Math.cos(theta - Math.PI / 4);
        double sin = Math.sin(theta - Math.PI / 4);
        //eliminates incorrect signs resulting from double precision
        if(Math.abs(cos)<.0000001){
            cos=0;
        }
        if(Math.abs(sin)<.0000001){
            sin=0;
        }
        double x = Math.signum(cos);
        double y = Math.signum(sin);

        bot.motorLF.setPower(power * -x + zpow);
        bot.motorRF.setPower(power * y + zpow);
        bot.motorRB.setPower(power * x + zpow);
        bot.motorLB.setPower(power * -y + zpow);


        if(gamepad2.a)
        {
            bot.intakeMotor.setPower(1);
        }

        if(gamepad2.b)
        {
            bot.intakeMotor.setPower(0);
        }
        if(gamepad2.y) {
            bot.intakeMotor.setPower(-1);
        }

        bot.bucketMotor.setPower(-gamepad2.right_stick_y);
        bot.hangMotor.setPower(-gamepad2.left_stick_y);

        if(gamepad2.dpad_up) {
            bot.bucketServo.setPosition(1.0);
        }
        if(gamepad2.dpad_down)
        {
            bot.bucketServo.setPosition(0.0);
        }

        if(bot.limit1.getVoltage()>.2||bot.limit2.getVoltage()>.2) {
            count++;
            if (count>2){
                bot.intakeMotor.setPower(-1);
                count = 0;
            }
        }
    }
}
