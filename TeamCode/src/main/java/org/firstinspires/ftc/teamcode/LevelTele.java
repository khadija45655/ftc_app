package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "level")
public class LevelTele extends OpMode {
    Map bot = new Map();
    @Override
    public void init() {
        bot.init(hardwareMap);
    }

    @Override
    public void loop() {
        if(gamepad2.right_bumper){
            bot.leverArmRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bot.leverArmRight.setTargetPosition(0);
            bot.leverArmRight.setPower(1);
            bot.leverArmLeft.setTargetPosition(0);
            bot.leverArmLeft.setPower(1);
        }
        if(bot.leverArmRight.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION)){
            bot.leverArmRight.setTargetPosition(0);
            bot.leverArmLeft.setTargetPosition(0);

        }
    }
}
