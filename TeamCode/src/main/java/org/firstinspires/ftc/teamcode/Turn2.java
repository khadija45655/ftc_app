package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Mode;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.RobotProcessor.RobotProcessor;

@Autonomous(name = "Turn2", group = "tensor")
public class Turn2 extends LinearOpMode {

    RobotProcessor proc = new RobotProcessor();

    /**
     * Override this method and place your code here.
     * <p>
     * Please do not swallow the InterruptedException, as it is used in cases
     * where the op mode needs to be terminated early.
     *
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {
        proc.bot.initBot(this,hardwareMap,Mode.Auto,telemetry);
        proc.initProc();
        waitForStart();

        proc.driveTrainProcessor.turn(90,.025,0.05,.02);


    }
}
