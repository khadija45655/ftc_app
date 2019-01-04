package org.firstinspires.ftc.teamcode.RobotProcessor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Robot.*;


public class RobotProcessor {
    Robot bot;
    DriveTrainProcessor driveTrainProcessor;
    HangProcessor hangProcessor;
    IntakeProcessor intakeProcessor;
    OutputProcessor outputProcessor;
    SensorProcessor sensorProcessor;
    ElapsedTime runTime;

    public RobotProcessor(){
        this.bot = new Robot();
        this.driveTrainProcessor = new DriveTrainProcessor(bot,bot.driveTrain,bot.sensors,bot.telemetry,bot.currentOpMode);
        this.hangProcessor = new HangProcessor(this.bot.hang);
        this.intakeProcessor = new IntakeProcessor(this.bot.intake);
        this.outputProcessor = new OutputProcessor(this.bot.output);
        this.sensorProcessor = new SensorProcessor(this.bot.sensors);
        runTime = new ElapsedTime();
    }



    public void descend()
    {
        int intialTicks = hangProcessor.hang.hangMotor.getCurrentPosition();
        int target = 10*(int)(9.5/(Math.PI* (1.5))*1680);
        //distance the hang needs to decend divided by the circumfrence multiplied by the pulses per rotation of a 60
        hangProcessor.hang.hangMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hangProcessor.hang.hangMotor.setTargetPosition(target+intialTicks);
        hangProcessor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hangProcessor.setPower(-1.0);

        runTime.reset();
        while(runTime.seconds()<10&&hangProcessor.hang.hangMotor.isBusy()) {
            driveTrainProcessor.strafeLeft(.2);
            bot.telemetry.addData("location",hangProcessor.hang.hangMotor.getCurrentPosition());

            bot.telemetry.addData("target", hangProcessor.hang.hangMotor.getTargetPosition());
            bot.telemetry.update();
        }
        hangProcessor.hang.hangMotor.setPower(0);
    }

}
