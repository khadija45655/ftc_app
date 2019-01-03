package org.firstinspires.ftc.teamcode.RobotProcessor;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.*;

public class DriveTrainProcessor {

    private RobotProcessor Proc;

    static final double P_TURN_COEFF = .018;
    static final double I_TURN_COEFF = 0;
    static final double D_TURN_COEFF = 0;
    static final double HEADING_THRESHOLD = 5;
    static final double ANTI_WINDUP = 2;

    public final static double TICKSPERROTATION = 537.6;
    public static final double OMNI_WHEEL_CIRCUMFERENCE = 4 * Math.PI;
    public final static int DIAMETER_OF_WHEEL = 4;
    public static final double DRIVE_GEAR_REDUCTION = 1.286;


    public void turn(double target) {
        //Turn using PID
        // clockwise = negative input, counter-clockwise = positive input

        double heading = Proc.bot.sensors.getHeading();
        double angleWanted = target + heading;
        double rcw =1;
        double speed = .2;
        double integral = 0;
        double previous_error = 0;
        while (rcw != 0 && Proc.bot.opModeIsActive()) {


            double error = Math.signum(angleWanted - Proc.bot.sensors.getHeading());

            if (Math.abs(error) < HEADING_THRESHOLD) {
                error = 0;
            }

            accelerate(speed*error);
        }
        accelerate(0);
    }

    private void accelerate(double speed) {
        double clip_speed = Range.clip(speed, -1, 1);
        Proc.bot.driveTrain.motorLF.setPower(clip_speed);
        Proc.bot.driveTrain.motorRF.setPower(clip_speed);
        Proc.bot.driveTrain.motorRB.setPower(clip_speed);
        Proc.bot.driveTrain.motorLB.setPower(clip_speed);
    }

}
