package org.firstinspires.ftc.teamcode.RobotProcessor;

import org.firstinspires.ftc.teamcode.Robot.Output;
import com.qualcomm.robotcore.util.ElapsedTime;

public class OutputProcessor {
    Output output;
    ElapsedTime runtime = new ElapsedTime();


    public OutputProcessor(Output output){
        this.output = output;
    }

    public void bucketUp() {
        runtime.reset();
        while (runtime.milliseconds()< 1250)
        {
            output.bucketMotor.setPower(1);
        }
    }
    public void bucketDown() {
        runtime.reset();
        while (runtime.milliseconds()< 750)
        {
            output.bucketMotor.setPower(-1);
        }
        output.bucketMotor.setPower(0);
    }
    public void openDoor(){
        output.bucketServo1.setPosition(1.0);
        output.bucketServo2.setPosition(1.0);
        runtime.reset();
        while(runtime.milliseconds()<1000)
        {

        }
    }
    public void closeDoor()
    {
        output.bucketServo2.setPosition(0);
        output.bucketServo1.setPosition(0);
        runtime.reset();
        while(runtime.milliseconds()<500)
        {

        }
    }
}
