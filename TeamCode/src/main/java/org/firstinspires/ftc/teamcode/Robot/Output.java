package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.*;

public class Output {
    public DcMotor bucketMotor;
    public Servo bucketServo1;
    public Servo bucketServo2;

    public Servo marker;

    public Output(HardwareMap hwMap, Mode mode){
        bucketMotor = hwMap.dcMotor.get("bucketMotor");
        bucketServo1 = hwMap.servo.get("bucketServo1");
        bucketServo2 = hwMap.servo.get("bucketServo2");

        marker = hwMap.servo.get("marker");

        bucketMotor.setDirection(DcMotor.Direction.FORWARD);
        bucketMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setMotorRunMode(mode);


    }

    public void setMotorRunMode(Mode mode){
        if(mode == Mode.Auto){
            bucketMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bucketMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
        else{
            bucketMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bucketMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }
    }
}
