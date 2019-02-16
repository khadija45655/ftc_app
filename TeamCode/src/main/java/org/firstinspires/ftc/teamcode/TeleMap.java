package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.hardware.*;
import com.qualcomm.hardware.*;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by wolfie on 9/1/17.
 */

public class TeleMap {
    HardwareMap hwMap = null;
    public DcMotor motorLF;
    public DcMotor motorLB;
    public DcMotor motorRF;
    public DcMotor motorRB;
    public DcMotor hangMotor;
    public DcMotor intakeMotor;
    public DcMotor bucketMotor;

    public Servo bucketServo1;
    public Servo bucketServo2;

    public Servo marker;

    //public AnalogInput limit1;
    //public AnalogInput limit2;

    public void init(HardwareMap ahwMap) {

        // save reference to HW Map
        hwMap = ahwMap;
        motorLB = hwMap.dcMotor.get("motorLB");
        motorLF = hwMap.dcMotor.get("motorLF");
        motorRF = hwMap.dcMotor.get("motorRF");
        motorRB = hwMap.dcMotor.get("motorRB");

        bucketMotor = hwMap.dcMotor.get("bucketMotor");
        hangMotor = hwMap.dcMotor.get("hangMotor");
        intakeMotor = hwMap.dcMotor.get("intakeMotor");

        bucketServo1 = hwMap.servo.get("bucketServo1");
        bucketServo2 = hwMap.servo.get("bucketServo2");

        marker = hwMap.servo.get("marker");

        //limit1 = hwMap.analogInput.get("limit1");

        //limit2 = hwMap.analogInput.get("limit2");


        motorLF.setDirection(DcMotor.Direction.FORWARD);
        motorRB.setDirection(DcMotor.Direction.FORWARD);
        motorRF.setDirection(DcMotor.Direction.FORWARD);
        motorLB.setDirection(DcMotor.Direction.FORWARD);

        bucketMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        hangMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bucketMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bucketMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bucketMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
}
