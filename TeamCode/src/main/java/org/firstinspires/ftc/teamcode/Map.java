package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaRoverRuckus;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by khadija on 11/30/2018.
 */
public class Map {
    public BNO055IMU imu;
    HardwareMap hwMap = null;
    public DcMotor motorLeftFront;
    public DcMotor motorRightFront;
    public DcMotor motorLeftBack;
    public DcMotor motorRightBack;
    public DcMotor hangMotor;
    public DcMotor intakeMotor;
    public DcMotor liftMotor;

    public Servo scoreServo;
    //public Servo door;
    public Servo marker;


    //public AnalogInput ultrasonic = null;
    //public AnalogInput potentioMeter;
    /*public Camera logitech = null;
    public Rev2mDistanceSensor prox = null;
    public NavxMicroNavigationSensor Navx = null;*/




    public int cameraMonitorViewId;

    //public WebcamName Logi;


    VuforiaLocalizer vuforia;

    ElapsedTime runtime = new ElapsedTime();


    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;
        imu = hwMap.get(BNO055IMU.class, "imu");
        motorLeftFront = hwMap.dcMotor.get("motorLeftFront");
        motorLeftBack = hwMap.dcMotor.get("motorLeftBack");
        motorRightBack = hwMap.dcMotor.get("motorRightBack");
        motorRightFront = hwMap.dcMotor.get("motorRightFront");
        liftMotor = hwMap.dcMotor.get("liftMotor");
        hangMotor = hwMap.dcMotor.get("hangMotor");
        intakeMotor = hwMap.dcMotor.get("intakeMotor");


        //door = hwMap.servo.get("door");
        scoreServo = hwMap.servo.get("scoreServo");
        marker = hwMap.servo.get("marker");

        VuforiaLocalizer.Parameters vuforiaParameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);

        //colorSensor = hwMap.get(ColorSensor.class, "colorSensor");
        //colorSensor2 = hwMap.get(ColorSensor.class, "colorSensor2");
        //ultrasonic = hwMap.get(AnalogInput.class, "ultrasonicLeft");

        //potentioMeter = hwMap.analogInput.get("potentioMeter");
        //talk to partner and change crater penetration angle with potentiometer

        cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        vuforiaParameters.cameraName.getCameraCharacteristics();
        motorLeftFront.setDirection(DcMotor.Direction.FORWARD);
        motorLeftBack.setDirection(DcMotor.Direction.FORWARD);
        motorRightFront.setDirection(DcMotor.Direction.FORWARD);
        motorRightBack.setDirection(DcMotor.Direction.FORWARD);
        hangMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);





        vuforiaParameters.vuforiaLicenseKey = "AfbM7ND/////AAAAGUXqRoQRDEkKupX0Zkdd3WhqVs68pW5fggxtJc7rlwOAI1WWfs5J4APPWl3FElqMVRdxwlDg3Rcx2DycCogRQGhyOZ6Gakktkgk22k/vy9q8OGLvDvGQQf6zOW3Qrs4hkn2qDWA4r5pDz3W8Aoh97+RCVTiVstECpe1mp97YGrYc5EeyW68aml6lirGr43motonPrXChztqG/3WpqYfFRFIsc+g+leI/ihWuAA1ZUFDYQjRV94GRl66w31kHcGtm+j2BKUlcQsVPmhizh+396O5r4yGkTcLBAZxyuyGm+lerwPJ9DWrkCiwVOtnCVqLUkfAoAjpuXuXEtW4JTlwqYmKVTuVDIg4Wcm7c8vLEBV/4";

        vuforiaParameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        vuforia = ClassFactory.getInstance().createVuforia(vuforiaParameters);

        // Load the data sets that for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsRoverRuckus);

        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hangMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }
}
