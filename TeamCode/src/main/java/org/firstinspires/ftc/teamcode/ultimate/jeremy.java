package org.firstinspires.ftc.teamcode.ultimate;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.revextensions2.ExpansionHubEx;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public abstract class jeremy extends LinearOpMode {
    public CameraName cameraName;
    //
    ExpansionHubEx controlHub;
    ExpansionHubEx expansionHub;
    //
    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor backRight;
    //
    DcMotor intake;
    DcMotor launcher;
    DcMotor wobble;//also X Encoder
    DcMotor xEnc;
    DcMotor yEncLeft;
    DcMotor yEncRight;
    //
    Servo feed;
    Servo keeper;
    Servo kelper;
    CRServo intakefeed;
    //
    DistanceSensor frontJS;
    DistanceSensor leftJS;
    DistanceSensor rightJS;
    //DistanceSensor oleftJS;
    //DistanceSensor orightJS;
    Double fJSlast;
    Double lJSlast;
    Double olJSlast;
    Double rJSlast;
    Double orJSlast;
    //
    ColorSensor tape;
    //
    final static Double FEEDPULL = 1.0;
    final static Double FEEDPUSH = 0.5;
    final static Double KEEPERCLOSED = 0.0;
    final static Double KEEPEROPEN = 0.5;
    //
    final static Double R2 = 15.5;//use for arcing odometry
    //
    DigitalChannel wobbleUp;
    //
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    static Float origin = 0F;
    //Double angle = 0.0;
    //
    OpenCvCamera webcam;
    StoneOrientationAnalysisPipeline pipeline;
    RingPipeline ringPipe;
    //
    public static Boolean cvReady = false;
    Integer widthThresh = 30;
    Integer heightThresh = 30;
    RingPipeline.AnalyzedRingGr bestRing = new RingPipeline.AnalyzedRingGr();
    //
    Double loctarang;//local
    Double glotarang;
    //
    DistanceSensor test;
    //
    Integer cpr = 28; //counts per rotation
    double gearratio = 139;
    Double diameter = 4.125;
    Double cpi = (cpr * gearratio)/(Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
    Double bias = 0.14;//0.714
    //
    Double conversion = cpi * bias;
    //
    static final Double launchConv = 1069.364;
    //
    Double turnSpeed = 15.0;//degrees second at full power
    //
    //<editor-fold desc="Var for Extr Functions">
    float leftx;
    float lefty;
    float rightx;
    //
    Boolean aPressed = false;
    Boolean bPressed = false;
    Boolean yPressed = false;
    Boolean oyPressed = false;
    Boolean xPressed = false;
    Boolean dLeftPressed = false;
    Boolean dUpPressed = false;
    Boolean dDownPressed = false;
    Double intakePower = 1.0;
    Double intakeFeederPower = 1.0;
    Boolean feedRunning = false;
    Long stopTime = 0L;
    Boolean keepClosed = true;
    //
    Boolean stopPending = false;
    Boolean intakeRunning = false;
    Boolean intakeFeederRunning = false;
    Boolean arrowPressed = false;
    //
    Double launcherPower = 0.925;
    Double psLauncherPower = 0.8;
    Boolean launcherToggle = false;
    Boolean togglePressed = false;
    Boolean psTogglePressed = false;
    //
    Double launcherRPM = 2600.0;
    Double psLauncherRPM = 2400.0;
    //
    Long feedStartTime = 0L;
    Long timeDif = 0L;
    //
    Integer tapeMode = 0;
    Integer tapeStart = 0;
    Boolean tapeSlowTurn = false;
    //</editor-fold>
    //
    //<editor-fold desc="Odometry">
    Integer ocpr = 720;
    Double odia = 1.45;//diameter of odometry wheels in inches
    Double ocpi = ocpr / (Math.PI * odia);
    Double obias = 1.85;
    Double tbias = 0.72876;
    //
    final static Double ROBOT_WIDTH = 17.5;//in inches
    final static Double ROBOT_LENGTH = 17.0;//in inches
    //
    final static Double FIELD_DIM_REAL = 144.0;
    final static Double FIELD_DIM_DIG = 153.0;
    //
    Double oconv = ocpi * obias;//odometry conversion
    Double oconvCan = FIELD_DIM_REAL / FIELD_DIM_DIG;
    //
    public Double xPos = 0.0;
    public Double yPos = 0.0;
    public Double gcAngle = 0.0;
    //
    Double oxOffset = 5.5;//positive is number of inches right from center
    Double oyOffset = -6.25 * 0.6842;//positive is number of inches forward from center, second number is y turn bias
    //
    Double rdoff = 6.5;
    Double ldoff = 8.0;
    Double fdoff = 10.0;
    //
    //</editor-fold>
    //
    final static Double propConst = 0.000002;//0.000002
    final static Double dampConst = 0.0002;//0.0001
    double lastError = 0.0;
    //
    Boolean opModeStarted;
    //
    double rollingAvg = 0;
    //
    double rpm = 0;
    int lastEnc = 0;
    Long lastTime =  System.currentTimeMillis();
    Long deltaTime = 0L;
    ArrayList<Double> rpmSamples = new ArrayList<>();
    //
    //<editor-fold desc="Init Functions">
    public void Init(){
        controlHub = hardwareMap.get(ExpansionHubEx.class, "Control Hub");
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        controlHub.setLedColor(255,0,0);
        expansionHub.setLedColor(255,0,0);
        //
        chassisHardware();
        motorHardware();
        sensorHardware();
        //
        resetEncoders();
        motorsWithEncoders();
        //
        setMotorReversals();
        //
        initGyro();
        initOpen();
        //
        for(int i = 0; i < 6; i++){
            rpmSamples.add(0.0);
        }
        //
        if(controlHub.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS) > 12.7) {
            controlHub.setLedColor(0, 255, 0);
            expansionHub.setLedColor(0, 255, 0);
            //
            telemetry.addData("Initialization", "complete");
        }else if(controlHub.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS) > 12.5){
            controlHub.setLedColor(255, 196, 0);
            expansionHub.setLedColor(255, 196, 0);
            //168, 28, 255
            telemetry.addData("Initialization", "complete");
            telemetry.addData("WARNING", "Battery power low");
        }else{
            controlHub.setLedColor(168, 28, 255);
            expansionHub.setLedColor(168, 28, 255);
            //
            telemetry.addData("Initialization", "complete");
            telemetry.addData("WARNING", "Battery power very low");
        }
        telemetry.update();
        //
    }
    //
    public void InitNoOpen(){
        controlHub = hardwareMap.get(ExpansionHubEx.class, "Control Hub");
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        controlHub.setLedColor(255,0,0);
        expansionHub.setLedColor(255,0,0);
        //
        chassisHardware();
        motorHardware();
        sensorHardware();
        //
        resetEncoders();
        motorsWithEncoders();
        //
        setMotorReversals();
        //
        initGyro();
        //
        for(int i = 0; i < 6; i++){
            rpmSamples.add(0.0);
        }
        //
        if(controlHub.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS) > 12.7) {
            controlHub.setLedColor(0, 255, 0);
            expansionHub.setLedColor(0, 255, 0);
            //
            telemetry.addData("Initialization", "complete");
        }else if(controlHub.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS) > 12.5){
            controlHub.setLedColor(255, 196, 0);
            expansionHub.setLedColor(255, 196, 0);
            //168, 28, 255
            telemetry.addData("Initialization", "complete");
            telemetry.addData("WARNING", "Battery power low");
        }else{
            controlHub.setLedColor(168, 28, 255);
            expansionHub.setLedColor(168, 28, 255);
            //
            telemetry.addData("Initialization", "complete");
            telemetry.addData("WARNING", "Battery power extremely low");
        }
        telemetry.update();
    }
    //
    public void YodaInit(){
        //
        initGyro();
        initOpen();
    }
    //
    public void InitLite(){
        chassisHardware();
        //
        resetEncoders();
        motorsWithEncoders();
        //
        setMotorReversals();
        //
        initGyro();
    }
    //
    public void chassisHardware(){
        frontRight = hardwareMap.dcMotor.get("frontright");
        frontLeft = hardwareMap.dcMotor.get("frontleft");
        backRight = hardwareMap.dcMotor.get("backright");
        backLeft = hardwareMap.dcMotor.get("backleft");
    }
    //
    public void motorHardware(){
        intake = hardwareMap.dcMotor.get("intake");
        launcher = hardwareMap.dcMotor.get("launcher");
        wobble = hardwareMap.dcMotor.get("wobble");
        xEnc = hardwareMap.dcMotor.get("wobble");
        yEncLeft = hardwareMap.dcMotor.get("yenc");
        yEncRight = hardwareMap.dcMotor.get("launcher");
        //
        feed = hardwareMap.servo.get("feed");
        keeper = hardwareMap.servo.get("keeper");
        kelper = hardwareMap.servo.get("kelper");
        intakefeed = hardwareMap.crservo.get("intakefeed");
        //
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wobble.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yEncLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //
        MotorConfigurationType motorConfigurationType = launcher.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        launcher.setMotorType(motorConfigurationType);
        //
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//run with encoder
    }
    //
    public void sensorHardware(){
        wobbleUp = hardwareMap.get(DigitalChannel.class, "wobbleUp");
        //test = hardwareMap.get(DistanceSensor.class, "test");
        frontJS = hardwareMap.get(DistanceSensor.class, "frontjs");
        leftJS = hardwareMap.get(DistanceSensor.class, "leftjs");
        //oleftJS = hardwareMap.get(DistanceSensor.class, "oleftjs");
        //orightJS = hardwareMap.get(DistanceSensor.class, "orightjs");
        rightJS = hardwareMap.get(DistanceSensor.class, "rightjs");
        fJSlast = frontJS.getDistance(DistanceUnit.INCH);
        tape = hardwareMap.get(ColorSensor.class, "tape");
    }
    //
    public void setMotorReversals(){
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    //
    public void motorsWithEncoders(){
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    //
    public void motorsWithoutEncoders(){
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    //
    public void motorsToPosition(){
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    //
    public void resetEncoders() {
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    //
    public void initGyro(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "GyroCal.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        telemetry.addData("init imu","");
        telemetry.update();
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //origin = -angles.firstAngle;
        telemetry.addData("imu initiated", "");
        telemetry.addData("Angle",getAngle());
        telemetry.update();
    }
    //
    public void initOpen(){
        //
        cvReady = false;
        //
        telemetry.addData("OpenCV", "initilaizing");
        telemetry.update();
        //
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        //
        sleep(1000);
        //
        // Open async and start streaming inside opened callback
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
            @Override
            public void onOpened(){
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                //
                ringPipe = new RingPipeline();
                webcam.setPipeline(ringPipe);
            }
        });
        //
        // Tell telemetry to update faster than the default 250ms period :)
        telemetry.setMsTransmissionInterval(20);
        //
        while(!cvReady){
            telemetry.addData("cvready", cvReady);
            telemetry.update();
        }
        telemetry.addData("cvready", true);
        telemetry.update();
        sleep(100);
        //
    }
    //
    public void initOpenStone(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        //
        // Open async and start streaming inside opened callback
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
            @Override
            public void onOpened(){
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                //
                pipeline = new StoneOrientationAnalysisPipeline();
                webcam.setPipeline(pipeline);
            }
        });
        //
        // Tell telemetry to update faster than the default 250ms period :)
        telemetry.setMsTransmissionInterval(20);
    }
    //
    public void waitForStartify(){
        waitForStart();
    }
    //</editor-fold>
    //
    //<editor-fold desc="Movement">
    public void moveTurn(double x, double y, double t, double factor, double turnFactor, double ayaw) {
        //
        double cyaw = -angles.firstAngle;
        if (t > 0 && cyaw - ayaw < -90) {
            cyaw += 360;
        } else if (t < 0 && cyaw - ayaw > 90) {
            cyaw = -360 - cyaw;
        }
        //
        double total = pythagorus(y, x);//find total power
        //
        double aWheelsPower;
        double bWheelsPower;
        //
        if (total == 0) {
            aWheelsPower = 0;
            bWheelsPower = 0;
        } else {
            double angle = inverseTrigGyro(x, y, total);//calculate angle of joystick
            //
            double calc = cyaw - ayaw;
            calc = fixAngle(calc);
            angle += calc;
            //
            aWheelsPower = Math.cos(angle * Math.PI / 180);//front left & back right
            bWheelsPower = Math.sin(angle * Math.PI / 180);//front right & back left
        }
        //
        double a = ((0.5 * aWheelsPower * total) + (0.5 * turnFactor * t)) * factor;//front left
        double b = ((0.5 * bWheelsPower * total) + (-0.5 * turnFactor * t)) * factor;//front right
        double c = ((0.5 * bWheelsPower * total) + (0.5 * turnFactor * t)) * factor;//back left
        double d = ((0.5 * aWheelsPower * total) + (-0.5 * turnFactor * t)) * factor;//back right
        //
        frontLeft.setPower(a);
        frontRight.setPower(b);
        backLeft.setPower(c);
        backRight.setPower(d);//set motor powers
        //
    }
    //
    public void move(double x, double y, double factor){
        //
        motorsWithEncoders();
        //
        //double total = pythagorus(y, x);//find total power
        double total = 1;
        //
        double angle = inverseTrigGyro(x, y, total);//calculate angle of joystick
        //
        double aWheelsPower = Math.cos(angle * Math.PI / 180);
        double bWheelsPower = Math.sin(angle * Math.PI / 180);//find power for a/b motors
        //
        telemetry.addData("a power", aWheelsPower);
        telemetry.addData("b power", bWheelsPower);
        //
        frontRight.setPower(bWheelsPower * total * factor);
        backLeft.setPower(bWheelsPower * total * factor);
        frontLeft.setPower(aWheelsPower * total * factor);
        backRight.setPower(aWheelsPower * total * factor);//set motor powers
        //
        telemetry.addData("front right set",bWheelsPower * total * factor + ", get: " + frontRight);
    }
    //
    public void globalMoveTurn(double x, double y, double t, double factor, double turnFactor, double origin) {
        //
        double cang = fixAngle(getAngle() - origin);
        //
        double total = pythagorus(y, x);//find total power
        //
        double aWheelsPower;
        double bWheelsPower;
        //
        if (total == 0) {
            aWheelsPower = 0;
            bWheelsPower = 0;
        } else {
            double h = inverseTrigGyro(x, y, total);//calculate angle of joystick
            loctarang = h;
            //
            double g = h + cang;
            //g = fixAngle(g);
            glotarang = g;
            //
            aWheelsPower = Math.cos(g * Math.PI / 180);//front left & back right
            bWheelsPower = Math.sin(g * Math.PI / 180);//front right & back left
        }
        //
        double moveP = 0.5;
        if(aWheelsPower == 0 && bWheelsPower == 0){
            moveP = 0;
        }else if(t == 0){
            moveP = 1;
        }
        double turnP = 1 - moveP;
        //
        double a = ((moveP * aWheelsPower * total) + (turnP * turnFactor * t)) * factor;//front left
        double b = ((moveP * bWheelsPower * total) + (-turnP * turnFactor * t)) * factor;//front right
        double c = ((moveP * bWheelsPower * total) + (turnP * turnFactor * t)) * factor;//back left
        double d = ((moveP * aWheelsPower * total) + (-turnP * turnFactor * t)) * factor;//back right
        //
        frontLeft.setPower(a);
        frontRight.setPower(b);
        backLeft.setPower(c);
        backRight.setPower(d);//set motor powers
        //
    }
    //
    public void globalMeccMove(double x, double y, double t, double Gfac, double Tfac, double Hfac, double origin){
        //
        double cang = fixAngle(getAngle() - origin);
        //
        double total = pythagorus(y, x);//find total power
        //
        double aWheelsPower;
        double bWheelsPower;
        //
        if (total == 0) {
            aWheelsPower = 0;
            bWheelsPower = 0;
        } else {
            double h = inverseTrigGyro(x, y, total);//calculate angle of joystick
            loctarang = h;
            //
            double g = h + cang;
            //g = fixAngle(g);
            glotarang = g;//may need to add 90 because of angle starting position
            //
            aWheelsPower = getAPower(g, Hfac);//front left & back right
            bWheelsPower = getBPower(g, Hfac);//front right & back left
        }
        //
        double moveP = 0.5;
        if(aWheelsPower == 0 && bWheelsPower == 0){
            moveP = 0;
        }else if(t == 0){
            moveP = 1;
        }
        double turnP = 1 - moveP;
        //
        telemetry.addData("Local angle", loctarang);
        telemetry.addData("Global angle", glotarang);
        telemetry.addData("A wheels", aWheelsPower);
        telemetry.addData("B wheels", bWheelsPower);
        //
        double a = ((moveP * aWheelsPower * total) + (turnP * Tfac * t)) * Gfac;//front left
        double b = ((moveP * bWheelsPower * total) + (-turnP * Tfac * t)) * Gfac;//front right
        double c = ((moveP * bWheelsPower * total) + (turnP * Tfac * t)) * Gfac;//back left
        double d = ((moveP * aWheelsPower * total) + (-turnP * Tfac * t)) * Gfac;//back right
        //
        frontLeft.setPower(a);
        frontRight.setPower(b);
        backLeft.setPower(c);
        backRight.setPower(d);//set motor powers
        //
        telemetry.addData("frontLeft power", frontLeft.getPower());
        //
    }
    //
    public void fullBaby(double x, double y, double t, double turnFactor, double powerFactor){
        if(x == 0 && y == 0 && t == 0){//no motion
            //
            still();
            telemetry.addData("Motion","Still");
            //
        }else if(t != 0) {
            //
            if(x == 0 && y == 0){
                babyTurn(t, powerFactor);
            }else if(Math.abs(x) > Math.abs(y)){
                babyMeccTurn(x, t, turnFactor, powerFactor);
            }else{
                babyMoveTurn(y, t, turnFactor, powerFactor);
            }
            //
        }else{//moving
            //
            telemetry.addData("Moving", "regular");
            babyMeccMove(x, y, powerFactor);
            //
        }
    }
    //
    public void babyMeccMove(double x, double y, double Gfac){
        if(Math.abs(x) > Math.abs(y)){
            frontLeft.setPower(x * Gfac);
            frontRight.setPower(x * -Gfac);
            backLeft.setPower(x * -Gfac);
            backRight.setPower(x * Gfac);
        }else{
            frontLeft.setPower(y * Gfac);
            frontRight.setPower(y * Gfac);
            backLeft.setPower(y * Gfac);
            backRight.setPower(y * Gfac);
        }
    }
    //
    public void babyTurn(double t, double Gfac){
        frontLeft.setPower(t * Gfac);
        frontRight.setPower(t * -Gfac);
        backLeft.setPower(t * Gfac);
        backRight.setPower(t * -Gfac);
    }
    //
    public void babyMoveTurn(double m, double t, double Tfac, double Gfac){
        double leftPower = (t * Gfac * Tfac) + (m * Gfac * (1 - Tfac));
        double rightPower = (-t * Gfac * Tfac) + (m * Gfac * (1 - Tfac));
        frontLeft.setPower(leftPower);
        frontRight.setPower(rightPower);
        backLeft.setPower(leftPower);
        backRight.setPower(rightPower);
    }
    //
    public void babyMeccTurn(double m, double t, double Tfac, double Gfac){
        frontLeft.setPower((t * Gfac * Tfac) + (m * Gfac * (1 - Tfac)));
        frontRight.setPower((-t * Gfac * Tfac) + (-m * Gfac * (1 - Tfac)));
        backLeft.setPower((t * Gfac * Tfac) + (-m * Gfac * (1 - Tfac)));
        backRight.setPower((-t * Gfac * Tfac) + (m * Gfac * (1 - Tfac)));
    }
    //
    public void turnWithEncoder(double input){
        motorsWithEncoders();
        //
        frontLeft.setPower(input);
        backLeft.setPower(input);
        frontRight.setPower(-input);
        backRight.setPower(-input);
    }
    //
    public void moveWithEncoder(double speed){
        //
        motorsWithEncoders();
        //
        frontLeft.setPower(speed);
        backLeft.setPower(speed);
        frontRight.setPower(speed);
        backRight.setPower(speed);
        return;
    }
    //
    public void still(){
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
    //</editor-fold>
    //
    //<editor-fold desc="TeleOp Functions">
    public void runIntake(boolean aButton, boolean yButton, boolean oyButton){
        if(aButton && !aPressed){
            aPressed = true;
            intakeRunning = !intakeRunning;
            if(intakeRunning){
                intake.setPower(intakePower);
                intakefeed.setPower(intakeFeederPower);
            }else{
                intake.setPower(0.0);
                intakefeed.setPower(0);
            }
        }else if(!aButton && aPressed){
            aPressed = false;
        }
        //
        if(yButton && !yPressed){
            yPressed = true;
            intakePower = -intakePower;
            if(intakeRunning){
                intake.setPower(intakePower);
            }
        }else if(!yButton && yPressed){
            yPressed = false;
        }
        //
        if (oyButton && !oyPressed) {
            oyPressed = true;
            intakePower = -intakePower;
            if (intakeRunning) {
                intake.setPower(intakePower);
            }
        }else if(!oyButton && oyPressed){
            oyPressed = false;
        }
    }
    //
    public void runIntakeFeeder(boolean dLeft, boolean dUp){
        /*if(dLeft && !dLeftPressed){
            dLeftPressed = true;
            intakeFeederRunning = !intakeFeederRunning;
            if(intakeFeederRunning){
                intakefeed.setPower(intakeFeederPower);
            }else {
                intakefeed.setPower(0);
            }
        }else if(!dLeft && dLeftPressed){
            dLeftPressed = false;
        }*/
        //
        if(dUp && !dUpPressed){
            dUpPressed = true;
            intakeFeederPower = -intakeFeederPower;
            if(intakeFeederRunning){
                intakefeed.setPower(intakeFeederPower);
            }
        }else if(!dUp && dUpPressed){
            dUpPressed = false;
        }
    }
    //
    public void runLauncherIncr(boolean dUp, boolean dDown, boolean leftBumper, boolean rightBumper){
        if((dUp || dDown) && !arrowPressed){
            arrowPressed = true;
            if(dUp && launcherPower < 1){
                launcherPower += .025;
            }else if(launcherPower > 0){
                launcherPower -= .025;
            }
            if(launcherToggle) {
                launcher.setPower(launcherPower);
            }
        }else if(!dUp && !dDown && arrowPressed){
            arrowPressed = false;
        }
        //
        if(leftBumper && !togglePressed){
            launcherToggle = !launcherToggle;
            if(launcherToggle){
                launcher.setPower(launcherPower);
            }else{
                launcher.setPower(0);
            }
            togglePressed = true;
        }else if(!leftBumper && togglePressed){
            togglePressed = false;
        }
        //
        if(rightBumper && !psTogglePressed){
            launcherPower = psLauncherPower;
            launcher.setPower(psLauncherPower);
            psTogglePressed = true;
        }else if(!rightBumper && psTogglePressed){
            psTogglePressed = false;
        }
        //
    }
    //
    public void runLauncherSmart(boolean dUp, boolean dDown, boolean leftBumper, boolean rightBumper, double rpm){
        if((dUp || dDown) && !arrowPressed){//arrow pressed
            arrowPressed = true;
            if(dUp && launcherRPM < 3000){//incr rpm by 50
                launcherRPM += 50;
            }else if(launcherRPM > 0){//decr rpm by 50
                launcherRPM -= 50;
            }
        }else if(!dUp && !dDown && arrowPressed){//arrow released
            arrowPressed = false;
        }
        //
        if(leftBumper && !togglePressed){//toggle pressed
            launcherToggle = !launcherToggle;//invert state
            togglePressed = true;
            if(!launcherToggle){//turn off if off
                launcher.setPower(0);
            }else{
                intake.setPower(0);
                intakeRunning = false;
                intakefeed.setPower(0);
            }
        }else if(!leftBumper && togglePressed){//toggle released
            togglePressed = false;
        }
        //
        if(rightBumper && !psTogglePressed){//ps toggle pressed
            launcherRPM = psLauncherRPM;
            psTogglePressed = true;
        }else if(!rightBumper && psTogglePressed){//ps toggle released
            psTogglePressed = false;
        }
        //
        if(launcherToggle) {
            launcher.setPower(calcPower(rpm,launcherRPM));
        }
    }
    //
    public void runFeed(boolean bButton){
        if(bButton && !bPressed && !feedRunning){
            feedStartTime = System.currentTimeMillis();
            //
            feedRunning = true;
            bPressed = true;
            //
            feed.setPosition(FEEDPUSH);
        }else if(feedRunning && (System.currentTimeMillis() > feedStartTime + 900)){
            feedRunning = false;
        }else if(feedRunning && System.currentTimeMillis() > feedStartTime + 400){
            feed.setPosition(FEEDPULL);
        }else if(!bButton && bPressed){
            bPressed = false;
        }
    }
    //
    public void runWobble(float leftTrigger, float rightTrigger){
        if(Math.abs(rightTrigger) > 0) {
            wobble.setPower(rightTrigger * .4);
        }else/* if(!wobbleUp.getState())*/{
            wobble.setPower(-leftTrigger * .4);
        /*}else if(wobbleUp.getState()){
            wobble.setPower(0);*/
        }
    }
    //
    public void keepEmDead(boolean xButton){
        if(xButton && !xPressed){
            if(keepClosed){
                keeper.setPosition(1.0);//set open
                kelper.setPosition(0.0);
            }else{
                keeper.setPosition(0.0);//set closed
                kelper.setPosition(1.0);
            }
            keepClosed = !keepClosed;
            xPressed = true;
        }else if(!xButton && xPressed){
            xPressed = false;
        }
    }
    //
    public void runTaper(double speed){
        if(angleInBounds(0, 5)){
            turnToAngleError(0, .05, 2);
        }else {
            turnToAngleError(0, .2, 7);//you can use tapemode
        }
        //
        frontLeft.setPower(-speed);
        backLeft.setPower(-speed);
        frontRight.setPower(-speed);
        backRight.setPower(-speed);
        //
        double startPos = frontLeft.getCurrentPosition();
        //
        while (tape.red() < 80 && opModeIsActive() && ((startPos - frontLeft.getCurrentPosition()) / conversion < 30)){}
        still();
        //
        moveToPosition(-5, speed);
        //
        motorsWithEncoders();
    }
    //
    public void autoPower(){
        strafeToPosition(-12, .3);
        //
        if(gamepad1.x){
            return;
        }
        turnToAngleError(5.5,.1,2);//first turn (right)
        if(gamepad1.x){
            return;
        }
        feed.setPosition(FEEDPUSH);
        sleep(1000);
        launcher.setPower(.81);
        //
        feed.setPosition(FEEDPULL);
        //
        if(gamepad1.x){
            return;
        }
        turnToAngleError(0, .1,2);//second turn (middle)
        if(gamepad1.x){
            return;
        }
        sleep(500);
        if(gamepad1.x){
            return;
        }
        feed.setPosition(FEEDPUSH);
        sleep(1000);
        launcher.setPower(.815);
        //
        feed.setPosition(FEEDPULL);
        //
        if(gamepad1.x){
            return;
        }
        turnToAngleError(-6,.1,2);//last turn (left)
        if(gamepad1.x){
            return;
        }
        sleep(500);
        feed.setPosition(FEEDPUSH);
        if(gamepad1.x){
            return;
        }
        sleep(1000);
        launcher.setPower(0);
        feed.setPosition(FEEDPULL);
    }
    //</editor-fold>
    //
    //<editor-fold desc="Autonomous">
    public void moveToPosition(double inches, double speed){
        //
        int move = (int)(Math.round(inches*conversion));
        //
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + move);
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + move);
        backRight.setTargetPosition(backRight.getCurrentPosition() + move);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + move);
        //
        motorsToPosition();
        //
        frontLeft.setPower(speed);
        backLeft.setPower(speed);
        frontRight.setPower(speed);
        backRight.setPower(speed);
        //
        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy() && opModeIsActive()){}
        still();
        return;
    }
    //
    public void moveToPositionLauncher(double inches, double speed, double rpmGoal){
        //
        int move = (int)(Math.round(inches*conversion));
        //
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + move);
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + move);
        backRight.setTargetPosition(backRight.getCurrentPosition() + move);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + move);
        //
        motorsToPosition();
        //
        frontLeft.setPower(speed);
        backLeft.setPower(speed);
        frontRight.setPower(speed);
        backRight.setPower(speed);
        //
        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy() && opModeIsActive()){
            calcRPM();
            launcher.setPower(calcPower(rollingAvg, rpmGoal));
        }
        still();
        return;
    }
    //
    public void wiggleToPosition(boolean left, double inches, double speed){
        //
        int move = (int)(Math.round(inches*conversion));
        //
        if(left) {
            backLeft.setTargetPosition(backLeft.getCurrentPosition() + move);
            frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + move);
            //
            motorsToPosition();
            //
            frontLeft.setPower(speed);
            backLeft.setPower(speed);
            //
            while (frontLeft.isBusy() && backLeft.isBusy() && opModeIsActive()){}
            //
        }else{
            backRight.setTargetPosition(backRight.getCurrentPosition() + move);
            frontRight.setTargetPosition(frontRight.getCurrentPosition() + move);
            //
            motorsToPosition();
            //
            frontRight.setPower(speed);
            backRight.setPower(speed);
            //
            while (frontRight.isBusy() && backRight.isBusy() && opModeIsActive()){}
            //
        }
        //
        still();
        return;
    }
    //
    public void strafeToPosition(double inches, double speed){
        //
        int move = (int)(Math.round(inches*conversion));
        //
        backLeft.setTargetPosition(backLeft.getCurrentPosition() - move);
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + move);
        backRight.setTargetPosition(backRight.getCurrentPosition() + move);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - move);
        //
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        frontLeft.setPower(speed);
        backLeft.setPower(-speed);
        frontRight.setPower(-speed);
        backRight.setPower(speed);
        //
        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy() && opModeIsActive()){}
        still();
        return;
    }
    //
    public void meccToPosition(double theta1, double distance, double t, double power, double Gfac, double Tfac, double Hfac){
        //
        double aWheelsPower = getAPower(theta1, Hfac);//front left & back right
        double bWheelsPower = getBPower(theta1, Hfac);//front right & back left
        //
        int move = (int)(Math.round(distance*conversion));
        //
        telemetry.addData("Move", move);
        telemetry.addData("Conversion", conversion);
        //
        double moveP = 0.5;
        if(aWheelsPower == 0 && bWheelsPower == 0){
            moveP = 0;
        }else if(t == 0){
            moveP = 1;
        }
        double turnP = 1 - moveP;
        //
        double a = ((moveP * aWheelsPower * power) + (turnP * Tfac * t)) * Gfac;//front left
        double b = ((moveP * bWheelsPower * power) + (-turnP * Tfac * t)) * Gfac;//front right
        double c = ((moveP * bWheelsPower * power) + (turnP * Tfac * t)) * Gfac;//back left
        double d = ((moveP * aWheelsPower * power) + (-turnP * Tfac * t)) * Gfac;//back right
        //
        telemetry.addData("frontleft distance", move * aWheelsPower);
        telemetry.update();
        //
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + (int)((moveP * move * aWheelsPower) + (turnP)));
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - (int)(move * bWheelsPower));
        backLeft.setTargetPosition(backLeft.getCurrentPosition() - (int)(move * bWheelsPower));
        backRight.setTargetPosition(backRight.getCurrentPosition() + (int)(move * aWheelsPower));
        //
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        frontLeft.setPower(a);
        frontRight.setPower(b);
        backLeft.setPower(c);
        backRight.setPower(d);
        //
        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy() && opModeIsActive()){}
        still();
        return;
        //
    }
    //
    public void globalMeccToPosition(double theta1, double distance, double t, double power, double Gfac, double Tfac, double Hfac, double origin){
        //
        double cang = fixAngle(getAngle() - origin);//calculate global to local difference
        //
        double g = theta1 + cang;//find global angle (theta1 is local)
        //g = fixAngle(g);
        glotarang = fixAngle(g + 90);//may need to add 90 because of angle starting position
        //
        double aWheelsPower = getAPower(g, Hfac);//front left & back right
        double bWheelsPower = getBPower(g, Hfac);//front right & back left
        //
        int move = (int)(Math.round(distance*conversion));
        //
        double moveP = 0.5;
        if(aWheelsPower == 0 && bWheelsPower == 0){
            moveP = 0;
        }else if(t == 0){
            moveP = 1;
        }
        double turnP = 1 - moveP;
        //
        double a = ((moveP * aWheelsPower * power) + (turnP * Tfac * t)) * Gfac;//front left
        double b = ((moveP * bWheelsPower * power) + (-turnP * Tfac * t)) * Gfac;//front right
        double c = ((moveP * bWheelsPower * power) + (turnP * Tfac * t)) * Gfac;//back left
        double d = ((moveP * aWheelsPower * power) + (-turnP * Tfac * t)) * Gfac;//back right
        //
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + (int)(move * a));
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - (int)(move * b));
        backLeft.setTargetPosition(backLeft.getCurrentPosition() - (int)(move * c));
        backRight.setTargetPosition(backRight.getCurrentPosition() + (int)(move * d));
        //
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        frontLeft.setPower(a);
        backLeft.setPower(b);
        frontRight.setPower(c);
        backRight.setPower(d);
        //
        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy() && opModeIsActive()){}
        still();
        return;
        //
    }
    //
    public void turnWithGyro(double degrees, double speedDirection){
        //<editor-fold desc="Initialize">
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double yaw = -angles.firstAngle;//make this negative
        telemetry.addData("Speed Direction", speedDirection);
        telemetry.addData("Yaw", yaw);
        telemetry.update();
        //
        telemetry.addData("stuff", speedDirection);
        telemetry.update();
        //
        double first;
        double second;
        //</editor-fold>
        //
        if (speedDirection > 0){//set target positions
            //<editor-fold desc="turn right">
            if (degrees > 10){
                first = (degrees - 10) + devertify(yaw);
                second = degrees + devertify(yaw);
            }else{
                first = devertify(yaw);
                second = degrees + devertify(yaw);
            }
            //</editor-fold>
        }else{
            //<editor-fold desc="turn left">
            if (degrees > 10){
                first = devertify(-(degrees - 10) + devertify(yaw));
                second = devertify(-degrees + devertify(yaw));
            }else{
                first = devertify(yaw);
                second = devertify(-degrees + devertify(yaw));
            }
            //
            //</editor-fold>
        }
        //
        //<editor-fold desc="Go to position">
        Double firsta = convertify(first - 5);//175
        Double firstb = convertify(first + 5);//-175
        //
        turnWithEncoder(speedDirection);
        //
        if (Math.abs(firsta - firstb) < 11) {
            while (!(firsta < yaw && yaw < firstb) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
            }
        }else{
            //
            while (!((firsta < yaw && yaw < 180) || (-180 < yaw && yaw < firstb)) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
            }
        }
        //
        Double seconda = convertify(second - 5);//175
        Double secondb = convertify(second + 5);//-175
        //
        turnWithEncoder(speedDirection / 3);
        //
        if (Math.abs(seconda - secondb) < 11) {
            while (!(seconda < yaw && yaw < secondb) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                /*telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();*/
            }
            while (!((seconda < yaw && yaw < 180) || (-180 < yaw && yaw < secondb)) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
            }
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
        //</editor-fold>
        //
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public double devertify(double degrees){
        if (degrees < 0){
            degrees = degrees + 360;
        }
        return degrees;
    }
    public double convertify(double degrees){
        if (degrees > 179){
            degrees = -(360 - degrees);
        } else if(degrees < -180){
            degrees = 360 + degrees;
        } else if(degrees > 360){
            degrees = degrees - 360;
        }
        return degrees;
    }
    //
    public void turnToAngle(double angle, double speed){
        double current = getAngle();
        //
        if (current > angle){
            turnWithEncoder(-speed);
        }else{
            turnWithEncoder(speed);
        }
        //
        while (!(angle - 5 < current && current < angle + 5)){
            current = getAngle();
            /*telemetry.addData("Target",angle);
            telemetry.addData("Current",current);
            telemetry.update();*/
        }
        still();
        //
    }
    //
    public void turnToAngleError(double angle, double speed, double error){
        //
        double current = getAngle();
        //
        if (current > angle){
            turnWithEncoder(-speed);
        }else{
            turnWithEncoder(speed);
        }
        //
        while (!(angleInBounds(angle, error)) && opModeIsActive()){
            calcRPM();
            launcher.setPower(calcPower(rollingAvg, rollingAvg));
            //angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //current = -angles.firstAngle;
            /*telemetry.addData("Target",angle);
            telemetry.addData("Current",current);
            telemetry.update();*/
        }
        still();
        //
    }
    //
    public void turnToAngleErrorLauncher(double angle, double speed, double error, double rpmGoal, TelemetryPacket packet, FtcDashboard dashboard){
        //
        double current = getAngle();
        //
        if (current > angle){
            turnWithEncoder(-speed);
        }else{
            turnWithEncoder(speed);
        }
        //
        while (!(angleInBounds(angle, error)) && opModeIsActive()){
            calcRPM();
            launcher.setPower(calcPower(rollingAvg, rpmGoal));
            packet.put("rpm", rollingAvg);
            packet.put("rpm goal", rpmGoal);
            dashboard.sendTelemetryPacket(packet);
        }
        still();
        //
    }
    //
    public void sleepLauncher(long milliseconds, double rpmGoal){
        Long startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() < startTime + milliseconds){
            calcRPM();
            launcher.setPower(calcPower(rollingAvg, rpmGoal));
        }
        still();
    }
    //
    /*public void turnToAngleOdo(double angle, double speed, double error){
        //
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double current = -angles.firstAngle;
        //
        if (current > angle){
            turnWithEncoder(-speed);
        }else{
            turnWithEncoder(speed);
        }
        //
        while (!(inBounds(angle, error)) && opModeIsActive()){
            //angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //current = -angles.firstAngle;
            /*telemetry.addData("Target",angle);
            telemetry.addData("Current",current);
            telemetry.update();
        }
        still();
        //
    }*/
    //
    public void turnPast(double angle, double speed, boolean right){
        //
        double current = getAngle();
        //
        if (right){
            turnWithEncoder(speed);
            //
            while (!(getAngle() > angle) && opModeIsActive()){}
        }else {
            turnWithEncoder(-speed);
            //
            while (!(getAngle() < angle) && opModeIsActive()){}
        }
        still();
        //
    }
    //
    public void stageToPosition(double inches,double speed1, double speed2){
        //
        double Stage1;
        double Stage2;
        //
        if (inches > 0){
            Stage1 = inches - 10;
            Stage2 = 10;
        }else{
            Stage1 = inches + 10;
            Stage2 = -10;
        }
        //
        int move1 = (int)(Math.round(Stage1*conversion));
        int move2 = (int)(Math.round(Stage2*conversion));
        int move = move1 + move2;
        //
        int bench = frontLeft.getCurrentPosition() + move1;
        //
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + move);
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + move);
        backRight.setTargetPosition(backRight.getCurrentPosition() + move);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + move);
        //
        motorsToPosition();
        //
        frontLeft.setPower(speed1);
        backLeft.setPower(speed1);
        frontRight.setPower(speed1);
        backRight.setPower(speed1);
        //
        boolean set = false;
        //
        if(inches > 0){
            while (frontLeft.getCurrentPosition() < bench && opModeIsActive()){}
        }else{
            while (frontLeft.getCurrentPosition() > bench && opModeIsActive()){}
        }
        //
        frontLeft.setPower(speed2);
        backLeft.setPower(speed2);
        frontRight.setPower(speed2);
        backRight.setPower(speed2);
        //
        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy() && opModeIsActive()){}
        //
        /*while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy() && opModeIsActive()){
            //
            if (inches > 0 && !set){
                if (frontLeft.getCurrentPosition() > bench){
                    frontLeft.setPower(speed2);
                    backLeft.setPower(speed2);
                    frontRight.setPower(speed2);
                    backRight.setPower(speed2);
                    set = true;
                }
            }else if (!set){
                if (frontLeft.getCurrentPosition() < bench){
                    frontLeft.setPower(speed2);
                    backLeft.setPower(speed2);
                    frontRight.setPower(speed2);
                    backRight.setPower(speed2);
                    set = true;
                }
            }
            //
            /*telemetry.addData("target",frontLeft.getTargetPosition());
            telemetry.addData("current",frontLeft.getCurrentPosition());
            telemetry.update();
        }*/
        still();
        return;
    }
    //
    public void odoYToPosition(FtcDashboard dashboard, TelemetryPacket packet, double yCoord, double speed){
        int curX = xEnc.getCurrentPosition();
        int curYLeft = yEncLeft.getCurrentPosition();
        //int curYRight = lastYRight;
        //
        int[] storeEncs;
        //
        if(yPos < yCoord) {//go forward
            frontLeft.setPower(speed);
            backLeft.setPower(speed);
            frontRight.setPower(speed);
            backRight.setPower(speed);
            //
            while(yPos < yCoord && opModeIsActive()){
                storeEncs = updateOdo(dashboard, packet, curX, curYLeft);
                curX = storeEncs[0];
                curYLeft = storeEncs[1];
                //sleep(100);
            }
            //
            still();
            //
        }else{//go back
            frontLeft.setPower(-speed);
            backLeft.setPower(-speed);
            frontRight.setPower(-speed);
            backRight.setPower(-speed);
            //
            while(yPos > yCoord && opModeIsActive()){
                storeEncs = updateOdo(dashboard, packet, curX, curYLeft);
                curX = storeEncs[0];
                curYLeft = storeEncs[1];
                //sleep(100);
            }
            //
            still();
        }
    }
    //
    public void odoXToPosition(FtcDashboard dashboard, TelemetryPacket packet, double xCoord, double speed){
        int curX = xEnc.getCurrentPosition();
        int curYLeft = yEncLeft.getCurrentPosition();
        //int curYRight = lastYRight;
        //
        double lastAngle;
        //
        int[] storeEncs;
        //
        if(xPos < xCoord) {//go right
            frontLeft.setPower(speed);
            backLeft.setPower(-speed);
            frontRight.setPower(-speed);
            backRight.setPower(speed);
            //
            while(xPos < xCoord && opModeIsActive()){
                storeEncs = updateOdo(dashboard, packet, curX, curYLeft);
                curX = storeEncs[0];
                curYLeft = storeEncs[1];
            }
            //
            still();
            //
        }else{//go left
            frontLeft.setPower(-speed);
            backLeft.setPower(speed);
            frontRight.setPower(speed);
            backRight.setPower(-speed);
            //
            while(xPos > xCoord && opModeIsActive()){
                storeEncs = updateOdo(dashboard, packet, curX, curYLeft);
                curX = storeEncs[0];
                curYLeft = storeEncs[1];
            }
            //
            still();
        }
    }
    //
    public void powerFromCorner(double speed){
        //
        int lastX = xEnc.getCurrentPosition();
        int lastYLeft = yEncLeft.getCurrentPosition();
        int curX = lastX;
        int curYLeft = lastYLeft;
        //
        origin = getRawGyro();
        gcAngle = getAngle();
        double lastAngle = gcAngle;
        //
        xPos = (FIELD_DIM_DIG / 2) - (ROBOT_WIDTH / 2) - 10;//-10 is temporary
        yPos = (FIELD_DIM_DIG / -2) + (ROBOT_LENGTH / 2) - 10;
        //
        motorsToPosition();
        moveToPosition(10, .3);
        strafeToPosition(-10, .3);
        motorsWithEncoders();
        //
        frontLeft.setPower(speed);
        backLeft.setPower(speed);
        frontRight.setPower(speed);
        backRight.setPower(speed);
        //
        while (!inBounds(yPos, -12, 4) && opModeIsActive()) {
            lastX = curX;
            lastYLeft = curYLeft;
            lastAngle = gcAngle;//store old values
            //
            curX = xEnc.getCurrentPosition();
            curYLeft = yEncLeft.getCurrentPosition();
            gcAngle = getAngle();//retrieve new values
            //
            xPos += getXOdometry(curX - lastX, curYLeft - lastYLeft, lastAngle, gcAngle);
            yPos += getYOdometry(curX - lastX, curYLeft - lastYLeft, lastAngle, gcAngle);//run algorithm
            telemetry.addData("yPos", yPos);
            telemetry.update();
        }
        still();
        //
        sleep(500);
        //
        frontLeft.setPower(-speed);
        backLeft.setPower(speed);
        frontRight.setPower(speed);
        backRight.setPower(-speed);
        //
        while (!inBounds(xPos, 0, 3) && opModeIsActive()){
            lastX = curX;
            lastYLeft = curYLeft;
            lastAngle = gcAngle;//store old values
            //
            curX = xEnc.getCurrentPosition();
            curYLeft = yEncLeft.getCurrentPosition();
            gcAngle = getAngle();//retrieve new values
            //
            xPos += getXOdometry(curX - lastX, curYLeft - lastYLeft, lastAngle, gcAngle);
            yPos += getYOdometry(curX - lastX, curYLeft - lastYLeft, lastAngle, gcAngle);//run algorithm
            telemetry.addData("xPos", xPos);
            telemetry.update();
        }
        still();
        //
        turnToAngleError(15, .1, 3);
        //
    }
    //
    public void powerFromWall(double speed){
        frontLeft.setPower(-speed);
        backLeft.setPower(-speed);
        frontRight.setPower(-speed);
        backRight.setPower(-speed);
        //
        double startPos = frontLeft.getCurrentPosition();
        //
        while (tape.red() < 80 && opModeIsActive() && ((startPos - frontLeft.getCurrentPosition()) / conversion < 30)){}
        still();
        //
        if((startPos - frontLeft.getCurrentPosition()) / conversion > 30){
            return;
        }
        //
        moveToPosition(-3, .3);
        motorsWithEncoders();
        //
        int lastX = xEnc.getCurrentPosition();//set odo values
        int lastYLeft = yEncLeft.getCurrentPosition();
        int curX = lastX;
        int curYLeft = lastYLeft;
        //
        origin = getRawGyro();
        gcAngle = getAngle();
        double lastAngle = gcAngle;
        //
        xPos = 0.0;//set coordinates to (0,0)
        yPos = 0.0;
        //
        frontLeft.setPower(speed);
        backLeft.setPower(-speed);
        frontRight.setPower(-speed);
        backRight.setPower(speed);
        //
        launcher.setPower(.825);
        //
        while (!inBounds(xPos, 20, 3) && opModeIsActive()){
            lastX = curX;
            lastYLeft = curYLeft;
            lastAngle = gcAngle;//store old values
            //
            curX = xEnc.getCurrentPosition();
            curYLeft = yEncLeft.getCurrentPosition();
            gcAngle = getAngle();//retrieve new values
            //
            xPos += getXOdometry(curX - lastX, curYLeft - lastYLeft, lastAngle, gcAngle);
            yPos += getYOdometry(curX - lastX, curYLeft - lastYLeft, lastAngle, gcAngle);//run algorithm
            telemetry.addData("xPos", xPos);
            telemetry.update();
        }
        still();
        //turns need to be fixed so they don't go the wrong way
        turnPast(-6, .1, false);
        sleep(500);
        //turnToAngleError(5.5,.1,4);//first turn (right)
        turnPast(1.5, .1,true);
        feed.setPosition(FEEDPUSH);
        sleep(1000);
        launcher.setPower(.8);
        //
        feed.setPosition(FEEDPULL);
        //
        //turnToAngleError(-1, .1,2);//second turn (middle)
        turnPast(0, .1, false);
        sleep(1000);
        feed.setPosition(FEEDPUSH);
        sleep(1000);
        launcher.setPower(.81);
        //
        feed.setPosition(FEEDPULL);
        //
        //turnToAngleError(-6,.1,2);//last turn (left)
        turnPast(-6, .1, false);
        sleep(500);
        feed.setPosition(FEEDPUSH);
        sleep(1000);
        launcher.setPower(0);
        //
        feed.setPosition(FEEDPULL);
        sleep(1000);
    }
    //</editor-fold>
    //
    //<editor-fold desc="OpenCV">
    static class StoneOrientationAnalysisPipeline extends OpenCvPipeline {
        /*
         * Our working image buffers
         */
        Mat cbMat = new Mat();
        Mat thresholdMat = new Mat();
        Mat morphedThreshold = new Mat();
        Mat contoursOnPlainImageMat = new Mat();
        //
        /*
         * Threshold values
         */
        static final int CB_CHAN_MASK_THRESHOLD = 80;
        static final double DENSITY_UPRIGHT_THRESHOLD = 0.03;
        //
        /*
         * The elements we use for noise reduction
         */
        Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6, 6));
        //
        /*
         * Colors
         */
        static final Scalar TEAL = new Scalar(3, 148, 252);
        static final Scalar PURPLE = new Scalar(158, 52, 235);
        static final Scalar RED = new Scalar(255, 0, 0);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        static final Scalar BLUE = new Scalar(0, 0, 255);
        //
        static final int CONTOUR_LINE_THICKNESS = 2;
        static final int CB_CHAN_IDX = 2;
        //
        static class AnalyzedStone {
            StoneOrientationExample.StoneOrientationAnalysisPipeline.StoneOrientation orientation;
            double angle;
        }
        //
        enum StoneOrientation {
            UPRIGHT,
            NOT_UPRIGHT
        }
        //
        ArrayList<StoneOrientationExample.StoneOrientationAnalysisPipeline.AnalyzedStone> internalStoneList = new ArrayList<>();
        volatile ArrayList<StoneOrientationExample.StoneOrientationAnalysisPipeline.AnalyzedStone> clientStoneList = new ArrayList<>();
        //
        /*
         * Some stuff to handle returning our various buffers
         */
        enum Stage {
            FINAL,
            Cb,
            MASK,
            MASK_NR,
            CONTOURS;
        }
        //
        StoneOrientationExample.StoneOrientationAnalysisPipeline.Stage[] stages = StoneOrientationExample.StoneOrientationAnalysisPipeline.Stage.values();
        //
        // Keep track of what stage the viewport is showing
        int stageNum = 0;
        //
        @Override
        public void onViewportTapped() {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int nextStageNum = stageNum + 1;
            //
            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }
            //
            stageNum = nextStageNum;
        }
        //
        @Override
        public Mat processFrame(Mat input) {
            // We'll be updating this with new data below
            internalStoneList.clear();
            //
            /*
             * Run the image processing
             */
            for(MatOfPoint contour : findContours(input)) {
                analyzeContour(contour, input);
            }
            //
            clientStoneList = new ArrayList<>(internalStoneList);
            //
            /*
             * Decide which buffer to send to the viewport
             */
            switch (stages[stageNum]) {
                case Cb: {
                    return cbMat;
                }
                //
                case FINAL: {
                    return input;
                }
                //
                case MASK: {
                    return thresholdMat;
                }
                //
                case MASK_NR: {
                    return morphedThreshold;
                }
                //
                case CONTOURS: {
                    return contoursOnPlainImageMat;
                }
            }
            //
            return input;
        }
        //
        public ArrayList<StoneOrientationExample.StoneOrientationAnalysisPipeline.AnalyzedStone> getDetectedStones() {
            return clientStoneList;
        }
        //
        ArrayList<MatOfPoint> findContours(Mat input) {
            // A list we'll be using to store the contours we find
            ArrayList<MatOfPoint> contoursList = new ArrayList<>();
            //
            // Convert the input image to YCrCb color space, then extract the Cb channel
            Imgproc.cvtColor(input, cbMat, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(cbMat, cbMat, CB_CHAN_IDX);
            //
            // Threshold the Cb channel to form a mask, then run some noise reduction
            Imgproc.threshold(cbMat, thresholdMat, CB_CHAN_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY_INV);
            morphMask(thresholdMat, morphedThreshold);
            //
            // Ok, now actually look for the contours! We only look for external contours.
            Imgproc.findContours(morphedThreshold, contoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
            //
            // We do draw the contours we find, but not to the main input buffer.
            input.copyTo(contoursOnPlainImageMat);
            Imgproc.drawContours(contoursOnPlainImageMat, contoursList, -1, BLUE, CONTOUR_LINE_THICKNESS, 8);
            //
            return contoursList;
        }
        //
        void morphMask(Mat input, Mat output) {
            /*
             * Apply some erosion and dilation for noise reduction
             */
            //
            Imgproc.erode(input, output, erodeElement);
            Imgproc.erode(output, output, erodeElement);
            //
            Imgproc.dilate(output, output, dilateElement);
            Imgproc.dilate(output, output, dilateElement);
        }
        //
        void analyzeContour(MatOfPoint contour, Mat input) {
            // Transform the contour to a different format
            Point[] points = contour.toArray();
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            //
            // Do a rect fit to the contour, and draw it on the screen
            RotatedRect rotatedRectFitToContour = Imgproc.minAreaRect(contour2f);
            drawRotatedRect(rotatedRectFitToContour, input);
            //
            // The angle OpenCV gives us can be ambiguous, so look at the shape of
            // the rectangle to fix that.
            double rotRectAngle = rotatedRectFitToContour.angle;
            if (rotatedRectFitToContour.size.width < rotatedRectFitToContour.size.height) {
                rotRectAngle += 90;
            }
            //
            // Figure out the slope of a line which would run through the middle, lengthwise
            // (Slope as in m from 'Y = mx + b')
            double midlineSlope = Math.tan(Math.toRadians(rotRectAngle));
            //
            // We're going to split the this contour into two regions: one region for the points
            // which fall above the midline, and one region for the points which fall below.
            // We'll need a place to store the points as we split them, so we make ArrayLists
            ArrayList<Point> aboveMidline = new ArrayList<>(points.length/2);
            ArrayList<Point> belowMidline = new ArrayList<>(points.length/2);
            //
            // Ok, now actually split the contour into those two regions we discussed earlier!
            for(Point p : points)
            {
                if(rotatedRectFitToContour.center.y - p.y > midlineSlope * (rotatedRectFitToContour.center.x - p.x))
                {
                    aboveMidline.add(p);
                }
                else
                {
                    belowMidline.add(p);
                }
            }

            // Now that we've split the contour into those two regions, we analyze each
            // region independently.
            StoneOrientationExample.StoneOrientationAnalysisPipeline.ContourRegionAnalysis aboveMidlineMetrics = analyzeContourRegion(aboveMidline);
            StoneOrientationExample.StoneOrientationAnalysisPipeline.ContourRegionAnalysis belowMidlineMetrics = analyzeContourRegion(belowMidline);

            if(aboveMidlineMetrics == null || belowMidlineMetrics == null)
            {
                return; // Get out of dodge
            }

            // We're going to draw line from the center of the bounding rect, to outside the bounding rect, in the
            // direction of the side of the stone with the nubs.
            Point displOfOrientationLinePoint2 = computeDisplacementForSecondPointOfStoneOrientationLine(rotatedRectFitToContour, rotRectAngle);

            /*
             * If the difference in the densities of the two regions exceeds the threshold,
             * then we assume the stone is on its side. Otherwise, if the difference is inside
             * of the threshold, we assume it's upright.
             */
            if(aboveMidlineMetrics.density < belowMidlineMetrics.density - DENSITY_UPRIGHT_THRESHOLD)
            {
                /*
                 * Assume the stone is on its side, with the top contour region being the
                 * one which contains the nubs
                 */

                // Draw that line we were just talking about
                Imgproc.line(
                        input, // Buffer we're drawing on
                        new Point( // First point of the line (center of bounding rect)
                                rotatedRectFitToContour.center.x,
                                rotatedRectFitToContour.center.y),
                        new Point( // Second point of the line (center - displacement we calculated earlier)
                                rotatedRectFitToContour.center.x-displOfOrientationLinePoint2.x,
                                rotatedRectFitToContour.center.y-displOfOrientationLinePoint2.y),
                        PURPLE, // Color we're drawing the line in
                        2); // Thickness of the line we're drawing

                // We outline the contour region that we assumed to be the side with the nubs
                Imgproc.drawContours(input, aboveMidlineMetrics.listHolderOfMatOfPoint, -1, TEAL, 2, 8);

                // Compute the absolute angle of the stone
                double angle = -(rotRectAngle-90);

                // "Tag" the stone with text stating its absolute angle
                drawTagText(rotatedRectFitToContour, Integer.toString((int) Math.round(angle))+" deg", input);

                StoneOrientationExample.StoneOrientationAnalysisPipeline.AnalyzedStone analyzedStone = new StoneOrientationExample.StoneOrientationAnalysisPipeline.AnalyzedStone();
                analyzedStone.angle = angle;
                analyzedStone.orientation = StoneOrientationExample.StoneOrientationAnalysisPipeline.StoneOrientation.NOT_UPRIGHT;
                internalStoneList.add(analyzedStone);
            }
            else if(belowMidlineMetrics.density < aboveMidlineMetrics.density - DENSITY_UPRIGHT_THRESHOLD)
            {
                /*
                 * Assume the stone is on its side, with the bottom contour region being the
                 * one which contains the nubs
                 */

                // Draw that line we were just talking about
                Imgproc.line(
                        input, // Buffer we're drawing on
                        new Point( // First point of the line (center + displacement we calculated earlier)
                                rotatedRectFitToContour.center.x+displOfOrientationLinePoint2.x,
                                rotatedRectFitToContour.center.y+displOfOrientationLinePoint2.y),
                        new Point( // Second point of the line (center of bounding rect)
                                rotatedRectFitToContour.center.x,
                                rotatedRectFitToContour.center.y),
                        PURPLE, // Color we're drawing the line in
                        2); // Thickness of the line we're drawing

                // We outline the contour region that we assumed to be the side with the nubs
                Imgproc.drawContours(input, belowMidlineMetrics.listHolderOfMatOfPoint, -1, TEAL, 2, 8);

                // Compute the absolute angle of the stone
                double angle = -(rotRectAngle-270);

                // "Tag" the stone with text stating its absolute angle
                drawTagText(rotatedRectFitToContour,  Integer.toString((int) Math.round(angle))+" deg", input);

                StoneOrientationExample.StoneOrientationAnalysisPipeline.AnalyzedStone analyzedStone = new StoneOrientationExample.StoneOrientationAnalysisPipeline.AnalyzedStone();
                analyzedStone.angle = angle;
                analyzedStone.orientation = StoneOrientationExample.StoneOrientationAnalysisPipeline.StoneOrientation.NOT_UPRIGHT;
                internalStoneList.add(analyzedStone);
            }
            else
            {
                /*
                 * Assume the stone is upright
                 */

                drawTagText(rotatedRectFitToContour, "UPRIGHT", input);

                StoneOrientationExample.StoneOrientationAnalysisPipeline.AnalyzedStone analyzedStone = new StoneOrientationExample.StoneOrientationAnalysisPipeline.AnalyzedStone();
                analyzedStone.angle = rotRectAngle;
                analyzedStone.orientation = StoneOrientationExample.StoneOrientationAnalysisPipeline.StoneOrientation.UPRIGHT;
                internalStoneList.add(analyzedStone);
            }
        }
        //
        static class ContourRegionAnalysis {
            /*
             * This class holds the results of analyzeContourRegion()
             */

            double hullArea;
            double contourArea;
            double density;
            List<MatOfPoint> listHolderOfMatOfPoint;
        }
        //
        static StoneOrientationExample.StoneOrientationAnalysisPipeline.ContourRegionAnalysis analyzeContourRegion(ArrayList<Point> contourPoints) {
            // drawContours() requires a LIST of contours (there's no singular drawContour()
            // method), so we have to make a list, even though we're only going to use a single
            // position in it...
            MatOfPoint matOfPoint = new MatOfPoint();
            matOfPoint.fromList(contourPoints);
            List<MatOfPoint> listHolderOfMatOfPoint = Arrays.asList(matOfPoint);

            // Compute the convex hull of the contour
            MatOfInt hullMatOfInt = new MatOfInt();
            Imgproc.convexHull(matOfPoint, hullMatOfInt);

            // Was the convex hull calculation successful?
            if(hullMatOfInt.toArray().length > 0)
            {
                // The convex hull calculation tells us the INDEX of the points which
                // which were passed in eariler which form the convex hull. That's all
                // well and good, but now we need filter out that original list to find
                // the actual POINTS which form the convex hull
                Point[] hullPoints = new Point[hullMatOfInt.rows()];
                List<Integer> hullContourIdxList = hullMatOfInt.toList();

                for (int i = 0; i < hullContourIdxList.size(); i++)
                {
                    hullPoints[i] = contourPoints.get(hullContourIdxList.get(i));
                }

                StoneOrientationExample.StoneOrientationAnalysisPipeline.ContourRegionAnalysis analysis = new StoneOrientationExample.StoneOrientationAnalysisPipeline.ContourRegionAnalysis();
                analysis.listHolderOfMatOfPoint = listHolderOfMatOfPoint;

                // Compute the hull area
                analysis.hullArea = Imgproc.contourArea(new MatOfPoint(hullPoints));

                // Compute the original contour area
                analysis.contourArea = Imgproc.contourArea(listHolderOfMatOfPoint.get(0));

                // Compute the contour density. This is the ratio of the contour area to the
                // area of the convex hull formed by the contour
                analysis.density = analysis.contourArea / analysis.hullArea;

                return analysis;
            }
            else
            {
                return null;
            }
        }
        //
        static Point computeDisplacementForSecondPointOfStoneOrientationLine(RotatedRect rect, double unambiguousAngle) {
            // Note: we return a point, but really it's not a point in space, we're
            // simply using it to hold X & Y displacement values from the middle point
            // of the bounding rect.
            Point point = new Point();

            // Figure out the length of the short side of the rect
            double shortSideLen = Math.min(rect.size.width, rect.size.height);

            // We draw a line that's 3/4 of the length of the short side of the rect
            double lineLength = shortSideLen * .75;

            // The line is to be drawn at 90 deg relative to the midline running through
            // the rect lengthwise
            point.x = (int) (lineLength * Math.cos(Math.toRadians(unambiguousAngle+90)));
            point.y = (int) (lineLength * Math.sin(Math.toRadians(unambiguousAngle+90)));

            return point;
        }
        //
        static void drawTagText(RotatedRect rect, String text, Mat mat) {
            Imgproc.putText(
                    mat, // The buffer we're drawing on
                    text, // The text we're drawing
                    new Point( // The anchor point for the text
                            rect.center.x-50,  // x anchor point
                            rect.center.y+25), // y anchor point
                    Imgproc.FONT_HERSHEY_PLAIN, // Font
                    1, // Font size
                    TEAL, // Font color
                    1); // Font thickness
        }
        //
        static void drawRotatedRect(RotatedRect rect, Mat drawOn) {
            /*
             * Draws a rotated rect by drawing each of the 4 lines individually
             */

            Point[] points = new Point[4];
            rect.points(points);

            for(int i = 0; i < 4; ++i)
            {
                Imgproc.line(drawOn, points[i], points[(i+1)%4], RED, 2);
            }
        }
    }
    //
    public int getRings(){
        bestRing = ringPipe.getBestRing();
        //
        if(bestRing != null){
            if (bestRing.width > widthThresh) {
                if (bestRing.height > heightThresh) {
                    return 4;
                } else {
                    return 1;
                }
            } else {
                return 0;
            }
        }else{
            return 0;
        }
    }
    //</editor-fold>
    //
    //<editor-fold desc="Operations">
    public double getXOdometry(double xWheel, double yWheel, double lastAngle, double curAngle){
        //
        double[] storeOdo = getOdoData(xWheel, yWheel, lastAngle, curAngle);//calc hypotenuse and global angle
        //
        double total = storeOdo[0];//get hypotenuse/magnitude of offset
        double aGlob = storeOdo[1];//get global angle
        //
        if(xWheel == 0 && yWheel == 0){
            return 0;//return zero if no change
        }else {
            return total * Math.sin(Math.toRadians(aGlob));//recalc x change globally
        }
    }
    //
    public double getYOdometry(double xWheel, double yWheel, double lastAngle, double curAngle){
        //
        double[] storeOdo = getOdoData(xWheel, yWheel, lastAngle, curAngle);//calc hypotenuse and global angle
        //
        double total = storeOdo[0];//get hypotenuse/magnitude of offset
        double aGlob = storeOdo[1];//get global angle
        //
        if(xWheel == 0 && yWheel == 0){
            return 0;//return zero if no change
        }else {
            return total * Math.cos(Math.toRadians(aGlob));//recalc y change globally
        }
    }
    //
    public double[] getOdoData(double xWheel, double yWheel, double lastAngle, double curAngle){
        double cang = fixAngle(curAngle - lastAngle);//find change in angle
        double gRot = fixAngle(curAngle);
        //
        double xLoc = (xWheel / oconv) - getTurnInches(oxOffset, cang);//find x change in inches
        double yLoc = (yWheel / oconv) - getTurnInches(oyOffset, cang);//find y change in inches
        //
        double total = pythagorus(xLoc, yLoc);//get hypotenuse/magnitude of movement
        double aLoc = inverseTrigGyro(xLoc, yLoc, total);//find local angle of movement
        //
        double aGlob = aLoc + gRot;//adjust movement angle for global
        //
        double[] use = {total, aGlob};//package variables for function return
        //
        return use;

    }
    //
    public double getArcOdoX(int a, int b){
        double r1 = getArcOdoR(a, b);
        double x2 = (r1 + (R2 / 2)) * Math.cos(Math.toRadians(getArcOdoAngle(a, r1)));
        //
        return r1 + (R2 / 2) - x2;
    }
    //
    public double getArcOdoY(int a, int b){
        double r1 = getArcOdoR(a, b);
        double theta = getArcOdoAngle(a, r1);
        //
        return (r1 + (R2 / 2)) * Math.sin(theta);
    }
    //
    public double getArcOdoR(int a, int b){
        return R2 / ((a / b) - 1);
    }
    //
    public double getArcOdoAngle(int a, double r1){
        return 180 * a / (r1 * Math.PI);
    }
    //
    public double getTurnInches(double offset, double angleChange){
        return (angleChange / 360) * 4 * offset * Math.PI * tbias;// * 2//return inches wheels move when turning
    }
    //
    public double getBPower(double theta1, double Hfac){
        double rawb = (Math.sin(Math.toRadians(theta1)) - (Hfac * Math.cos(Math.toRadians(theta1)))) / -2;
        //
        double maxAngle = Math.atan(1 / -Hfac);
        double maxPower = (Math.sin(maxAngle) - (Hfac * Math.cos(maxAngle))) / -2;
        //
        return rawb * (1 / maxPower);//front right & back left
    }
    //
    public double getAPower(double theta1, double Hfac){
        double rawb = (Math.sin(Math.toRadians(theta1)) - (Hfac * Math.cos(Math.toRadians(theta1)))) / -2;
        double rawa = Hfac * Math.cos(Math.toRadians(theta1)) - rawb;
        //
        double maxAngle = Math.atan(1 / -Hfac);
        double maxPower = (Math.sin(maxAngle) - (Hfac * Math.cos(maxAngle))) / -2;
        //
        return rawa * (1 / maxPower);//front left & back right
    }
    //
    public double getDFarCoord(int side, double input){
        //sides: 0 is left, 1 is front, 2 is right
        Double doff = 0.0;
        if(side == 0){
            doff = ldoff;
        }else if(side == 1){
            doff = fdoff;
        }else{
            doff = rdoff;
        }
        //
        return (doff + input) * Math.cos(Math.toRadians(getAngle()));
    }
    public double getDCloseCoord(int side, double input){//sides: 0 is left, 1 is front, 2 is right
        //
        Double doff = 0.0;
        if(side == 0){
            doff = ldoff;
        }else if(side == 1){
            doff = fdoff;
        }else{
            doff = rdoff;
        }
        //
        return (doff + input) * Math.sin(Math.toRadians(getAngle()));
    }
    //
    public double getAngle(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return fixAngle(-angles.firstAngle - origin);
    }
    //
    public Float getRawGyro(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return -angles.firstAngle;
    }
    //
    public double pythagorus(double a, double b){
        return Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2));
    }
    //
    public double fixAngle(double angle){
        if(angle > 180){
            angle -= 360;
        }else if(angle < -180){
            angle += 360;
        }
        return angle;
    }
    //
    public double fixUnitAngle(double angle){
        if(angle > 360){
            angle -= 360;
        }else if(angle < 0){
            angle += 360;
        }
        return angle;
    }
    //
    public double inverseTrigMath(double x, double y, double h){
        double theta;
        double angleCore = Math.toDegrees(Math.acos(x / h));
        if(y > 0){
            theta = angleCore;
        }else{
            theta = -angleCore + 360;
        }
        return theta;
    }
    //
    public double inverseTrigGyro(double x, double y, double h){
        double theta;
        double angleCore = Math.toDegrees(Math.asin(x / h));
        if(y > 0){
            theta = angleCore;//uses arc sine because angle comes from the top rather than the side
        }else{
            theta = -angleCore + 180;
        }
        return theta;
    }
    //
    public boolean angleInBounds(double target, double error){
        //
        double rB = target + error;
        double lB = target - error;
        //
        double a = getAngle();
        //
        if (fixAngle(rB) != rB || fixAngle(lB) != lB){
            if ((lB < a && a < 180) || (-180 < a && a < rB)){
                return true;
            }else {
                return false;
            }
        }else{
            if (lB < getAngle() && getAngle() < rB){
                return true;
            }else {
                return false;
            }
        }
    }
    //
    public boolean inBounds(double value, double target, double error){
        return (target - error) < value && value < (target + error);
    }
    //
    public double filterfJS(double input){
        if(input < 100){
            fJSlast = input;
            return input;
        }else{
            return fJSlast;
        }
    }
    //
    public double filterlJS(double input){
        if(input < 100){
            lJSlast = input;
            return input;
        }else{
            return lJSlast;
        }
    }
    public double filterolJS(double input){
        if(input < 100){
            olJSlast = input;
            return input;
        }else{
            return olJSlast;
        }
    }
    public double filterrJS(double input){
        if(input < 100){
            rJSlast = input;
            return input;
        }else{
            return rJSlast;
        }
    }
    public double filterorJS(double input){
        if(input < 100){
            orJSlast = input;
            return input;
        }else{
            return orJSlast;
        }
    }
    //
    public void resetOrigin(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        origin = -angles.firstAngle;
    }
    //
    public double calcPower(double rpm, double rpmGoal){
        double curPower = launcher.getPower();
        double curError = rpmGoal - rpm;
        double deltaError = curError - lastError;
        //
        double output = curPower + (propConst * curError);
        if(Math.abs(deltaError) > 300){
            output += dampConst * (deltaError);
        }
        lastError = curError;
        return output;
    }
    //
    public void calcRPM(){
        deltaTime = System.currentTimeMillis() - lastTime;
        rpm = launchConv * (launcher.getCurrentPosition() - lastEnc) / deltaTime;
        lastTime = System.currentTimeMillis();
        lastEnc = launcher.getCurrentPosition();
        //
        rollingAvg += rpm / 6;
        rollingAvg -= rpmSamples.get(0) / 6;
        rpmSamples.remove(0);
        rpmSamples.add(rpm);
    }
    //</editor-fold>
    //
    //<editor-fold desc="Odometry">
    public int[] updateOdo(FtcDashboard dashboard, TelemetryPacket packet, int curX, int curYLeft){
        //
        int lastX = curX;
        int lastYLeft = curYLeft;
        //lastYRight = curYRight;
        double lastAngle = gcAngle;//store old values
        //
        curX = xEnc.getCurrentPosition();
        curYLeft = yEncLeft.getCurrentPosition();
        //curYRight = yEncRight.getCurrentPosition();
        gcAngle = getAngle();//retrieve new values
        //
        xPos += getXOdometry(curX - lastX, curYLeft - lastYLeft, lastAngle, gcAngle);
        yPos += getYOdometry(curX - lastX, curYLeft - lastYLeft, lastAngle, gcAngle);//run algorithm
        //
        double[] correction = wallTouchCorrect(xPos, yPos, gcAngle);//if running against a wall, use to correct position
        xPos = correction[0];
        yPos = correction[1];
        //
        /*packet = new TelemetryPacket();
        //
        packet.fieldOverlay()
                .setStroke("black")
                .fillRect(fUnits(FIELD_DIM_DIG / -2),fUnits(FIELD_DIM_DIG / 6),fUnits(FIELD_DIM_DIG),2);
        drawRobot(packet, xPos, yPos, gcAngle);
        //
        packet.put("angle", gcAngle);
        packet.put("xPos", xPos);
        packet.put("yPos", yPos);
        packet.put("curX", curX);
        packet.put("curYLeft", curYLeft);
        //packet.put("curYRight", curYRight);
        dashboard.sendTelemetryPacket(packet);*/
        //
        return new int[]{curX, curYLeft};
    }
    //
    public double fUnits(double inches){
        return inches * oconvCan;
    }
    //
    public Point rotatePoint(Point input, double angle){
        //
        double h = pythagorus(input.x, input.y);
        double a = inverseTrigMath(input.x, input.y, h);
        //
        double b = a - angle;//because gyro angle is opposite of mathematical angle
        //
        return new Point(h * Math.cos(Math.toRadians(b)),h * Math.sin(Math.toRadians(b)));
    }
    //
    public void drawRobot(TelemetryPacket packet, double xPos, double yPos, double angle){
        rotRect(packet, xPos, yPos, ROBOT_WIDTH, ROBOT_LENGTH, angle, "red");
        rotRect(packet, xPos + (Math.sin(Math.toRadians(angle)) * ((ROBOT_LENGTH / 2) - 1)), yPos + (Math.cos(Math.toRadians(angle)) * ((ROBOT_LENGTH / 2) - 1)), ROBOT_WIDTH, 2, angle, "blue");
    }
    //
    public void rotRect(TelemetryPacket packet, double xPos, double yPos, double width, double length, double angle, String color){
        //
        double w2 = width / 2;
        double l2 = length / 2;
        //
        double[] xCoords = {-w2, w2, w2, -w2};//unturned x coords
        double[] yCoords = {l2, l2, -l2, -l2};//unturned y coords
        //
        Point use;
        for (int i = 0; i < xCoords.length; i++){//loop through every point
            use = rotatePoint(new Point(xCoords[i], yCoords[i]), angle);//rotate point
            xCoords[i] = -fUnits(use.x + xPos);//convert to canvas coordinates & store in arrays
            yCoords[i] = fUnits(use.y + yPos);
        }
        //
        packet.fieldOverlay()
                .setStroke(color)
                .strokePolygon(yCoords, xCoords);
    }
    //
    public double getMidRotDist(double width, double length, double angle){
        return pythagorus(width / 2, length / 2) * Math.sin(Math.toRadians(angle + inverseTrigGyro(width, length, pythagorus(width, length))));
    }
    //
    public double[] wallTouchCorrect(double xPos, double yPos, double angle){
        //
        double g1 = Math.abs(getMidRotDist(ROBOT_WIDTH, ROBOT_LENGTH, angle));//calculate both distances to the midpoint of a rotated rectangle
        double g2 = Math.abs(getMidRotDist(ROBOT_WIDTH, ROBOT_LENGTH, -angle));//the longer distance will be longer in all directions
        //
        double g3;
        if(g1 > g2){//find the longer distance to the midpoint
            g3 = g1;
        }else{
            g3 = g1;
        }
        //
        if(xPos + g3 > FIELD_DIM_REAL / 2){//check positive x breach
            xPos = FIELD_DIM_REAL / 2 - g3;
        }else if (xPos - g3 < FIELD_DIM_REAL / -6){//check negative x breach
            xPos = FIELD_DIM_REAL / -6 + g3;
        }
        if(yPos + g3 > FIELD_DIM_REAL / 2){//check positive y breach
            yPos = FIELD_DIM_REAL / 2 - g3;
        }else if(yPos - g3 < FIELD_DIM_REAL / -2){//check negative breach
            yPos = FIELD_DIM_REAL / -2 + g3;
        }
        return new double[]{xPos, yPos};
    }
    //
    public void checkLineCross(double xPos, double yPos){
        //
        //rotatePoint()
        //
    }
    //
    public void newHopeProto(FtcDashboard dashboard, TelemetryPacket packet){
        //
        turnToAngleError(0, .1, 3);
        //
        odoYToPosition(dashboard, packet, 2.5, .2);
        //
        odoXToPosition(dashboard, packet, 26.5, .2);
        //
        turnToAngleError(0, .1, 10);
        //
    }
    //</editor-fold>
}
