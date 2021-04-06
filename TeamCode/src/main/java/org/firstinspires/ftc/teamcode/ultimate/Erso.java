package org.firstinspires.ftc.teamcode.ultimate;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.lang.reflect.Array;
import java.util.ArrayList;

import static java.lang.Math.abs;
import static java.lang.Math.addExact;

@TeleOp(name = "Erso", group = "test")
public class Erso extends jeremy{
    //
    boolean aButton;
    boolean bButton;
    boolean yButton;
    boolean xButton;
    boolean oyButton;
    boolean dUp;
    boolean dDown;
    boolean leftBumper;
    boolean rightBumper;
    boolean firstdLeft;
    boolean firstdRight;
    boolean firstdUp;
    boolean firstdDown;
    float leftTrigger;
    float rightTrigger;
    //
    public void runOpMode(){
        //
        ArrayList<Double> rpmSamples = new ArrayList<>();
        for(int i = 0; i < 6; i++){
            rpmSamples.add(0.0);
        }
        //
        InitNoOpen();
        //
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lastEnc = launcher.getCurrentPosition();
        //
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        //
        /*int startX = xEnc.getCurrentPosition();
        int startYLeft = yEncLeft.getCurrentPosition();
        //int startYRight = yEncRight.getCurrentPosition();
        double startAngle = getRawGyro();*/
        //
        int lastX = xEnc.getCurrentPosition();
        int lastYLeft = yEncLeft.getCurrentPosition();
        int lastYRight = yEncRight.getCurrentPosition();
        int curX = lastX;
        int curYLeft = lastYLeft;
        //int curYRight = lastYRight;
        //
        //double origin = getAngle();
        gcAngle = getAngle();
        double lastAngle = gcAngle;
        //
        xPos = (FIELD_DIM_DIG / 2) - (ROBOT_WIDTH / 2) - 10;//-10 is temporary
        yPos = (FIELD_DIM_DIG / -2) + (ROBOT_LENGTH / 2) - 10;
        //
        double[] correction;
        //
        int autoMode = 0;
        boolean sleeping = false;
        double moveSpeed = 0.3;
        Long startTime = System.currentTimeMillis();
        double rpmGoal = 2700;//2400 for power shots
        boolean runLauncher = false;
        //
        packet.put("Initialization", "complete");
        dashboard.sendTelemetryPacket(packet);
        waitForStartify();
        //
        while(opModeIsActive()){
            //
            if (gamepad1.a){
                origin = getRawGyro();
            }
            //
           /* if(gamepad1.x){
                newHopeProto(dashboard, packet);
                //
                curX = xEnc.getCurrentPosition();
                curYLeft = yEncLeft.getCurrentPosition();
                //
            }*/
            //
            switch (autoMode){
                case 0://TeleOp
                    //
                    leftx = gamepad1.left_stick_x;
                    lefty = -gamepad1.left_stick_y;
                    rightx = gamepad1.right_stick_x;
                    //
                    aButton = gamepad1.b;
                    bButton = gamepad2.b;//gamepad 2
                    yButton = gamepad1.y;
                    oyButton = gamepad2.y;
                    dUp = gamepad2.dpad_up;
                    dDown = gamepad2.dpad_down;
                    leftBumper = gamepad2.left_bumper;//gamepad 2
                    rightBumper = gamepad2.right_bumper;
                    firstdLeft = gamepad1.dpad_left;//start and stop intake feeder
                    firstdUp = gamepad1.dpad_up;//reverse intake feeder direction
                    //
                    fullBaby(leftx, lefty, rightx, 0.5, 1.0);
                    //
                    runIntake(aButton,yButton,oyButton);
                    //
                    runIntakeFeeder(firstdLeft,firstdUp);
                    //
                    runFeed(bButton);
                    //
                    //runLauncherIncr(dUp,dDown,leftBumper,rightBumper);
                    //
                    packet = new TelemetryPacket();
                    //
                    packet.fieldOverlay()
                            .setStroke("black")
                            .fillRect(fUnits(FIELD_DIM_DIG / -2),fUnits(FIELD_DIM_DIG / 6),fUnits(FIELD_DIM_DIG),2);
                    drawRobot(packet, xPos, yPos, gcAngle);
                    //
                    calcRPM();
                    //
                    if(dUp) {
                        runLauncher = true;
                    }else if(dDown){
                        runLauncher = false;
                    }
                    if(runLauncher){
                        launcher.setPower(calcPower(rollingAvg, rpmGoal));
                    }else{
                        launcher.setPower(0);
                    }
                    //
                    packet.put("autoMode", autoMode);
                    //packet.put("Time", System.currentTimeMillis());
                    //packet.put("Start time", startTime);
                    //packet.put("angle", gcAngle);
                    //packet.put("xPos", xPos);
                    //packet.put("yPos", yPos);
                    //packet.put("curX", curX);
                    //packet.put("curYLeft", curYLeft);
                    packet.put("rpm", rpm);
                    packet.put("rpm average", rollingAvg);
                    packet.put("rpm error", rpmGoal - rpm);
                    packet.put("power", launcher.getPower());
                    packet.put("calcPower", calcPower(rollingAvg, rpmGoal));
                    packet.put("deltaTime", deltaTime);
                    packet.put("lastEnc", lastEnc);
                    //packet.put("curYRight", curYRight);
                    dashboard.sendTelemetryPacket(packet);
                    //
                    if(gamepad1.x){
                        if(gcAngle > 0) {
                            babyTurn(-.2, 1);
                        }else{
                            babyTurn(.2, 1);
                        }
                        autoMode = 1;
                    }
                    //
                    break;
                case 1://Turn to angle
                    //
                    if(angleInBounds(0, 7) && !sleeping){//wait for turn ready
                        still();
                        sleeping = true;
                        startTime = System.currentTimeMillis();
                    }else if(sleeping && System.currentTimeMillis() > startTime + 500){//wait 0.5 seconds
                        if(yPos > 2.5) {
                            frontLeft.setPower(-moveSpeed);
                            backLeft.setPower(-moveSpeed);
                            frontRight.setPower(-moveSpeed);
                            backRight.setPower(-moveSpeed);
                        }else{
                            frontLeft.setPower(moveSpeed);
                            backLeft.setPower(moveSpeed);
                            frontRight.setPower(moveSpeed);
                            backRight.setPower(moveSpeed);
                        }
                        //launcherToggle = true;
                        //launcher.setPower(launcherPower);
                        sleeping = false;
                        autoMode = 2;
                    }
                    //
                    break;
                case 2://Move to Y pos
                    //
                    if(inBounds(yPos, 1, 4) && !sleeping){//wait for y in bounds
                        still();
                        sleeping = true;
                        startTime = System.currentTimeMillis();
                    }else if(sleeping && System.currentTimeMillis() > startTime + 500){//wait 0.5 seconds
                        if (xPos < 26.5) {
                            frontLeft.setPower(moveSpeed);
                            backLeft.setPower(-moveSpeed);
                            frontRight.setPower(-moveSpeed);
                            backRight.setPower(moveSpeed);
                        }else{
                            frontLeft.setPower(-moveSpeed);
                            backLeft.setPower(moveSpeed);
                            frontRight.setPower(moveSpeed);
                            backRight.setPower(-moveSpeed);
                        }
                        sleeping = false;
                        autoMode = 3;
                    }
                    //
                    break;
                case 3://Move to X pos
                    //
                    if(inBounds(xPos, 26.5, 4) && !sleeping){
                        still();
                        sleeping = true;
                        startTime = System.currentTimeMillis();
                    }else if(sleeping && System.currentTimeMillis() > startTime + 500){
                        if(gcAngle > 0) {
                            babyTurn(-.2, 1);
                        }else{
                            babyTurn(.2, 1);
                        }
                        sleeping = false;
                        autoMode = 4;
                    }
                    //
                    break;
                case 4://Turn to angle again
                    //
                    if(angleInBounds(0, 10) && !sleeping){
                        still();
                        autoMode = 0;
                    }
                    //
                    break;
            }
            //
            lastX = curX;
            lastYLeft = curYLeft;
            //lastYRight = curYRight;
            lastAngle = gcAngle;//store old values
            //
            curX = xEnc.getCurrentPosition();
            curYLeft = yEncLeft.getCurrentPosition();
            //curYRight = yEncRight.getCurrentPosition();
            gcAngle = getAngle();//retrieve new values
            //
            xPos += getXOdometry(curX - lastX, curYLeft - lastYLeft, lastAngle, gcAngle);
            yPos += getYOdometry(curX - lastX, curYLeft - lastYLeft, lastAngle, gcAngle);//run algorithm
            //
            correction = wallTouchCorrect(xPos, yPos, gcAngle);//if running against a wall, use to correct position
            xPos = correction[0];
            yPos = correction[1];
            //
        }
        //
    }
    //
}
