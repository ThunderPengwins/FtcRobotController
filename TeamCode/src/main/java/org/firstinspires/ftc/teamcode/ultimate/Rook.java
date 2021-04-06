package org.firstinspires.ftc.teamcode.ultimate;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Rook", group = "working")
public class Rook extends jeremy{
    //
    public void runOpMode(){
        //
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        //
        packet.put("Initialization", "In progress...");
        dashboard.sendTelemetryPacket(packet);
        Init();
        //
        origin = getRawGyro();
        //
        packet.put("Initialization","Systems ready...");
        dashboard.sendTelemetryPacket(packet);
        //
        feed.setPosition(FEEDPULL);
        //
        int ringNum = getRings();
        packet.put("Initialization","Stabilizing OpenCV...");
        dashboard.sendTelemetryPacket(packet);
        sleep(1000);
        for (int n = 0; n < 10; n++){
            ringNum = getRings();
            packet.put("Initialization","Stabilizing OpenCV...");
            packet.put("rings", ringNum);
            dashboard.sendTelemetryPacket(packet);
            sleep(100);
        }
        packet.put("Initialization","Complete!");
        packet.put("rings", ringNum);
        dashboard.sendTelemetryPacket(packet);
        //
        waitForStartify();
        //
        moveToPosition(55,.3);
        //
        //<editor-fold desc="strafe with odometry">
        motorsWithEncoders();
        //
        int lastX = xEnc.getCurrentPosition();//set odo values
        int lastYLeft = yEncLeft.getCurrentPosition();
        int curX = lastX;
        int curYLeft = lastYLeft;
        origin = getRawGyro();
        gcAngle = getAngle();
        double lastAngle;
        //
        frontLeft.setPower(-.3);
        backLeft.setPower(.3);
        frontRight.setPower(.3);
        backRight.setPower(-.3);
        //
        while (!inBounds(xPos, -20, 3) && opModeIsActive()){
            calcRPM();
            launcher.setPower(calcPower(rollingAvg, psLauncherRPM));
            //
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
            packet.put("rpm", rollingAvg);
            packet.put("rpm goal", psLauncherRPM);
            dashboard.sendTelemetryPacket(packet);
        }
        still();
        //</editor-fold>
        //
        packet.put("ring","1");
        dashboard.sendTelemetryPacket(packet);
        //
        double rpm1 = 2475;
        turnToAngleErrorLauncher(5.5,.1,2, rpm1, packet, dashboard);//first turn (right)
        feed.setPosition(FEEDPUSH);
        sleepLauncher(1000, rpm1);
        //
        feed.setPosition(FEEDPULL);
        packet.put("ring","2");
        dashboard.sendTelemetryPacket(packet);
        //
        double rpm2 = 2400;
        turnToAngleErrorLauncher(0, .1,2, rpm2, packet, dashboard);//second turn (middle)
        sleepLauncher(500, rpm2);
        feed.setPosition(FEEDPUSH);
        sleepLauncher(1000, rpm2);
        //
        feed.setPosition(FEEDPULL);
        packet.put("ring","3");
        dashboard.sendTelemetryPacket(packet);
        //
        turnToAngleErrorLauncher(-6,.1,2, 2475, packet, dashboard);//last turn (left)
        sleepLauncher(500, 2475);
        feed.setPosition(FEEDPUSH);
        sleepLauncher(1000, 2475);
        launcher.setPower(0);
        //
        feed.setPosition(FEEDPULL);
        sleep(1000);
        //
        if(ringNum == 0){
            turnToAngleError(-95, .3, 5);
            //
            moveToPosition(-40, .5);
            //
            wobble.setPower(-0.4);
            sleep(700);
            wobble.setPower(0);
            keeper.setPosition(0);//set open
            kelper.setPosition(1);
            sleep(500);
            //
            moveToPosition(6, .6);
            //
            turnToAngleError(-15, .4, 5);
            //
            moveToPosition(-30, .6);
            sleep(500);
            //
            keeper.setPosition(1);//set closed
            kelper.setPosition(0);
            sleep(700);
            //
            turnWithGyro(10, .3);
            //
            moveToPosition(40, .5);
            //
            turnWithGyro(75, -.5);
            //
            keeper.setPosition(0);//set open
            kelper.setPosition(1);
            sleep(500);
            //
        }else if(ringNum == 1){
            turnToAngleError(-130, .3, 5);
            //
            moveToPosition(-33, .6);
            //
            wobble.setPower(-0.4);
            sleep(700);
            wobble.setPower(0);
            keeper.setPosition(0);//set open
            kelper.setPosition(1);
            sleep(500);
            //
            moveToPosition(6, .6);
            //
            intake.setPower(1);
            //
            turnWithGyro(110, .4);
            //
            intakefeed.setPower(1);
            //
            moveToPosition(-55, .5);
            //
            launcher.setPower(.97);
            //
            keeper.setPosition(1);//set closed
            kelper.setPosition(0);
            sleep(700);
            //
            moveToPosition(40, .5);
            //
            turnPast(-10, .3, true);
            turnToAngleError(0, .1, 2);
            //
            intake.setPower(0);
            intakefeed.setPower(0);
            //
            sleep(1000);
            //
            feed.setPosition(FEEDPUSH);
            sleep(500);
            feed.setPosition(FEEDPULL);
            sleep(500);
            feed.setPosition(FEEDPUSH);
            sleep(1000);
            launcher.setPower(0);
            feed.setPosition(FEEDPULL);
            //
            turnToAngle(-120, 1);
            //
            moveToPosition(-12, .8);
            //
            keeper.setPosition(0);//set open
            kelper.setPosition(1);
            sleep(500);
        }else{
            //
            turnToAngleError(-125, .3, 5);
            //
            moveToPosition(-60, .7);
            //
            wobble.setPower(-0.4);
            sleep(700);
            wobble.setPower(0);
            keeper.setPosition(0);//set open
            kelper.setPosition(1);
            sleep(500);
            //
            moveToPosition(5, .7);
            //
            turnToAngleError(-28, .6, 5);
            //
            moveToPosition(-30, .7);
            //
            turnToAngleError(8, .5, 5);
            //
            moveToPosition(-39, .6);
            //
            keeper.setPosition(1);//set closed
            kelper.setPosition(0);
            sleep(700);
            //
            turnToAngleError(3, .3, 5);
            //
            moveToPosition(80, .7);
            //
            turnWithGyro(100, -.5);
            //
            keeper.setPosition(0);//set open
            kelper.setPosition(1);
            sleep(500);
            //
            moveToPosition(10, .8);
            //
            turnWithGyro(30, -.6);
            //
            moveToPosition(20, .8);
            //
            //moveToPosition(35, .5);
        }
        //
        //moveToPosition(10,.7);
        //
        /*for(int i = 0; i < 3; i++) {
            sleep(900);
            feed.setPosition(FEEDPUSH);
            sleep(1000);
            feed.setPosition(FEEDPULL);
            if(i != 2) {
                strafeToPosition(-7.25, .3);
            }
        }
        launcher.setPower(0);*/
        //

    }
}
