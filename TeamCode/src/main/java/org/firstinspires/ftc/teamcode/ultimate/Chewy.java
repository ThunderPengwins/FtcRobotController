package org.firstinspires.ftc.teamcode.ultimate;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Chewy", group = "working")
public class Chewy extends jeremy {
    //
    public void runOpMode(){
        //
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //
        Init();
        //
        feed.setPosition(FEEDPULL);
        //
        /*int ringNum = getRings();
        telemetry.addData("Rings", ringNum);
        telemetry.update();
        sleep(100);*/
        //
        waitForStartify();
        //
        //Log.i("Program started", "////");
        //
        int ringNum = getRings();
        for(int i = 0; i < 10; i++){
            ringNum = getRings();
            sleep(50);
        }
        telemetry.addData("Rings", ringNum);
        telemetry.update();
        if(ringNum == 0) {
            //intake.setPower(-1);
            moveToPosition(62, .3);
            //intake.setPower(0);
            //
            wiggleToPosition(true, 20, .2);
            //
            wiggleToPosition(true, -20, .2);
            //
            moveToPosition(-2, .3);
        }else if(ringNum == 1){
            moveToPosition(82, .3);
            //
            wiggleToPosition(false, 20, .2);
            //
            wiggleToPosition(false, -15, .2);
            //
            moveToPosition(-27, .3);
        }else{
            //moveToPosition(102, .75);
            stageToPosition(110, .6, .4);
            //
            wiggleToPosition(true, 20, .2);
            //
            wiggleToPosition(true, -10, .2);
            turnToAngle(-3, .05);
            //
            //moveToPosition(-37, .75);
            //
            launcher.setPower(.918);
            /*moveWithEncoder(-.2);
            while(filterfJS(frontJS.getDistance(DistanceUnit.INCH )) < 62.5 && opModeIsActive()){
                //telemetry.addData("Distance", frontJS.getDistance(DistanceUnit.INCH));
                //Log.i("Distance", Double.toString(frontJS.getDistance(DistanceUnit.INCH)));
                //Log.i("Corrected Distance", Double.toString(filterfJS(frontJS.getDistance(DistanceUnit.INCH))));
                //telemetry.update();
            }
            still();*/
            //moveToPosition(-42, .3);
            stageToPosition(-58, .75, .3);
        }
        launcher.setPower(.918);
        if(ringNum == 0) {
            turnWithGyro(15, -.2);
        }else{
            //turnWithGyro(18, -.2);
            //turnPast(-15, .2, false);
            turnToAngle(-18, .2);
            if(ringNum == 1){
                intake.setPower(-1);
            }
        }
        //
        if(ringNum != 4) {
            sleep(1500);
        }
        //
        for(int i = 0; i < 3; i++) {
            sleep(900);
            feed.setPosition(FEEDPUSH);
            sleep(1000);
            feed.setPosition(FEEDPULL);
        }
        launcher.setPower(0);
        //
        wobble.setPower(-0.4);
        keeper.setPosition(0);//set open
        kelper.setPosition(1);
        sleep(500);
        wobble.setPower(0);
        //
        if(ringNum == 0) {
            turnWithGyro(50, .3);
            //
            moveToPosition(-37, .3);//go to wobble goal
            //
            keeper.setPosition(1.0);//set closed
            kelper.setPosition(0.0);
            //
            sleep(700);
            //
            turnWithGyro(190, .3);
            moveToPosition(-40, .3);//return from wobble goal
            //
            keeper.setPosition(0);//set open
            kelper.setPosition(1);
            sleep(700);
        }else if(ringNum == 1){
            //
            intake.setPower(-1);
            turnWithGyro(54, .3);
            //
            moveToPosition(-36, .3);//go to wobble goal
            //
            keeper.setPosition(1.0);//set closed
            kelper.setPosition(0.0);
            sleep(900);
            //
            /*wobble.setPower(.5);//pick up wobble goal
            sleep(1000);
            wobble.setPower(0);*/
            //
            intakefeed.setPower(1.0);
            turnWithGyro(157, .3);
            intake.setPower(0);
            intakefeed.setPower(0);
            //
            moveToPosition(-50, .5);//return from wobble goal
            //
            /*wobble.setPower(-0.4);//set down wobble goal
            sleep(500);
            wobble.setPower(0);*/
            //
            keeper.setPosition(0);//set open
            kelper.setPosition(1);
            sleep(700);
            //
            moveToPosition(5, .5);
            //
        }else{
            //
            turnToAngle(2, .1);
            //
            moveToPosition(-24, .3);
            //
            turnWithGyro(65, .3);
            //
            moveToPosition(-14, .3);//go to wobble goal
            //
            keeper.setPosition(1.0);//set closed
            kelper.setPosition(0.0);
            sleep(900);
            //
            /*wobble.setPower(.45);//pick up wobble goal
            sleep(1000);
            wobble.setPower(0);*/
            //
            moveToPosition(22, .4);
            //
            turnToAngle(180, .3);//turn towards zone
            //
            //stageToPosition(-64, 1, .6);
            moveToPosition(-60, 1);//go to place wobble
            sleep(500);
            //moveToPosition(-62, .9);
            //
            /*wobble.setPower(-0.4);
            sleep(500);
            wobble.setPower(0);*/
            //
            keeper.setPosition(0);//set open
            kelper.setPosition(1);
            sleep(800);
            //
            //turnPast(-150, .7, false);
            //
            moveToPosition(25, 1);
            //
        }
    }
    //
}