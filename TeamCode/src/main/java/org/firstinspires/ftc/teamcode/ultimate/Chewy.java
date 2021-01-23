package org.firstinspires.ftc.teamcode.ultimate;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Chewy", group = "working")
public class Chewy extends jeremy {
    //
    public void runOpMode(){
        //
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //
        Init();
        telemetry.addData("Init", "complete");
        telemetry.update();
        //
        feed.setPosition(FEEDPULL);
        //
        waitForStartify();
        //
        int ringNum = getRings();
        telemetry.addData("Rings", ringNum);
        telemetry.update();
        if(ringNum == 0) {
            intake.setPower(-1);
            moveToPosition(62, .3);
            intake.setPower(0);
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
            moveToPosition(102, .3);
            //
            moveToPosition(-42, .3);
        }
        //
        launcher.setPower(.918);
        turnWithGyro(20, -.2);
        //
        sleep(1000);
        //
        for(int i = 0; i < 3; i++) {
            sleep(900);
            feed.setPosition(FEEDPUSH);
            sleep(900);
            feed.setPosition(FEEDPULL);
        }
        launcher.setPower(0);
        //
        wobble.setPower(-0.4);
        keeper.setPosition(0);//set open
        sleep(500);
        wobble.setPower(0);
        //
        turnWithGyro(55, .3);
        //
        moveToPosition(-39, .3);
        //
        keeper.setPosition(1.0);//set closed
        //
        sleep(700);
        //
        //wobble.setPower(1.0);
        //while (!wobbleUp.getState() && opModeIsActive()){}
        //sleep(900);
        //wobble.setPower(0);
        //
        turnWithGyro(190, .3);
        moveToPosition(-40, .3);
        //
        /*wobble.setPower(-0.4);
        sleep(500);
        wobble.setPower(0);*/
        //
        keeper.setPosition(0);//set open
        sleep(700);
        /*wobble.setPower(1.0);
        //while (!wobbleUp.getState() && opModeIsActive()){}
        sleep(900);
        wobble.setPower(0);*/
        //
        moveToPosition(10, .3);
        turnWithGyro(23, -.3);
        strafeToPosition(15, .3);
        moveToPosition(-15, .2);
        //
        /*if(ringNum == 0){
            strafeToPosition(-10, .3);
        }
        //
        moveToPosition(15, .2);*/
    }
    //
}