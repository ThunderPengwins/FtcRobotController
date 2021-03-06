package org.firstinspires.ftc.teamcode.ultimate;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Solo", group = "working")
public class Solo extends jeremy{
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
            moveToPosition(62, .3);
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
        launcher.setPower(.92);
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
        if(ringNum == 0){
            strafeToPosition(-10, .3);
        }
        //
        moveToPosition(15, .2);
    }
    //
}
