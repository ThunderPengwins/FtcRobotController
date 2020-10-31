package org.firstinspires.ftc.teamcode.ultimate;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Solo", group = "working")
public class Solo extends jeremy{
    //
    public void runOpMode(){
        //
        Init();
        //
        waitForStartify();
        //
        //motorsToPosition();
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
        turnWithGyro(20, -.2);
        //
        launcher.setPower(.95);
        sleep(2000);
        //
        feed.setPower(1.0);
        sleep(1400 * 4);
        feed.setPower(0);
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
