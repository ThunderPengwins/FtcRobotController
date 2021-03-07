package org.firstinspires.ftc.teamcode.ultimate;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Galen", group = "Auto")
public class Galen extends jeremy{
    //
    public void runOpMode(){
        //
        waitForStartify();
        //
        intake.setPower(1);
        intakefeed.setPower(1);
        sleep(300);
        //
        moveToPosition(-30, .3);
        sleep(1000);
        //
        intake.setPower(0);
        intakefeed.setPower(0);
        //
    }
}
