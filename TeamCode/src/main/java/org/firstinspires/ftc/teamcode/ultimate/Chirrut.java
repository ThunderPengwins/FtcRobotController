package org.firstinspires.ftc.teamcode.ultimate;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous (name = "Chirrut", group = "test")
public class Chirrut extends jeremy {
    //
    public void runOpMode(){
        //
        InitNoOpen();
        //
        waitForStartify();
        //
        double distance = 14;
        //
        frontRight.setPower(-.3);
        backLeft.setPower(-.3);
        frontLeft.setPower(0.05);
        backRight.setPower(0.05);
        //
        motorsToPosition();
        //
        frontRight.setTargetPosition((int)(-distance * conversion));
        backLeft.setTargetPosition((int)(-distance * conversion));
        frontLeft.setTargetPosition((int)(distance / 6 * conversion));
        backRight.setTargetPosition((int)(distance / 6 * conversion));
        //
        while(frontRight.isBusy() && opModeIsActive()){}
        //
        still();
        //
        intake.setPower(1);
        intakefeed.setPower(1);
        //
        moveToPosition(-5, .3);
        //
        sleep(1000);
        //
        moveToPosition(-5, .3);
        //
        sleep(1000);
        //
        launcher.setPower(9.7);
        //
        moveToPosition(15, .3);
        //
        turnToAngleError(-10, .1, 3);
        //
        sleep(1000);
        //
        for(int i = 0; i < 4; i++) {
            feed.setPosition(FEEDPUSH);
            sleep(1000);
            feed.setPosition(FEEDPULL);
            sleep(1300);
        }
        launcher.setPower(0);
        //
        intake.setPower(0);
        intakefeed.setPower(0);
        //
    }
    //
}
