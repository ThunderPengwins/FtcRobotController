package org.firstinspires.ftc.teamcode.ultimate;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name = "Ben", group = "working")
public class Ben extends jeremy{
    //
    public void runOpMode(){
        //
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        //
        packet.put("Initialization", "In progress...");
        dashboard.sendTelemetryPacket(packet);
        Init();
        packet.put("Initialization","Complete");
        dashboard.sendTelemetryPacket(packet);
        //
        feed.setPosition(FEEDPULL);
        //
        waitForStartify();
        //
        moveToPosition(55,.3);
        /*moveWithEncoder(.3);
        //
        while(tape.red() < 80 && opModeIsActive()){}
        //
        still();
        //
        moveToPosition(-10, .2);*/
        //
        launcher.setPower(.825);//pslp = .85
        //
        strafeToPosition(-24, .3);
        //
        packet.put("ring","1");
        dashboard.sendTelemetryPacket(packet);
        //
        turnToAngleError(5.5,.1,2);//first turn (right)
        feed.setPosition(FEEDPUSH);
        sleep(1000);
        launcher.setPower(.81);
        //
        feed.setPosition(FEEDPULL);
        packet.put("ring","2");
        dashboard.sendTelemetryPacket(packet);
        //
        turnToAngleError(0, .1,2);//second turn (middle)
        sleep(500);
        feed.setPosition(FEEDPUSH);
        sleep(1000);
        launcher.setPower(.815);
        //
        feed.setPosition(FEEDPULL);
        packet.put("ring","3");
        dashboard.sendTelemetryPacket(packet);
        //
        turnToAngleError(-6,.1,2);//last turn (left)
        sleep(500);
        feed.setPosition(FEEDPUSH);
        sleep(1000);
        launcher.setPower(0);
        //
        feed.setPosition(FEEDPULL);
        sleep(1000);
        //
        turnToAngleError(-130, .3,5);
        //
        moveToPosition(-33, .5);
        //
        wobble.setPower(-0.4);
        sleep(500);
        wobble.setPower(0);
        keeper.setPosition(0);//set open
        kelper.setPosition(1);
        sleep(500);
        //
        moveToPosition(6, .5);
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
        /*wobble.setPower(.5);
        sleep(1000);
        wobble.setPower(0);*/
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
        sleep(1000);
        feed.setPosition(FEEDPULL);
        sleep(1000);
        feed.setPosition(FEEDPUSH);
        sleep(1000);
        launcher.setPower(0);
        //
        turnToAngle(-130, .8);
        //
        moveToPosition(-12, .6);
        //
        keeper.setPosition(0);//set open
        kelper.setPosition(1);
        sleep(500);
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
