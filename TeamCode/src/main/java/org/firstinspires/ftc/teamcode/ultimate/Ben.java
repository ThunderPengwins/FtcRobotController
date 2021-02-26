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
        turnToAngleError(4,.15,2);//first turn (right)
        feed.setPosition(FEEDPUSH);
        sleep(1000);
        launcher.setPower(.81);
        //
        feed.setPosition(FEEDPULL);
        packet.put("ring","2");
        dashboard.sendTelemetryPacket(packet);
        //
        turnToAngleError(0, .15,3);//second turn (middle)
        sleep(500);
        feed.setPosition(FEEDPUSH);
        sleep(1000);
        launcher.setPower(.815);
        //
        feed.setPosition(FEEDPULL);
        packet.put("ring","3");
        dashboard.sendTelemetryPacket(packet);
        //
        turnToAngleError(-6,.15,3);//last turn (left)
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
        moveToPosition(-35, .3);
        //
        wobble.setPower(-0.4);
        sleep(500);
        wobble.setPower(0);
        keeper.setPosition(0);//set open
        kelper.setPosition(1);
        sleep(500);
        //
        moveToPosition(15, .2);
        //
        turnWithGyro(95, .3);
        //
        moveToPosition(55, .3);
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
