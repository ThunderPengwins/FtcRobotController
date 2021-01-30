package org.firstinspires.ftc.teamcode.ultimate;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Leia", group="test")
public class Leia extends jeremy {
    //
    public void runOpMode(){
        //
        InitNoOpen();
        //
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        //
        waitForStartify();
        //
        while(opModeIsActive()){
            //
            packet.put("left x", getDFarCoord(0, leftJS.getDistance(DistanceUnit.INCH)));
            packet.put("front y", getDFarCoord(1, frontJS.getDistance(DistanceUnit.INCH)));
            packet.put("right x", getDFarCoord(2, rightJS.getDistance(DistanceUnit.INCH)));
            dashboard.sendTelemetryPacket(packet);
            //
        }
        //
    }
}
