package org.firstinspires.ftc.teamcode.ultimate;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Leia", group="test")
public class Leia extends jeremy {
    //
    Double lJS = 0.0;
    Double fJS = 0.0;
    Double rJS = 0.0;
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
            lJS = filterlJS(leftJS.getDistance(DistanceUnit.INCH));
            fJS = filterfJS(frontJS.getDistance(DistanceUnit.INCH));
            rJS = filterrJS(rightJS.getDistance(DistanceUnit.INCH));
            //
            //packet.fieldOverlay().clear();
            packet.put("left filter", lJS);
            packet.put("front filter", fJS);
            packet.put("right filter", rJS);
            packet.put("left x", getDFarCoord(0, lJS));
            packet.put("front y", getDFarCoord(1, fJS));
            packet.put("right x", getDFarCoord(2, rJS));
            try {
                packet.fieldOverlay()
                        .setStroke("red")
                        .strokeRect(-20, -20, fJS, 40);
                packet.put("Canvas", "successful");
            }catch(Exception e){
                packet.put("Canvas", "failure: " + e);
            }//*/
            dashboard.sendTelemetryPacket(packet);
            //
        }
        //
    }
}
