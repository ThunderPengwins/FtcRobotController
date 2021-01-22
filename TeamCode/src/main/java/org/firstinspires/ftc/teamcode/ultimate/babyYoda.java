package org.firstinspires.ftc.teamcode.ultimate;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;

@TeleOp(name = "babyYoda", group = "test")
public class babyYoda extends jeremy{
    //
    boolean aPressed = false;
    ArrayList<RingPipeline.AnalyzedRingGr> ringListSnap = new ArrayList<>();
    //
    public void runOpMode(){
        //
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //
        YodaInit();
        //
        waitForStartify();
        //
        while(opModeIsActive()){
            if(gamepad1.a && !aPressed){
                ringPipe.onViewportTapped();
                aPressed = true;
            }else if(!gamepad1.a && aPressed){
                aPressed = false;
            }
            bestRing = ringPipe.getBestRing();
            //
            if(bestRing != null){
                if (bestRing.width > widthThresh) {
                    if (bestRing.height > heightThresh) {
                        telemetry.addData("Rings", "four");
                    } else {
                        telemetry.addData("Rings", "one");
                    }
                } else {
                    telemetry.addData("Rings", "none");
                }
                telemetry.addData("Width", bestRing.width);
                telemetry.addData("Height", bestRing.height);
            }else{
                telemetry.addData("Rings", "none");
                telemetry.addData("Width", "n/a");
                telemetry.addData("Height", "n/a");
            }
            telemetry.addData("Stage", ringPipe.stages[ringPipe.stageNum]);
            telemetry.update();
        }
        //
    }
    //
}
