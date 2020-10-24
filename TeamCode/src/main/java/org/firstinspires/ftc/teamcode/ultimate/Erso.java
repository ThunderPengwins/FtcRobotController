package org.firstinspires.ftc.teamcode.ultimate;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Erso", group = "test")
public class Erso extends jeremy{
    //
    public void runOpMode(){
        //
        InitLite();
        //
        waitForStartify();
        //
        while(opModeIsActive()){
            //
            //telemetry.addData("X pos", getXOdometry(x, y, a, origin));
            //telemetry.addData("Y pos", getYOdometry(x, y, a, origin));
            telemetry.update();
            //
        }
        //
    }
    //
}
