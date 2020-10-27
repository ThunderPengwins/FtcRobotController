package org.firstinspires.ftc.teamcode.ultimate;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "babyYoda", group = "test")
public class babyYoda extends jeremy{
    //
    boolean aPressed = false;
    //
    public void runOpMode(){
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
        }
        //
    }
    //
}
