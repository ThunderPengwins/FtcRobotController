package org.firstinspires.ftc.teamcode.ultimate;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Erso", group = "test")
public class Erso extends jeremy{
    //
    DcMotor XEncoder;
    DcMotor YEncoder;
    //
    public void runOpMode(){
        //
        XEncoder = hardwareMap.dcMotor.get("xencoder");
        YEncoder = hardwareMap.dcMotor.get("yencoder");
        //
        InitLite();
        //
        int startX = XEncoder.getCurrentPosition();
        int startY = YEncoder.getCurrentPosition();
        //
        int curX = XEncoder.getCurrentPosition();
        int curY = YEncoder.getCurrentPosition();
        //
        double origin = getAngle();
        //
        waitForStartify();
        //
        while(opModeIsActive()){
            //
            telemetry.addData("X pos", getXOdometry(curX - startX, curY - startY, origin));
            telemetry.addData("Y pos", getYOdometry(curX - startX, curY - startY, origin));
            telemetry.update();
            //
        }
        //
    }
    //
}
