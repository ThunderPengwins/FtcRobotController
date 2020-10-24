package org.firstinspires.ftc.teamcode.ultimate;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "horizontal Calibration", group = "setup")
public class horiCali extends jeremy {
    //
    Double globalBias = 1.0;
    Double turnBias = 0.5;
    Double horizontalBias = 1.1;
    //
    public void runOpMode(){
        //
        InitLite();
        //
        waitForStartify();
        //
        telemetry.addData("New", "thing");
        telemetry.update();
        //
        moveToPosition(30, .3);
        //
        telemetry.addData("Press a for", "0 degree movement - 30 inches");
        telemetry.update();
        //
        while(!gamepad1.a && opModeIsActive()){}
        //
        telemetry.addData("Moving at", "0 degrees");
        //
        meccToPosition(0,30,0,.3, globalBias, turnBias, horizontalBias);
        //
        telemetry.addData("Completed move at", "0 degrees");
        telemetry.addData("Press a for", "30 degree movement - 30 inches");
        telemetry.update();
        //
        while(!gamepad1.a && opModeIsActive()){}
        //
        telemetry.addData("Moving at", "30 degrees");
        telemetry.update();
        //
        meccToPosition(30,30,0,.3, globalBias, turnBias, horizontalBias);
        //
        telemetry.addData("Completed move at", "03 degrees");
        telemetry.addData("Press a for", "45 degree movement - 30 inches");
        telemetry.update();
        //
        while(!gamepad1.a && opModeIsActive()){}
        //
        telemetry.addData("Moving at", "45 degrees");
        telemetry.update();
        //
        meccToPosition(45,30,0,.3, globalBias, turnBias, horizontalBias);
        //
        telemetry.addData("Completed move at", "45 degrees");
        telemetry.addData("Press a for", "60 degree movement - 30 inches");
        telemetry.update();
        //
        while(!gamepad1.a && opModeIsActive()){}
        //
        telemetry.addData("Moving at", "60 degrees");
        telemetry.update();
        //
        meccToPosition(60,30,0,.3, globalBias, turnBias, horizontalBias);
        //
        telemetry.addData("Completed move at", "60 degrees");
        telemetry.addData("Press a for", "90 degree movement - 30 inches");
        telemetry.update();
        //
        while(!gamepad1.a && opModeIsActive()){}
        //
        telemetry.addData("Moving at", "90 degrees");
        telemetry.update();
        //
        meccToPosition(90,30,0,.3, globalBias, turnBias, horizontalBias);
        //
        telemetry.addData("Completed move at", "90 degrees");
        telemetry.addData("Calibration","complete");
        telemetry.update();
    }
    //
}
