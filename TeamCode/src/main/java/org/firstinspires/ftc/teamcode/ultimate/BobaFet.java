package org.firstinspires.ftc.teamcode.ultimate;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Boba Fet", group = "test")
public class BobaFet extends jeremy {
    //
    boolean aButton;
    boolean bButton;
    boolean yButton;
    boolean oyButton;
    boolean dUp;
    boolean dDown;
    boolean bumber;
    //
    public void runOpMode() throws InterruptedException {
        //
        Init();
        //
        waitForStartify();
        //
        while(opModeIsActive()){
            //
            aButton = gamepad1.a;
            bButton = gamepad1.b;
            yButton = gamepad1.y;
            oyButton = gamepad2.y;
            dUp = gamepad1.dpad_up;
            dDown = gamepad1.dpad_down;
            bumber = gamepad1.left_bumper;
            //
            runIntake(aButton, yButton, oyButton);
            //
            runFeed(bButton);
            //
            runLauncherIncr(dUp,dDown,bumber);
            //
            telemetry.addData("Feed running", feedRunning);
            telemetry.addData("Huh?", feedRunning && (System.currentTimeMillis() > feedStartTime + 1000));
            telemetry.addData("Stop time", feedStartTime + 1000);
            telemetry.addData("Current time", System.currentTimeMillis());
            telemetry.addData("Time difference", Math.round(500 * Math.ceil((System.currentTimeMillis() - feedStartTime) / 500)));
            telemetry.addData("Launcher power", Math.round(100 * launcherPower) + "%");
            telemetry.addData("Intake running", intakeRunning);
            telemetry.addData("Intake power", intakePower);
            telemetry.update();
            //
        }
        //
    }
    //
}
