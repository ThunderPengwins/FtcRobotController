package org.firstinspires.ftc.teamcode.ultimate;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Boba Fet", group = "test")
public class BobaFet extends jeremy {
    //
    public void runOpMode() throws InterruptedException {
        //
        Init();
        //
        waitForStartify();
        //
        while(opModeIsActive()){
            //
            runIntake();
            //
            runFeed();
            //
            runLauncherIncr();
            //
            telemetry.addData("Feed running", feedRunning);
            telemetry.addData("Stop pending", stopPending);
            telemetry.addData("Stop time", stopTime);
            telemetry.addData("Current time", System.currentTimeMillis());
            telemetry.addData("Time difference", Math.round(1000 * Math.ceil((System.currentTimeMillis() - feedStartTime) / 1000)));
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
