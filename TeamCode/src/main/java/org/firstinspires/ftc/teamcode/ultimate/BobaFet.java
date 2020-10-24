package org.firstinspires.ftc.teamcode.ultimate;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Boba Fet", group = "test")
public class BobaFet extends jeremy {
    //
    Boolean aPressed = false;
    Boolean intakeRunning = false;
    Boolean arrowPressed = false;
    //
    Double launcherPower = 0.2;
    //
    public void runOpMode() throws InterruptedException {
        //
        Init();
        //
        waitForStartify();
        //
        while(opModeIsActive()){
            //
            if(gamepad1.a && !aPressed){
                aPressed = true;
                intakeRunning = !intakeRunning;
                if(intakeRunning){
                    intake.setPower(1.0);
                }else{
                    intake.setPower(0.0);
                }
            }else if(!gamepad1.a && aPressed){
                aPressed = false;
            }
            //
            if((gamepad1.dpad_up || gamepad1.dpad_down) && !arrowPressed){
                arrowPressed = true;
                if(gamepad1.dpad_up && launcherPower < 1){
                    launcherPower += .05;
                }else if(launcherPower > 0){
                    launcherPower -= .05;
                }
                launcher.setPower(launcherPower);
            }else if(!gamepad1.dpad_up && !gamepad1.dpad_down && arrowPressed){
                arrowPressed = false;
            }
            //
            telemetry.addData("Launcher power", Math.round(100 * launcherPower) + "%");
            telemetry.addData("Intake running", intakeRunning);
            telemetry.update();
            //
        }
        //
    }
    //
}
