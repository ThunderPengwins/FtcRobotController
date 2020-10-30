package org.firstinspires.ftc.teamcode.ultimate;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Luke", group = "test")
public class Luke extends jeremy {
    //
    double powerFactor = 1.0;
    double horiFactor = 1.1;
    //
    float leftx;
    float lefty;
    float rightx;
    float otherlefty;
    float otherrighty;
    //
    double ayaw = 0;
    //
    double mT = 0;
    boolean ma = false;
    double mr = 0;
    //
    double origin = 0;
    //
    public void runOpMode() {
        //
        Init();
        //
        //backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        //
        waitForStartify();
        //
        while (opModeIsActive()){
            //
            if (gamepad1.a){
                origin = getAngle();
                //orchosen = true;
            }
            //
            leftx = gamepad1.left_stick_x;
            lefty = -gamepad1.left_stick_y;
            rightx = gamepad1.right_stick_x;
            otherlefty = -gamepad2.left_stick_y;
            otherrighty = -gamepad2.right_stick_y;
            //
            if(leftx == 0 && lefty == 0 && rightx == 0){//no motion
                //
                still();
                telemetry.addData("Motion","Still");
                //ayaw = getAngle();
                //setLight("red");
                //
            }else{//moving
                //
                telemetry.addData("Moving", "planetary");
                globalMeccMove(leftx, lefty, rightx, powerFactor, 0.5, horiFactor, origin);
                //
            }
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
            telemetry.addData("Launcher power", Math.round(100 * launcherPower) + "%");
            telemetry.addData("Intake running", intakeRunning);
            telemetry.update();
        }
        //
    }
    //
}
