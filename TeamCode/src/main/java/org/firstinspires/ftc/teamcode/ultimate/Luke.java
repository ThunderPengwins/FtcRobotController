package org.firstinspires.ftc.teamcode.ultimate;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Luke", group = "working")
public class Luke extends jeremy {
    //
    double horiFactor = 1.1;
    //
    boolean aButton;
    boolean bButton;
    boolean yButton;
    boolean xButton;
    boolean dUp;
    boolean dDown;
    boolean bumber;
    float leftTrigger;
    float rightTrigger;
    //
    double ayaw = 0;
    boolean moveTurning = false;
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
            //
            aButton = gamepad1.a;
            bButton = gamepad2.b;
            yButton = gamepad1.y;
            xButton = gamepad2.x;
            dUp = gamepad2.dpad_up;
            dDown = gamepad2.dpad_down;
            bumber = gamepad2.left_bumper;
            leftTrigger = gamepad2.left_trigger;
            rightTrigger = gamepad2.right_trigger;
            //
            fullBaby(leftx, lefty, rightx, 0.5, 1.0);//all chassis motion
            //
            if(gamepad1.left_trigger > 0){//turn for launch
                turnToAngle(-8, .3);
            }
            //
            runIntake(aButton,yButton);
            //
            runFeed(bButton);
            //
            runLauncherIncr(dUp,dDown,bumber);//run launcher with arrows to increment by 5%
            //
            runWobble(leftTrigger, rightTrigger);
            //
            keepEmDead(xButton);//run wobble servo
            //
            telemetry.addData("Feed running", feedRunning);
            telemetry.addData("Stop pending", stopPending);
            telemetry.addData("Stop time", stopTime);
            telemetry.addData("Launcher power", Math.round(100 * launcherPower) + "%");
            telemetry.addData("Intake running", intakeRunning);
            telemetry.addData("Wobble up", wobbleUp.getState());
            telemetry.addData("Keeper position", keeper.getPosition());
            telemetry.update();
        }
        //
    }
    //
}
