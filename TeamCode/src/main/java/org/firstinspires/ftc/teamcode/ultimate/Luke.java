package org.firstinspires.ftc.teamcode.ultimate;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
    boolean firstdLeft;
    boolean firstdUp;
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
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
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
            firstdLeft = gamepad1.dpad_left;//start and stop intake feeder
            firstdUp = gamepad1.dpad_up;//reverse intake feeder direction
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
            runIntakeFeeder(firstdLeft, firstdUp);
            //
            runFeed(bButton);
            //
            runLauncherIncr(dUp,dDown,bumber);//run launcher with arrows to increment by 5%
            //
            runWobble(-leftTrigger, -rightTrigger);
            //
            keepEmDead(xButton);//run wobble servo
            //
            telemetry.addData("angle", getAngle());
            telemetry.addData("frontJS", filterfJS(frontJS.getDistance(DistanceUnit.INCH)));
            telemetry.addData("leftJS", leftJS.getDistance(DistanceUnit.INCH));
            telemetry.addData("rightJS", rightJS.getDistance(DistanceUnit.INCH));
            telemetry.addData("Feed running", feedRunning);
            telemetry.addData("Stop pending", stopPending);
            telemetry.addData("Stop time", stopTime);
            telemetry.addData("Launcher power", Math.round(100 * launcherPower) + "%");
            telemetry.addData("Intake running", intakeRunning);
            telemetry.addData("Intakefeed running", intakeFeederRunning);
            telemetry.addData("Intakefeed power", intakeFeederPower);
            telemetry.addData("Wobble up", wobbleUp.getState());
            telemetry.addData("Keeper position", keeper.getPosition());
            telemetry.update();
        }
        //
    }
    //
}
