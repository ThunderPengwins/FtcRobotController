package org.firstinspires.ftc.teamcode.ultimate;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Basic", group = "working")
public class Basic extends jeremy{
    //
    DcMotor left;
    DcMotor right;
    //
    Double motorPower = 1.0;
    Boolean togglePressed = false;
    //
    public void runOpMode(){
        //
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //
        waitForStartify();
        //
        while (opModeIsActive()) {
            //
            if (gamepad1.left_stick_y != 0 && gamepad1.right_stick_x != 0) {
                if(-gamepad1.left_stick_y > 0){
                    left.setPower((-gamepad1.left_stick_y / 2 + (2 * Math.ceil(gamepad1.right_stick_x / 2))) * motorPower);
                    right.setPower((-gamepad1.left_stick_y / 2 + (2 * Math.ceil(gamepad1.right_stick_x / -2))) * motorPower);
                }else{
                    left.setPower((-gamepad1.left_stick_y / 2 - (2 * Math.ceil(gamepad1.right_stick_x / -2))) * motorPower);
                    right.setPower((-gamepad1.left_stick_y / 2 - (2 * Math.ceil(gamepad1.right_stick_x / 2))) * motorPower);
                }
            } else if(gamepad1.left_stick_y != 0){
                left.setPower(-gamepad1.left_stick_y * motorPower);
                right.setPower(-gamepad1.left_stick_y * motorPower);
            } else{
                left.setPower(gamepad1.right_stick_x * motorPower);
                right.setPower(-gamepad1.right_stick_x * motorPower);
            }
            //
            if(gamepad1.left_bumper && !togglePressed){
                if(motorPower == 1.0){
                    motorPower = 0.5;
                }else {
                    motorPower = 1.0;
                }
                togglePressed = true;
            }else if(!gamepad1.left_bumper && togglePressed){
                togglePressed = false;
            }
            //
            telemetry.addData("left stick", -gamepad1.left_stick_y);
            telemetry.addData("motor power", motorPower);
            telemetry.update();
            //
        }
    }
    //
}
