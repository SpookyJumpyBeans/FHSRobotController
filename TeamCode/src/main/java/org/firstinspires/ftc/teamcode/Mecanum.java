package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Mecanum extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        DcMotor motorLift = hardwareMap.dcMotor.get("motorLift");
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Servo claw = hardwareMap.servo.get("claw");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        claw.setPosition(1);
        waitForStart();

        if (isStopRequested()) return;
        boolean level0 = false;
        boolean level1 = false;
        boolean level2 = false;
        boolean level3 = false;
        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            //Level 1
            if(gamepad2.a)
            {
                motorLift.setPower(0.5);
                level1 = true;
            }
            if(motorLift.getCurrentPosition()>=200 && level1){
                motorLift.setPower(0);
                level1 = false;
            }
            //Level 2
            if(gamepad2.b)
            {
                motorLift.setPower(0.5);
                level2 = true;
            }
            if(motorLift.getCurrentPosition()>=300 && level2){
                motorLift.setPower(0);
                level2 = false;
            }
            //Level 3
            if(gamepad2.y)
            {
                motorLift.setPower(0.5);
                level3 = true;
            }
            if(motorLift.getCurrentPosition()>=400 && level3){
                motorLift.setPower(0);
                level3 = false;
            }
            //Level 0
            if(gamepad2.x)
            {
                motorLift.setPower(-0.3);
                level0 = true;
            }
            if(motorLift.getCurrentPosition()<=0 && level0){
                motorLift.setPower(0);
                level0 = false;
            }

            //claw code for opening and closing
            if(gamepad1.right_bumper){
                claw.setPosition(0.75);
            } else if (gamepad1.left_bumper) {
                claw.setPosition(1);
            }

//            telemetry.addData("claw: ", claw.getPosition());
//            telemetry.addData("y: ", y);
//            telemetry.addData("rx: ", rx);
            telemetry.addData("lift: ", motorLift.getCurrentPosition());
            telemetry.update();
        }
    }
}