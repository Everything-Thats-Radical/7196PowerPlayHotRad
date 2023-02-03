package org.firstinspires.ftc.teamcode.drive;

import java.lang.Math.*;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp (name = "CopiedItV4", group = "Iterative Opmode")
public class CopiedItV4 extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("FLDrive");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("BLDrive");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("FRDrive");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("BRDrive");


        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        imu.resetYaw();

        waitForStart();

        //double startHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double desiredHeading = 0;
        double yHeading = 0;
        double xHeading = -90;
        double bHeading = 90;
        double aHeading = 180;

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double botHeadingDeg = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if(gamepad1.y){
                desiredHeading = yHeading;
            }
            if(gamepad1.x){
                desiredHeading = xHeading;
            }
            if(gamepad1.b){
                desiredHeading = bHeading;
            }
            if(gamepad1.a){
                desiredHeading = aHeading;
            }

            double rotate = botHeadingDeg - desiredHeading;
            rotate += 540;
            rotate = (rotate % 360) - 180;
            rx += rotate/-90;

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX - rx) / denominator;
            double backLeftPower = (rotY - rotX - rx) / denominator;
            double frontRightPower = (rotY - rotX + rx) / denominator;
            double backRightPower = (rotY + rotX + rx) / denominator;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            telemetry.addData("Bot Heading: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("Desired Heading: ", desiredHeading);
            //telemetry.addData("Turn direction: ", turnDirection);
            //telemetry.addData("Degrees to Turn: ", degreesToTurn);

            telemetry.addData("rx: ", rx);
            telemetry.update();
        }
    }
}