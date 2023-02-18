package org.firstinspires.ftc.teamcode.drive;

//import com.google.blocks.ftcrobotcontroller.runtime.BNO055IMUAccess; Had imported, but was giving error
// kept it in just in case, because I (Isaiah) am not sure it was me who wanted it here

import static java.lang.Math.signum;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@TeleOp (name = "CookingWithGasEnhanced", group = "Iterative Opmode")
public class CookingWithGasEnhanced extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //Declare OpMode members
        ElapsedTime runtime = new ElapsedTime();
        ElapsedTime autoDropRequestTimer = new ElapsedTime();
        DcMotor FLDrive = null;
        DcMotor FRDrive = null;
        DcMotor BLDrive = null;
        DcMotor BRDrive = null;
        Servo clampyBoi = null;
        DcMotor STRAIGHTUPPPP = null;
        DistanceSensor junctionSensor = null;
        DistanceSensor centerDistanceSensor = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");
        ColorSensor colorSensor;
        boolean autoDropCone = false;
        double desiredHeading = 0;
        boolean liftAtDesiredPosition = false;

        //lift movement variables
        double initialLiftPosition;
        double currentLiftPosition;
        double desiredLiftPosition = 0;
        boolean autoPoiseLift = false;
        boolean autoStrikeLift = false;
        boolean autoRePoiseLift = false;
        boolean autoPickupOpenClip = false;
        boolean autoScoreOpenClip = false;
        boolean autoDropRequest = false;
        double ticksNeeded;
        boolean robotControlLift = false;
        double STRAIGHTUPPPPPower = 0;
        ElapsedTime timer = new ElapsedTime();

        telemetry.addData("Status", "Initializing");
        telemetry.update();

        FLDrive = hardwareMap.get(DcMotor.class, "FLDrive");
        FRDrive = hardwareMap.get(DcMotor.class, "FRDrive");
        BLDrive = hardwareMap.get(DcMotor.class, "BLDrive");
        BRDrive = hardwareMap.get(DcMotor.class, "BRDrive");
        clampyBoi = hardwareMap.get(Servo.class, "clampyBoi");
        STRAIGHTUPPPP = hardwareMap.get(DcMotor.class, "STRAIGHTUPPPP");
        junctionSensor = hardwareMap.get(DistanceSensor.class, "junctionSensor");
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

        FLDrive.setDirection(DcMotor.Direction.REVERSE);
        BLDrive.setDirection(DcMotor.Direction.REVERSE);
        FRDrive.setDirection(DcMotor.Direction.REVERSE);
        BRDrive.setDirection(DcMotor.Direction.REVERSE);
        clampyBoi.setDirection(Servo.Direction.FORWARD);
        STRAIGHTUPPPP.setDirection(DcMotor.Direction.REVERSE);

        STRAIGHTUPPPP.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        STRAIGHTUPPPP.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        STRAIGHTUPPPP.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        double yHeading = 0;
        double xHeading = -90;
        double bHeading = 90;
        double aHeading = 180;

        if (isStopRequested()) return;

        while (opModeIsActive()) { //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP

            if (colorSensor instanceof SwitchableLight) {
                ((SwitchableLight) colorSensor).enableLight(true);
            }

            double botHeadingDeg = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            STRAIGHTUPPPPPower = gamepad2.left_stick_y;

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

            boolean clawOpen = gamepad2.y;
            boolean slowMode = gamepad1.right_bumper;
            boolean dropCheck = (0.2 < (gamepad2.right_trigger));
            boolean slowSlide = (0.2 < (gamepad2.left_trigger));
            double speedMultiplier;
            initialLiftPosition = 0;
            currentLiftPosition = STRAIGHTUPPPP.getCurrentPosition();

            if (slowMode) {
                speedMultiplier = .5;
            } else {
                speedMultiplier = 1.0;
            }

            double rotate = botHeadingDeg - desiredHeading;
            rotate += 540;
            rotate = (rotate % 360) - 180;
            rx += rotate/-70;

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

            FLDrive.setPower(frontLeftPower * speedMultiplier);
            BLDrive.setPower(backLeftPower * speedMultiplier);
            FRDrive.setPower(frontRightPower * speedMultiplier);
            BRDrive.setPower(backRightPower * speedMultiplier);

            // MAIN SCORING CODE
            if (dropCheck) {
                if ((junctionSensor.getDistance(DistanceUnit.INCH) < 5) && (junctionSensor.getDistance(DistanceUnit.INCH)) > 3.5) {
                    autoDropCone = true;
                } else {
                    autoDropCone = false;
                }
            }


            if (autoDropCone) {
                clampyBoi.setPosition(.12);
            } else if (autoPickupOpenClip) { // for autoConePickup
                clampyBoi.setPosition(.12);
            } else if (autoScoreOpenClip) {
                clampyBoi.setPosition(.12);
            } else if (clawOpen) {
                clampyBoi.setPosition(.12);
            } else {
                clampyBoi.setPosition(.01);
            }

            if (slowSlide && (Math.abs(STRAIGHTUPPPPPower) > .1)) {
                STRAIGHTUPPPP.setPower(-STRAIGHTUPPPPPower * .5);
            } else if (Math.abs(STRAIGHTUPPPPPower) > .1) { // Small deadzone to counteract stick drift
                STRAIGHTUPPPP.setPower(-STRAIGHTUPPPPPower);
            } else {
                STRAIGHTUPPPP.setPower(0);
            }

            // Automatic pickup code:
            if (gamepad2.b) { // cancel button
                desiredLiftPosition = liftInchesToTicks(6);
                autoPoiseLift = false;
                autoStrikeLift = false;
                autoRePoiseLift = false;
                autoPickupOpenClip = false;
                autoScoreOpenClip = false;
            }
            if (gamepad2.a) { // initiate auto pickup
                desiredLiftPosition = liftInchesToTicks(6);
                autoPoiseLift = true;
            }
            if (autoPoiseLift) {
                autoPickupOpenClip = true;
                robotControlLift = true;
                if (centerDistanceSensor.getDistance(DistanceUnit.INCH) < 1.4 && liftAtDesiredPosition) {
                    autoPoiseLift = false;
                    desiredLiftPosition = liftInchesToTicks(0);
                    autoStrikeLift = true;
                    timer.reset();
                }
            }
            if (autoStrikeLift) {
                autoPickupOpenClip = false;
                if ((clampyBoi.getPosition() < .0109) && (timer.time() > .5) && liftAtDesiredPosition) {
                    autoStrikeLift = false;
                    robotControlLift = true;
                    desiredLiftPosition = liftInchesToTicks(6);
                    autoRePoiseLift = true;
                }
            }
            if (autoRePoiseLift) {
                robotControlLift = true;
                if(liftAtDesiredPosition){
                    autoRePoiseLift = false;
                }
            }
/*
            if (gamepad2.dpad_up){
                desiredLiftPosition = 6000;
                robotControlLift = true;
            }
            */

            currentLiftPosition = STRAIGHTUPPPP.getCurrentPosition();
            ticksNeeded = desiredLiftPosition - currentLiftPosition;

            if (Math.abs(ticksNeeded) > 20 && robotControlLift){
                STRAIGHTUPPPP.setPower(1 * signum(ticksNeeded));
                liftAtDesiredPosition = false;
            }else{
                liftAtDesiredPosition = true;
                robotControlLift = false;
            }

            double redVal = colorSensor.red();
            double greenVal = colorSensor.green();
            double blueVal = colorSensor.blue();
            double totalVal = redVal + greenVal + blueVal;
            double redPercent = redVal / totalVal;
            double greenPercent = greenVal / totalVal;
            double bluePercent = blueVal / totalVal;
            boolean seeingSilver;
            boolean seeingRed;
            boolean seeingBlue;
            if ((redPercent > .15) && (redPercent < .27) && (greenPercent > .35) && (greenPercent < .45) && (bluePercent > .32) && (bluePercent < .46)) {
                seeingSilver = true;
            } else {
                seeingSilver = false;
            }
            if (redPercent > greenPercent && redPercent > bluePercent) {
                seeingRed = true;
            } else {
                seeingRed = false;
            }
            if (bluePercent > redPercent && bluePercent > greenPercent) {
                seeingBlue = true;
            } else {
                seeingBlue = false;
            }
            if ((centerDistanceSensor.getDistance(DistanceUnit.INCH) > 0.35) && (centerDistanceSensor.getDistance(DistanceUnit.INCH) < 1.4) && (seeingRed || seeingBlue) && (autoDropRequest)) {//1.3
                autoScoreOpenClip = true;
                autoDropRequestTimer.reset();
                autoDropRequest = false;
            }
            if ((centerDistanceSensor.getDistance(DistanceUnit.INCH) < 1.5) && (centerDistanceSensor.getDistance(DistanceUnit.INCH) > 1) && (seeingSilver) && (autoDropRequest)) {
                autoScoreOpenClip = true;
                autoDropRequestTimer.reset();
                autoDropRequest = false;
            }
            if (autoScoreOpenClip && autoDropRequestTimer.time() > .5) {
                autoScoreOpenClip = false;
            }


// END OF MAIN SCORING CODE -----------------------------
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Red percent: ", redPercent);
            telemetry.addData("Green percent: ", greenPercent);
            telemetry.addData("Blue percent: ", bluePercent);
            telemetry.addData("Red value: ", redVal);
            telemetry.addData("Green value: ", greenVal);
            telemetry.addData("Blue value: ", blueVal);
            telemetry.addData("seeing silver? ", seeingSilver);
            telemetry.addData("seeingRed?", seeingRed);
            telemetry.addData("seeingBlue?", seeingBlue);
            telemetry.addData("Motors", "Front Left (%.2f), Front Right (%.2f), Back Left (%.2f), " + "Back Right (%.2f)", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.addData("Distance (in inches)", centerDistanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }


    public static double liftInchesToTicks(double inches){
        double inchesPerRev = (2 * Math.PI);
        double ticks_per_inch = (1120 / inchesPerRev);
        double ticks = inches * ticks_per_inch;
        return ticks;
    }
}
