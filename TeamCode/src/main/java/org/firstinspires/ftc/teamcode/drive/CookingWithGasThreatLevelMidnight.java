package org.firstinspires.ftc.teamcode.drive;

import static java.lang.Math.signum;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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

@TeleOp (name = "CookingWithGasThreatLevelMidnight", group = "Iterative Opmode")
public class CookingWithGasThreatLevelMidnight extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //Declare OpMode members
        ElapsedTime runtime = new ElapsedTime(); // keep track of elapsed time. Not used except for telemetry
        ElapsedTime autoDropRequestTimer = new ElapsedTime();// Timer to keep track of movements with automatic pickup
        ElapsedTime clampyBoiMovementTimer = new ElapsedTime();
        // The claw servo is usually slower than we would like, so we add waits before the lift can move.
        DcMotor FLDrive = null; // standard motor declarations
        DcMotor FRDrive = null;
        DcMotor BLDrive = null;
        DcMotor BRDrive = null;
        Servo clampyBoi = null; // claw servo
        DcMotor STRAIGHTUPPPP = null; // lift motor
        DistanceSensor junctionSensor = null; // Side sensor in scooper. Not used.
        //centerDistanceSensor handles all distances and color for automatic pickup and the experimental automatic drop
        DistanceSensor centerDistanceSensor;
        ColorSensor colorSensor;
        boolean autoDropCone = false;
        double desiredHeading = 0;
        boolean liftAtDesiredPosition = false;

        //lift movement variables
        double currentLiftPosition;
        double desiredLiftPosition = 0;
        boolean autoPoiseLift = false;
        boolean autoStrikeLift = false;
        boolean autoRePoiseLift = false;
        boolean autoPickupOpenClip = false;
        boolean autoScoreOpenClip = false;
        boolean autoDropRequest = false;
        boolean robotControlLift = false;
        double liftTicksNeeded = 0;
        double STRAIGHTUPPPPPower = 0;
        int coneStackHeight = 0; // number of cones on stack
        boolean xReleased = true;
        double poiseHeight = 0;
        double strikeHeight = 0;
        double speedMultiplier;
        ElapsedTime timer = new ElapsedTime();
        boolean LiftSlowmode = gamepad2.right_bumper;

        telemetry.addData("Status", "Initializing");
        telemetry.update();

        FLDrive = hardwareMap.get(DcMotor.class, "FLDrive");
        FRDrive = hardwareMap.get(DcMotor.class, "FRDrive");
        BLDrive = hardwareMap.get(DcMotor.class, "BLDrive");
        BRDrive = hardwareMap.get(DcMotor.class, "BRDrive");
        clampyBoi = hardwareMap.get(Servo.class, "clampyBoi");
        STRAIGHTUPPPP = hardwareMap.get(DcMotor.class, "STRAIGHTUPPPP");
        junctionSensor = hardwareMap.get(DistanceSensor.class, "junctionSensor");
        centerDistanceSensor = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");
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

            double liftMult = 1;
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if(gamepad1.y){ // automatic turning commands
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

            if(LiftSlowmode){
                liftMult = .5;
            } else {
                liftMult = 1;
            }



            boolean clawOpen = gamepad2.y;
            boolean slowMode = gamepad1.right_bumper;
            if (slowMode) {
                speedMultiplier = .5;
            } else {
                speedMultiplier = 1.0;
            }

            double botHeadingDeg = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double rotate = botHeadingDeg - desiredHeading; // algorithm for automatic turning
            rotate += 540;
            rotate = (rotate % 360) - 180;
            rx += rotate/-70;

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); // bot heading for field centric
            // Rotate the movement direction counter to the bot's rotation
            // Changes x and y from robot centric drive to field-centric
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX - rx) / denominator; // standard mecanum wheel formulas
            double backLeftPower = (rotY - rotX - rx) / denominator;
            double frontRightPower = (rotY - rotX + rx) / denominator;
            double backRightPower = (rotY + rotX + rx) / denominator;

            FLDrive.setPower(frontLeftPower * speedMultiplier); // set power to wheels
            BLDrive.setPower(backLeftPower * speedMultiplier);
            FRDrive.setPower(frontRightPower * speedMultiplier);
            BRDrive.setPower(backRightPower * speedMultiplier);

            // claw handling code
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

            //TODO: add function where instead of handling driver's custom heights and the driver's selected
            // automatic heights separately, while the lift driver's joystick is controlling the lift, set the current
            // lift position as the desired position, so that when the driver lets go of the controller, the lift stays there.
            // The driver can adjust from there with more custom stuff, or can set the desired position based off of pre-made
            // heights for things like automatic pickup heights and different junction heights.

            if(xReleased && gamepad2.x){
                if(coneStackHeight < 5){
                    coneStackHeight++;
                }
                xReleased = false;
            } else if (!xReleased && !gamepad2.x){
                xReleased = true;
            }

            currentLiftPosition = STRAIGHTUPPPP.getCurrentPosition();
            if(!(gamepad2.left_stick_y == 0)){
                robotControlLift = false;
                autoPoiseLift = false;
                autoStrikeLift = false;
                autoRePoiseLift = false;
                autoPickupOpenClip = false;
                autoScoreOpenClip = false;
                STRAIGHTUPPPP.setPower((-gamepad2.left_stick_y));
                coneStackHeight=0;
            } else if(gamepad2.dpad_down || gamepad2.dpad_up || gamepad2.dpad_left || gamepad2.dpad_right){
                robotControlLift = true;
            } else{
                STRAIGHTUPPPP.setPower(0);
            }


            // Automatic pickup code
            currentLiftPosition = STRAIGHTUPPPP.getCurrentPosition();
            if (gamepad2.b) { // cancel button
                autoPoiseLift = false;
                autoStrikeLift = false;
                autoRePoiseLift = false;
                autoPickupOpenClip = false;
                autoScoreOpenClip = false;
                robotControlLift = false;
                coneStackHeight=0;
            }
            if (gamepad2.a) { // initiate auto pickup sequence
                switch(coneStackHeight){
                    case 0:
                    case 1:
                        strikeHeight = liftInchesToTicks(1.1);
                        poiseHeight = strikeHeight + liftInchesToTicks(6);
                        break;
                    case 2:
                        strikeHeight = liftInchesToTicks(2.4);
                        poiseHeight = strikeHeight + liftInchesToTicks(6);
                        break;
                    case 3:
                        strikeHeight = liftInchesToTicks(3.4);
                        poiseHeight = strikeHeight + liftInchesToTicks(6);
                        break;
                    case 4:
                        strikeHeight = liftInchesToTicks(5.3);
                        poiseHeight = strikeHeight + liftInchesToTicks(6);
                        break;
                    case 5:
                        strikeHeight = liftInchesToTicks(6.8);
                        poiseHeight = strikeHeight + liftInchesToTicks(6);
                        break;

                }
                autoPoiseLift = true; // start first auto pickup sequence
                robotControlLift = true;
            }
            if (autoPoiseLift) {
                desiredLiftPosition = poiseHeight; // tell height handler to start adjusting height
                autoPickupOpenClip = true; // open clip
                if (centerDistanceSensor.getDistance(DistanceUnit.INCH) < 1 && liftAtDesiredPosition && clampyBoi.getPosition() == 0.12) {
                    autoStrikeLift = true;
                    autoPoiseLift = false;
                    liftAtDesiredPosition = false;
                }
            }
            if (autoStrikeLift) {
                desiredLiftPosition = strikeHeight;
                if (liftAtDesiredPosition) {
                    autoPickupOpenClip = false;
                    autoRePoiseLift = true;
                    autoStrikeLift = false;
                    liftAtDesiredPosition = false;
                    clampyBoiMovementTimer.reset();
                }
            }
            if (autoRePoiseLift) {
                if(clampyBoiMovementTimer.seconds() > .2) {
                    desiredLiftPosition = poiseHeight;
                    if (liftAtDesiredPosition) {
                        autoRePoiseLift = false;
                        coneStackHeight = 0;
                    }
                }
            }

            if(gamepad2.dpad_down){
                desiredLiftPosition = liftInchesToTicks(3);
                autoPoiseLift = false;
                autoStrikeLift = false;
                autoRePoiseLift = false;
                autoPickupOpenClip = false;
                autoScoreOpenClip = false;
                coneStackHeight=0;
            }
            if(gamepad2.dpad_right){
                desiredLiftPosition = liftInchesToTicks(16);
                autoPoiseLift = false;
                autoStrikeLift = false;
                autoRePoiseLift = false;
                autoPickupOpenClip = false;
                autoScoreOpenClip = false;
                coneStackHeight=0;
            }
            if(gamepad2.dpad_left){
                desiredLiftPosition = liftInchesToTicks(26);
                autoPoiseLift = false;
                autoStrikeLift = false;
                autoRePoiseLift = false;
                autoPickupOpenClip = false;
                autoScoreOpenClip = false;
                coneStackHeight=0;
            }
            if(gamepad2.dpad_up){
                desiredLiftPosition = liftInchesToTicks(36);
                autoPoiseLift = false;
                autoStrikeLift = false;
                autoRePoiseLift = false;
                autoPickupOpenClip = false;
                autoScoreOpenClip = false;
                coneStackHeight=0;
            }

            if(robotControlLift){
                liftTicksNeeded = desiredLiftPosition - currentLiftPosition;
                if ((Math.abs(liftTicksNeeded)/200) > .1){ // .1 is the minimum power deliverable by the lift motor to still move the lift.
                    // essentially, if we know we can't move the lift, we don't.
                    STRAIGHTUPPPP.setPower((Math.abs(liftTicksNeeded)/200) * signum(liftTicksNeeded));
                }else{
                    STRAIGHTUPPPP.setPower(0);
                }
                if (Math.abs(liftTicksNeeded) > 20) { // change this to 30?
                    liftAtDesiredPosition = false;
                } else {
                    liftAtDesiredPosition = true;
                }
            }

            // color sensor handling
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

            // Show the elapsed game time and wheel power.

            /*telemetry.addData("Motors", "Front Left (%.2f), Front Right (%.2f), Back Left (%.2f), " + "Back Right (%.2f)", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.addData("gp2 left joystick y value: ", gamepad2.left_stick_y);
            telemetry.addData("Lift position: ", STRAIGHTUPPPP.getCurrentPosition());
            telemetry.addData("desired lift position: ", desiredLiftPosition);
            telemetry.addData("liftTicksNeeded: ", liftTicksNeeded);
            */
            telemetry.addData("distance sensed: ", centerDistanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("clampyBoi position: ", clampyBoi.getPosition());
            telemetry.addData("autoPoiseLift?: ", autoPoiseLift);
            telemetry.addData("autoStrikeLift?: ", autoStrikeLift);
            telemetry.addData("autoRePoiseLift? ", autoRePoiseLift);
            telemetry.addData("auto pose lift exit ticket ", (centerDistanceSensor.getDistance(DistanceUnit.INCH) < 1.4 && liftAtDesiredPosition && clampyBoi.getPosition() > 0.11));
            telemetry.addData("STRAIGHTUPPPP power ", STRAIGHTUPPPP.getPower());
            telemetry.addData("lift at desired position? ", liftAtDesiredPosition);
            telemetry.addData("liftPosition:  ", STRAIGHTUPPPP.getCurrentPosition());
            telemetry.addData("desired position:  ", desiredLiftPosition);
            telemetry.addData("ticks needed ", liftTicksNeeded);
            telemetry.addData("robot control lift? ", robotControlLift);
            telemetry.addData("x released? ", xReleased);
            telemetry.addData("coneStackHeight: ", coneStackHeight);
            telemetry.addData("poiseHeight: ", poiseHeight);
            telemetry.addData("strikeHeight: ", strikeHeight);

            //centerDistanceSensor.getDistance(DistanceUnit.INCH) < 1.4 && liftAtDesiredPosition && clampyBoi.getPosition() > 0.11
            telemetry.update();
        }
    }

    public static double liftInchesToTicks(double inches){
        int ticksPerInch = 180;
        return inches*ticksPerInch;
    }
}
