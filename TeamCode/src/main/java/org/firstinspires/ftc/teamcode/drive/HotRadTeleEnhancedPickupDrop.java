package org.firstinspires.ftc.teamcode.drive;

//import com.google.blocks.ftcrobotcontroller.runtime.BNO055IMUAccess; Had imported, but was giving error
// kept it in just in case, because I (Isaiah) am not sure it was me who wanted it here

import static java.lang.Math.signum;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp (name = "HotRadTeleEnhancedPickupDrop", group = "Iterative Opmode")
public class HotRadTeleEnhancedPickupDrop extends LinearOpMode {
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

        //lift movement variables
        double initialLiftPosition;
        double currentLiftPosition;
        double desiredLiftPosition = 0;
        boolean autoPoiseLift = false;
        boolean autoStrikeLift = false;
        boolean autoRePoiseLift = false;
        boolean autoOpenClip = false;
        boolean autoDropRequest = false;
        ElapsedTime timer = new ElapsedTime();
/*
// Declare time keeper at the start of your opmode.
public ElapsedTime mRuntime = new ElapsedTime();


// Reset the game clock to zero in Start()
mRunTime.reset()


// Check for 2.5 seconds elapsed in loop();
if (mRunTime.time() > 2.5)
{
do stuff.
}
 */

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


        // Retrieve the IMU from the hardware map
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        runtime.reset();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) { //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP

            if (colorSensor instanceof SwitchableLight) {
                ((SwitchableLight) colorSensor).enableLight(true);
            }

            boolean clawOpen = gamepad2.y;
            boolean slowMode = gamepad1.right_bumper;
            boolean dropCheck = (0.2 < (gamepad2.right_trigger));
            boolean slowSlide = (0.2 < (gamepad2.left_trigger));
            boolean autoUp = gamepad2.left_stick_button;
            double speedMultiplier;
            initialLiftPosition = 0;
            currentLiftPosition = STRAIGHTUPPPP.getCurrentPosition();

            //Retrieve driving values from controller
            int word = 6;
            double y = gamepad1.left_stick_y * .8; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * .8; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x * .8;
            double STRAIGHTUPPPPPower = gamepad2.left_stick_y;

            if (gamepad2.x){
                autoDropRequest = true;
            }




// START OF MAIN DRIVING CODE ---------------------------------------------------------------------------------------------------
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x - rx) / denominator;
            double backLeftPower = (y - x - rx) / denominator;
            double frontRightPower = (y - x + rx) / denominator;
            double backRightPower = (y + x + rx) / denominator;

            // set power to motors
            if (slowMode) {
                speedMultiplier = .5;
            } else {
                speedMultiplier = 1.0;
            }

            FLDrive.setPower(frontLeftPower * speedMultiplier);
            FRDrive.setPower(frontRightPower * speedMultiplier);
            BLDrive.setPower(backLeftPower * speedMultiplier);
            BRDrive.setPower(backRightPower * speedMultiplier);

// END OF MAIN DRIVING CODE --------------------------------------------------------------------------------------------------

// START OF MAIN SCORING CODE --------------------------
            if (dropCheck) {
                if ((junctionSensor.getDistance(DistanceUnit.INCH) < 5) && (junctionSensor.getDistance(DistanceUnit.INCH)) > 3.5) {
                    autoDropCone = true;
                } else {
                    autoDropCone = false;
                }
            }

// Control servo
            if (autoDropCone) {
                clampyBoi.setPosition(.12);
            } else if (autoOpenClip){ // for autoConePickup
                clampyBoi.setPosition(.12);
            }
            else if (clawOpen) {
                clampyBoi.setPosition(.12);
            } else {
                clampyBoi.setPosition(.01);
            }
// End of control for servo

            if (slowSlide && (Math.abs(STRAIGHTUPPPPPower) > .1)){
                STRAIGHTUPPPP.setPower(-STRAIGHTUPPPPPower * .5);
            }
            else if(Math.abs(STRAIGHTUPPPPPower) > .1){ // Small deadzone to counteract stick drift
                STRAIGHTUPPPP.setPower(-STRAIGHTUPPPPPower);
            } else {
                STRAIGHTUPPPP.setPower(0);
            }

            if(autoUp){
                moveLift("up", 30, 1, STRAIGHTUPPPP);
            }
            telemetry.addData("Lift EncoderPosition: ", STRAIGHTUPPPP.getCurrentPosition());
            if (gamepad2.b){
                desiredLiftPosition = liftInchesToTicks(6);
                autoPoiseLift = false;
                autoStrikeLift = false;
                autoRePoiseLift = false;
                autoOpenClip = false;
            }
            if (gamepad2.a){
                desiredLiftPosition = liftInchesToTicks(6);
                autoPoiseLift = true;
            }

            //Automatic pickup code:
            if (autoPoiseLift){
                autoOpenClip = true;
                currentLiftPosition = STRAIGHTUPPPP.getCurrentPosition();
                double ticksNeeded = desiredLiftPosition - currentLiftPosition;
                if (Math.abs(ticksNeeded) > 100) {
                    if(Math.abs(ticksNeeded) < 100) {
                        STRAIGHTUPPPP.setPower(.5 * signum(ticksNeeded));
                    }else{
                        STRAIGHTUPPPP.setPower(1 * signum(ticksNeeded));
                    }
                } else if (centerDistanceSensor.getDistance(DistanceUnit.INCH) < 1.4) {
                    autoPoiseLift = false;
                    desiredLiftPosition = liftInchesToTicks(0);
                    autoStrikeLift = true;
                    timer.reset();
                }
            }

            if (autoStrikeLift){
                currentLiftPosition = STRAIGHTUPPPP.getCurrentPosition();
                double ticksNeeded = desiredLiftPosition - currentLiftPosition;
                if (Math.abs(ticksNeeded) > 100) {
                    if(Math.abs(ticksNeeded) < 100) {
                        STRAIGHTUPPPP.setPower(.5 * signum(ticksNeeded));
                    }else{
                        STRAIGHTUPPPP.setPower(1 * signum(ticksNeeded));
                    }
                } else{
                    autoOpenClip = false;
                    if((clampyBoi.getPosition() < .0109) && (timer.time() > .5)){
                        autoStrikeLift = false;
                        desiredLiftPosition = liftInchesToTicks(6);
                        autoRePoiseLift = true;
                    }
                }
            }

            if (autoRePoiseLift){
                currentLiftPosition = STRAIGHTUPPPP.getCurrentPosition();
                double ticksNeeded = desiredLiftPosition - currentLiftPosition;
                if (Math.abs(ticksNeeded) > 100) {
                    if(Math.abs(ticksNeeded) < 100) {
                        STRAIGHTUPPPP.setPower(.5 * signum(ticksNeeded));
                    }else{
                        STRAIGHTUPPPP.setPower(1 * signum(ticksNeeded));
                    }
                }else{
                    autoRePoiseLift = false;
                }
            }
            // end of auto pickup code

            double redVal = colorSensor.red();
            double greenVal = colorSensor.green();
            double blueVal = colorSensor.blue();

            double totalVal = redVal + greenVal + blueVal;

            double redPercent = redVal / totalVal;
            double greenPercent = greenVal / totalVal;
            double bluePercent = blueVal / totalVal;

            boolean seeingSilver = false;
            boolean seeingRed = false;
            boolean seeingBlue = false;

            if ((redPercent > .15) && (redPercent < .27) && (greenPercent > .35) && (greenPercent < .45) && (bluePercent > .32) && (bluePercent < .46)){
                seeingSilver = true;
            }else{
                seeingSilver = false;
            }

            if (redPercent > greenPercent && redPercent > bluePercent){
                seeingRed = true;
            }else{
                seeingRed = false;
            }

            if (bluePercent > redPercent && bluePercent > greenPercent){
                seeingBlue = true;
            }else{
                seeingBlue = false;
            }

            if ((junctionSensor.getDistance(DistanceUnit.INCH) > 2.6) && (junctionSensor.getDistance(DistanceUnit.INCH) < 2.4) && (seeingRed || seeingBlue) && (autoDropRequest)){
                autoOpenClip = true;
                autoDropRequestTimer.reset();
                autoDropRequest = false;
            }
            if((junctionSensor.getDistance(DistanceUnit.INCH) < 6.5) && (junctionSensor.getDistance(DistanceUnit.INCH) > 4.5) && (seeingSilver) && (autoDropRequest)){
                autoOpenClip = true;
                autoDropRequestTimer.reset();
                autoDropRequest = false;
            }
            if(autoOpenClip && autoDropRequestTimer.time() > .5){
                autoOpenClip = false;
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
            telemetry.addData("Motors", "Front Left (%.2f), Front Right (%.2f), Back Left (%.2f), " +
                            "Back Right (%.2f)", frontLeftPower, frontRightPower, backLeftPower,
                    backRightPower);
            telemetry.addData("Distance (in inches)", junctionSensor.getDistance(DistanceUnit.INCH));
            //telemetry.addData("Slides", "left (%.2f), right (%.2f)", x, y);

            telemetry.update();
        }
    }

    public void moveLift(String direction, double height, double power, DcMotor up) {
        double revsToInch = (2 * Math.PI);
        double ticks_per_inch = (1120 / revsToInch); //TICKS PER INCH MAY BE INCORRECT
        // THE LIFT WENT HIGHER THAN EXPECTED LAST TIME THIS WAS RUN AND BROKE THE LIFT
        // FIND ACTUAL TICKS PER INCH BEFORE RUNNING
        double ticksNeeded = height * ticks_per_inch;
        double initialPosition = up.getCurrentPosition();
        double currentPosition = up.getCurrentPosition();
        int directionSign = 1;
        double ticksMoved;

        if (direction.equals("up")) {
            directionSign = 1;
        } else if (direction.equals("down")) {
            directionSign = -1;
        } else {
            directionSign = -1;
        }

        ticksMoved = Math.abs(initialPosition - currentPosition);
        while (ticksNeeded > ticksMoved) {
            currentPosition = up.getCurrentPosition();
            ticksMoved = Math.abs(initialPosition - currentPosition);

            telemetry.addData("ticksNeeded ", ticksNeeded);
            telemetry.addData("ticksMoved ", ticksMoved);
            telemetry.addData("ticksNeeded - ticksMoved ", ticksNeeded - ticksMoved);
            telemetry.update();

            up.setPower(power * directionSign);
        }
        up.setPower(0);
    }
    public static double liftInchesToTicks(double inches){
        double inchesPerRev = (2 * Math.PI);
        double ticks_per_inch = (1120 / inchesPerRev);
        double ticks = inches * ticks_per_inch;
        return ticks;
    }


}
