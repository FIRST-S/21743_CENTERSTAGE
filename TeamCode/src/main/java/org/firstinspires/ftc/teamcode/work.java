package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp (name = "pls work" ,group = "Linear Opmode")

//@Disabled
public class work extends LinearOpMode {
    //Dashboard demo variables
    public static double ORBITAL_FREQUENCY = 0.05;

    public static boolean intakeOpen = true;
    public static double SPIN_FREQUENCY = 0.25;

    //public static double INTAKE_OPEN_POS = 0.0;
    public static double INTAKE_OPEN_POS = 0.95;
    //public static double INTAKE_CLOSE_POS = 0.4;
    public static double INTAKE_CLOSE_POS = 0;
    public static double ORBITAL_RADIUS = 50;
    public static double SIDE_LENGTH = 10;

    private IMU imu = null;      // Control/Expansion Hub IMU


    //Motor demo variables
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx backRightDrive = null;

    private DcMotorEx climberMoter = null;

    private Servo hook = null;

    private ColorRangeSensor distRight = null;
    private Servo intake = null;
    private Servo shooter = null;
    private DcMotorEx arm = null;

    public void initializeMotors()
    {
        //Configuring and Classifying
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "FL");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "FR");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "BL");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "BR");
        climberMoter = hardwareMap.get(DcMotorEx.class, "climber");
        arm = hardwareMap.get(DcMotorEx.class, "Arm");
        intake = hardwareMap.get(Servo.class, "intake");
        distRight = hardwareMap.get(ColorRangeSensor.class, "distRight");
        hook = hardwareMap.get(Servo.class, "hook");
        shooter = hardwareMap.get(Servo.class, "shooter");


        //self explanatory
        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        climberMoter.setDirection(DcMotorSimple.Direction.REVERSE);

        //braking when no power set
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climberMoter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //setting runmode
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        climberMoter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //configuring arm, setting it's starting position to 0
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPositionTolerance(1);
        arm.setTargetPosition(0);
        arm.setPower(0.5);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        climberMoter.setDirection(DcMotorSimple.Direction.REVERSE);
        hook.setDirection(Servo.Direction.FORWARD);
        hook.scaleRange(0,1);
    }

    //irrelevant
    private static void rotatePoints(double[] xPoints, double[] yPoints, double angle) {
        for (int i = 0; i < xPoints.length; i++) {
            double x = xPoints[i];
            double y = yPoints[i];
            xPoints[i] = x * Math.cos(angle) - y * Math.sin(angle);
            yPoints[i] = x * Math.sin(angle) + y * Math.cos(angle);
        }
    }

    //irrelevant
    public void dashboardDemo(){
        double time = getRuntime();

        double bx = ORBITAL_RADIUS * Math.cos(2 * Math.PI * ORBITAL_FREQUENCY * time);
        double by = ORBITAL_RADIUS * Math.sin(2 * Math.PI * ORBITAL_FREQUENCY * time);
        double l = SIDE_LENGTH / 2;

        double[] bxPoints = { l, -l, -l, l };
        double[] byPoints = { l, l, -l, -l };
        rotatePoints(bxPoints, byPoints, 2 * Math.PI * SPIN_FREQUENCY * time);
        for (int i = 0; i < 4; i++) {
            bxPoints[i] += bx;
            byPoints[i] += by;
        }


        sleep(20);
    }

    //irrelevant
    private void joystickTankDrive()
    {
        float Lj_y;
        float Rj_y;

        Lj_y = gamepad1.left_stick_y;
        Rj_y = gamepad1.right_stick_y;

        frontLeftDrive.setPower(Lj_y);
        frontRightDrive.setPower(Rj_y);
        backLeftDrive.setPower(Lj_y);
        backRightDrive.setPower(Rj_y);
    }

    private void joystickMecanumDrive() {

        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]

        double denominator = Math.max(Math.abs(y) + Math.abs(rx), 1);
        double frontLeftPower = -(y + rx) / denominator; //A-RC has negative before parentheses and B-RC has no negative
        double backLeftPower = -(y + rx) / denominator;
        double frontRightPower = -(y - rx) / denominator;
        double backRightPower = -(y - rx) / denominator;

        frontLeftDrive.setPower(frontLeftPower + (gamepad1.left_stick_x/3));
        frontRightDrive.setPower(frontRightPower + (gamepad1.left_stick_x/3));
        backLeftDrive.setPower(backLeftPower - (gamepad1.left_stick_x/3));
        backRightDrive.setPower(backRightPower - (gamepad1.left_stick_x/3));

        if (gamepad1.y) {
            frontLeftDrive.setPower(-.2);
            frontRightDrive.setPower(-.2);
            backLeftDrive.setPower(-.2);
            backRightDrive.setPower(-.2);
        }
    }

    private void climber() {
       // float y = -gamepad2.left_stick_y; // Remember, Y stick value is reversed
        if (gamepad2.y){
            climberMoter.setPower(-1);
        }
        else if (gamepad2.a) {
            climberMoter.setPower(1);
        }
        else {
            climberMoter.setPower(0);
        }

        if (gamepad2.left_bumper) {
            hook.setPosition(.85); //first one
        } else if (gamepad2.right_bumper) {
            hook.setPosition(0.65); //hang one
        } else if (gamepad2.b) {
            hook.setPosition(0); //down one
        }
    }


    public void armFunctions() {
        int encoderCount = arm.getCurrentPosition();

        if (gamepad1.right_trigger > .3) {
            intake.setPosition(INTAKE_CLOSE_POS);
        } else if (gamepad1.left_trigger >.3) {
            intake.setPosition(INTAKE_OPEN_POS);
        }


        if (gamepad2.right_trigger > .3) {
            arm.setTargetPosition(encoderCount + Math.round(40 * gamepad2.right_trigger));
        } else if (gamepad2.left_trigger > .3) {
            arm.setTargetPosition(encoderCount - Math.round(40 * gamepad2.left_trigger));
        }
    }


    //irrelevant
    private void fieldCentricDrive() {
        double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = -gamepad1.right_stick_x;
        double lf = -gamepad2.left_stick_y;

        // Calculate the current angle of the robot (yaw) relative to the field.
        double robotAngle = -Math.toRadians(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)); // Modify this to get the actual robot angle.

        // Calculate the field-centric components of movement.
        double fieldX = x * Math.cos(robotAngle) - y * Math.sin(robotAngle);
        double fieldY = x * Math.sin(robotAngle) + y * Math.cos(robotAngle);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
//        // but only if at least one is out of the range [-1, 1]
//        double denominator = Math.max(Math.abs(fieldY) + Math.abs(fieldX) + Math.abs(rx), 1);
//        double frontLeftPower = (fieldY + fieldX + rx) / denominator;
//        double backLeftPower = (fieldY - fieldX + rx) / denominator;
//        double frontRightPower = (fieldY - fieldX - rx) / denominator;
//        double backRightPower = (fieldY + fieldX - rx) / denominator;

        double denominator = Math.max(Math.abs(y) + Math.abs(rx), 1);
        double frontLeftPower = -(fieldY + rx) / denominator; //A-RC has negative before parentheses and B-RC has no negative
        double backLeftPower = -(fieldY + rx) / denominator;
        double frontRightPower = -(fieldY - rx) / denominator;
        double backRightPower = -(fieldY - rx) / denominator;

        // Set the motor powers.
        frontLeftDrive.setPower(frontLeftPower);
        backLeftDrive.setPower(backLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backRightDrive.setPower(backRightPower);
    }

    // Modify this method to return the actual robot angle in degrees.
    private double getCurrentRobotAngle() {
        // Implement this method to return the current robot angle in degrees.
        // You might use sensors like a gyro or IMU to get the robot's orientation.
        // Make sure it returns the angle in degrees.
        return 0.0; // Replace with actual implementation.
    }

    int ignoreThis = 1;





    public void runOpMode()  {
        initializeMotors();
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();



        intake.scaleRange(0,1);
        //intake.setPosition(Servo.Direction.FORWARD);
        intake.setDirection(Servo.Direction.REVERSE);
        //intake.setPosition(0.4);
        intake.setPosition(0.0);

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("hook pos", hook.getPosition());
            telemetry.addData("FL ( 0)", frontLeftDrive.getCurrentPosition());
            telemetry.addData("FR (1)", frontRightDrive.getCurrentPosition());
            telemetry.addData("BL (2)", backLeftDrive.getCurrentPosition());
            telemetry.addData("BR (3)", backRightDrive.getCurrentPosition());
            telemetry.addData("arm pos", arm.getCurrentPosition());
            telemetry.addData("intake pos", intake.getPosition());
            telemetry.addData("yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("Arm, up: Right Trigger, down: Left Trigger ", ignoreThis);
            telemetry.addData("Intake, open: Pad 1 Right Trig close: Pad 1 Left Trig ", ignoreThis);
            telemetry.addData("Climber, up: Y, down: A", ignoreThis);
            armFunctions();


           // telemetry.addData("climber pos", climberMoter.getCurrentPosition());
            updateTelemetry(telemetry);
            //joystickTankDrive();
            joystickMecanumDrive();
//            fieldCentricDrive();
            climber();
//            dashboardDemo();
            if (gamepad2.x) {
                shooter.setPosition(.1);
            }
        }
    }
}
