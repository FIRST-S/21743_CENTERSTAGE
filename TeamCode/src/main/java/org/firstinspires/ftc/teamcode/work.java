package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

    public static double INTAKE_OPEN_POS = 0.0;
    public static double INTAKE_CLOSE_POS = 0.4;
    public static double ORBITAL_RADIUS = 50;
    public static double SIDE_LENGTH = 10;

    private IMU imu = null;      // Control/Expansion Hub IMU


    //Motor demo variables
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx backRightDrive = null;

    private DcMotorEx climberMoter = null;


    private Servo intake = null;

    private DcMotorEx arm = null;

    public void initializeMotors()
    {
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "FL");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "FR");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "BL");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "BR");
        //climberMoter = hardwareMap.get(DcMotorEx.class, "climber");
        arm = hardwareMap.get(DcMotorEx.class, "Arm");
        intake = hardwareMap.get(Servo.class, "intake");


        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        //climberMoter.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //climberMoter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //climberMoter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }

    private static void rotatePoints(double[] xPoints, double[] yPoints, double angle) {
        for (int i = 0; i < xPoints.length; i++) {
            double x = xPoints[i];
            double y = yPoints[i];
            xPoints[i] = x * Math.cos(angle) - y * Math.sin(angle);
            yPoints[i] = x * Math.sin(angle) + y * Math.cos(angle);
        }
    }


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
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;
        double lf = gamepad2.left_stick_y;


        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = -(y + x + rx) / denominator; //A-RC has negative before parentheses and B-RC has no negative
        double backLeftPower = -(y - x + rx) / denominator;
        double frontRightPower = -(y - x - rx) / denominator;
        double backRightPower = -(y + x - rx) / denominator;

        frontLeftDrive.setPower(frontLeftPower);
        backLeftDrive.setPower(backLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backRightDrive.setPower(backRightPower);


    }

    private void climber() {
        float y = -gamepad2.left_stick_y; // Remember, Y stick value is reversed
        if (gamepad2.y){
            climberMoter.setPower(1);
        }
        else if (gamepad2.a) {
            climberMoter.setPower(-1);
        }
        else {
            climberMoter.setPower(0);
        }

    }

    public void armFunctions() {
//        boolean pickUpPose = false;
//        boolean scorePos = false;
//        if (gamepad1.a){
//            if (pickUpPose){
//                pickUpPose = false;
//            }
//            else {
//                pickUpPose = true;
//            }
//        }
//        else if (gamepad1.b){
//            if (scorePos){
//                scorePos = false;
//            }
//            else {
//                if (pickUpPose == false){
//                    scorePos = true;
//                }
//                else {
//                    pickUpPose = false;
//                    scorePos = true;
//                }
//            }
//        }
        if (gamepad2.x) {
            arm.setTargetPosition(-110);
        }
        else if (gamepad2.b) {
            arm.setTargetPosition(-740);
        }
        else if (gamepad2.a){
            arm.setTargetPosition(-30);
        }

        if (gamepad1.right_bumper){
            intake.setPosition(INTAKE_CLOSE_POS);
        }
        if (gamepad1.left_bumper){
            intake.setPosition(INTAKE_OPEN_POS);
        }


    }
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
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(fieldY) + Math.abs(fieldX) + Math.abs(rx), 1);
        double frontLeftPower = (fieldY + fieldX + rx) / denominator;
        double backLeftPower = (fieldY - fieldX + rx) / denominator;
        double frontRightPower = (fieldY - fieldX - rx) / denominator;
        double backRightPower = (fieldY + fieldX - rx) / denominator;

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





    public void runOpMode()  {
        initializeMotors();
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();





        intake.scaleRange(0,1);
        intake.setDirection(Servo.Direction.FORWARD);
        intake.setPosition(0);

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("FL (0)", frontLeftDrive.getCurrentPosition());
            telemetry.addData("FR (1)", frontRightDrive.getCurrentPosition());
            telemetry.addData("BL (2)", backLeftDrive.getCurrentPosition());
            telemetry.addData("BR (3)", backRightDrive.getCurrentPosition());
            telemetry.addData("arm pos", arm.getCurrentPosition());
            telemetry.addData("intake pos", intake.getPosition());
            telemetry.addData("yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            armFunctions();


//            telemetry.addData("climber pos", climberMoter.getCurrentPosition());
            updateTelemetry(telemetry);

            //joystickTankDrive();
//            joystickMecanumDrive();
            fieldCentricDrive();
            //climber();
//            dashboardDemo();
        }
    }
}
