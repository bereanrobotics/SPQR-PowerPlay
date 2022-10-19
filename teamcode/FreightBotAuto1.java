package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.*;

/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 */
@Autonomous(name = "FreightBot Auto1")
public class FreightBotAuto1 extends LinearOpMode {

    DcMotorEx m1, m2, m3, m4, arm, rotor;
    Servo handServo, frontServo;
    BNO055IMU imu;
    DistanceSensor frontDistance; // leftDistance, rightDistance, backDistance;
    //ColorSensor colorSensor;
    ElapsedTime runtime;

    private void updateTelemetry() {
        //telemetry.addData("Color","R %d  G %d  B %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());
        //telemetry.addData("Heading"," %.1f", gyro.getHeading());
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        telemetry.addData("Heading", " %.1f", orientation.firstAngle * 180.0 / Math.PI);

        telemetry.addData("Front Distance", " %.1f", frontDistance.getDistance(DistanceUnit.CM));
//        telemetry.addData("Left Distance", " %.1f", leftDistance.getDistance(DistanceUnit.CM));
//        telemetry.addData("Right Distance", " %.1f", rightDistance.getDistance(DistanceUnit.CM));
//        telemetry.addData("Back Distance", " %.1f", backDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("Encoders"," %d %d %d %d", m1.getCurrentPosition(), m2.getCurrentPosition(),
                m3.getCurrentPosition(), m4.getCurrentPosition());
        telemetry.update();
    }

    void waitFor(double timeout) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < timeout)) {
            updateTelemetry();
            idle();
        }
    }

    void stopDriving() {
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
    }

    void drive(double px, double py, double pa ) {
        if (Math.abs(pa) < 0.05) pa = 0;
        double p1 = -px + py - pa;
        double p2 = px + py + -pa;
        double p3 = -px + py + pa;
        double p4 = px + py + pa;
        double max = Math.max(1.0, Math.abs(p1));
        max = Math.max(max, Math.abs(p2));
        max = Math.max(max, Math.abs(p3));
        max = Math.max(max, Math.abs(p4));
        p1 /= max;
        p2 /= max;
        p3 /= max;
        p4 /= max;
        m1.setPower(p1);
        m2.setPower(p2);
        m3.setPower(p3);
        m4.setPower(p4);
    }

    void setup() {
        m1 = hardwareMap.get(DcMotorEx.class, "back_left");
        m2 = hardwareMap.get(DcMotorEx.class, "front_left");
        m3 = hardwareMap.get(DcMotorEx.class, "front_right");
        m4 = hardwareMap.get(DcMotorEx.class, "back_right");
        m1.setDirection(DcMotor.Direction.REVERSE);
        m2.setDirection(DcMotor.Direction.REVERSE);
        m4.setDirection(DcMotor.Direction.REVERSE);
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm = hardwareMap.get(DcMotorEx.class, "arm_motor");
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //rotor = hardwareMap.get(DcMotorEx.class, "rotor_motor");
        //rotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        Servo handServo = hardwareMap.get(Servo.class, "hand_servo");
        Servo frontServo = hardwareMap.get(Servo.class, "front_servo");
        frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
        //leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
        //rightDistance = hardwareMap.get(DistanceSensor.class, "right_distance");
        //backDistance = hardwareMap.get(DistanceSensor.class, "back_distance");


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelerationIntegrationAlgorithm = null;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationData = null;
        parameters.calibrationDataFile = "";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "Who cares.";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

//        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");

        runtime = new ElapsedTime();
    }

    public void runOpMode(){
        setup();

        telemetry.addData("Press Start When Ready","");
        telemetry.update();

        // reset arm
        handServo.setPosition(1);
//        arm.setPower(-1);
//        sleep(1000);
//        arm.setPower(0);

        waitForStart();

        drive(0, 1,0);
        waitFor(2);
        stopDriving();

//        drive(0,0, 1);
//        waitFor(1);
//        drive(-1,0, 1);
//        waitFor(0.5);
//        drive(0,-1, 0);
//        waitFor(0.6);
//        stopDriving();
//        rotor.setPower(-1);
//        waitFor(4);
//        rotor.setPower(0);
//        drive(0,1, 0);
//        waitFor(0.5);
//        drive(0,0, -1);
//        waitFor(1);
//        drive(0,-1, 0);
//        waitFor(0.3);
//
//        // move to warehouse and drop on shared depot
//        drive(-1,0, 0);
//        waitFor(3);
//        drive(0,1, 0);
//        waitFor(1.3);
//        stopDriving();
//        arm.setPower(1);
//        waitFor(1);
//        arm.setPower(0);
//        handServo.setPosition(0);
//        waitFor(0.4);
//        arm.setPower(-1);
//        drive(0,-1, 0);
//        waitFor(1);
    }
}
