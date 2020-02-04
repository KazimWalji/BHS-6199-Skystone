/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Forward", group="Linear Opmode")
//@Disabled
public class Forward extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;
    private ElapsedTime runTime = new ElapsedTime();
    private DcMotor armMotor = null;
    private Servo armServo = null;
    OpenCvCamera phoneCam;
    BNO055IMU imu;
    @Override
    public void runOpMode() {

        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftRear = hardwareMap.get(DcMotor.class, "left_rear");
        rightRear = hardwareMap.get(DcMotor.class, "right_rear");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        armServo = hardwareMap.get(Servo.class, "servo_arm");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        runtime.reset();
        int currLiftPos = armMotor.getCurrentPosition();
        int[] pos = {currLiftPos, 400, 1100, 1800, 2500, 3200, 3800, 1499, 1699};
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            encoderSlow(1000,1000,1000,1000, .5, 5, 800);
            sleep(60000);
        }
    }
    public void encoder(int lf, int rf, int lr, int rr, double pow, int sec) {
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));

        rightFront.setTargetPosition(rf);
        leftRear.setTargetPosition(lr);
        rightRear.setTargetPosition(rr);
        leftFront.setTargetPosition(lf);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode((DcMotor.RunMode.RUN_TO_POSITION));


        leftFront.setPower(pow);
        rightFront.setPower(pow);
        rightRear.setPower(pow);
        leftRear.setPower(pow);
        leftFront.setPower(pow);
        runTime.reset();
        while ((rightRear.isBusy() || leftRear.isBusy() || rightFront.isBusy() || leftFront.isBusy()) && opModeIsActive() && runTime.seconds() < sec) {


        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        leftRear.setPower(0);
    }

    public void turnAngle(double firstA, double secA, double pow1, double pow2, double pow3, double pow4) {
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double currAngle;
        Orientation angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = (angle.firstAngle + 360) % 360;
        telemetry.addData("currAngle", currAngle);
        telemetry.update();
        while ((currAngle < firstA || currAngle > secA) && opModeIsActive()) {
            leftFront.setPower(pow1);
            leftRear.setPower(pow2);
            rightRear.setPower(pow3);
            rightFront.setPower(pow4);
            angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currAngle = (angle.firstAngle + 360) % 360;
            telemetry.addData("currAngle", currAngle);
            telemetry.update();
        }
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        rightFront.setPower(0);

    }

    public double getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (angles.firstAngle + 360) % 360;
    }
    public void checkAngle(int firstA, int secA) {
        if (firstA-3 > (((imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)).firstAngle + 360) % 360))
        {
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            double currAngle;
            Orientation angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currAngle = (angle.firstAngle + 360) % 360;
            telemetry.addData("currAngle", currAngle);
            telemetry.update();
            while ((currAngle < firstA || currAngle > secA) && opModeIsActive()) {
                leftFront.setPower(-.2);
                leftRear.setPower(-.2);
                rightRear.setPower(.2);
                rightFront.setPower(.2);
                angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                currAngle = (angle.firstAngle + 360) % 360;
                telemetry.addData("currAngle", currAngle);
                telemetry.update();
            }
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);
            rightFront.setPower(0);
        }
        else if (secA+3 < (((imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)).firstAngle + 360) % 360))
        {
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            double currAngle;
            Orientation angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currAngle = (angle.firstAngle + 360) % 360;
            telemetry.addData("currAngle", currAngle);
            telemetry.update();
            while ((currAngle < firstA || currAngle > secA) && opModeIsActive()) {
                leftFront.setPower(.2);
                leftRear.setPower(.2);
                rightRear.setPower(-.2);
                rightFront.setPower(-.2);
                angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                currAngle = (angle.firstAngle + 360) % 360;
                telemetry.addData("currAngle", currAngle);
                telemetry.update();
            }
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);
            rightFront.setPower(0);
        }

    }

    public void encoderSlow (int lf, int rf, int lr, int rr, double pow, int sec, int tick) {
        pow = pow + .1;
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));

        rightFront.setTargetPosition(rf);
        leftRear.setTargetPosition(lr);
        rightRear.setTargetPosition(rr);
        leftFront.setTargetPosition(lf);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode((DcMotor.RunMode.RUN_TO_POSITION));


        leftFront.setPower(pow);
        rightFront.setPower(pow);
        rightRear.setPower(pow);
        leftRear.setPower(pow);
        leftFront.setPower(pow);
        runTime.reset();
        while ((rightRear.isBusy() || leftRear.isBusy() || rightFront.isBusy() || leftFront.isBusy()) && opModeIsActive() && runTime.seconds() < sec) {
            if(rightRear.getCurrentPosition()>tick)
            {
                leftFront.setPower(.1);
                rightFront.setPower(.1);
                rightRear.setPower(.1);
                leftRear.setPower(.1);
                leftFront.setPower(.1);
            }

        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        leftRear.setPower(0);
    }

}