package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class RedAutoState extends LinearOpMode {
    OpenCvCamera phoneCam;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;
    private DcMotor yeeter = null;
    private ElapsedTime runTime = new ElapsedTime();
    private DcMotor armMotor = null;
    private Servo armServo = null;

    private Servo foundL = null;
    private Servo foundR = null;

    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static float rectHeight = .6f / 8f;
    private static float rectWidth = 1.5f / 8f;

    private static float offsetX = -1.2f / 8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 1f / 8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f / 8f + offsetX, 4f / 8f + offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2f / 8f + offsetX, 4f / 8f + offsetY};
    private static float[] rightPos = {6f / 8f + offsetX, 4f / 8f + offsetY};
    BNO055IMU imu;
    //moves all rectangles right or left by amount. units are in ratio to monitor
    @Override
    public void runOpMode() {
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using the phone's internal camera. We pass it a
         * CameraDirection enum indicating whether to use the front or back facing
         * camera, as well as the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        /*
         * Open the connection to the camera device
         */
        phoneCam.openCameraDevice();

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        phoneCam.setPipeline(new StageSwitchingPipeline());

        /*
         * Tell the camera to start streaming images to us! Note that you must make sure
         * the resolution you specify is supported by the camera. If it is not, an exception
         * will be thrown.
         *
         * Also, we specify the rotation that the camera is used in. This is so that the image
         * from the camera sensor can be rotated such that it is always displayed with the image upright.
         * For a front facing camera, rotation is defined assuming the user is looking at the screen.
         * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
         * away from the user.
         */
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        /*
         * Wait for the user to press start on the Driver Station
         */

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        // make sure the gyro is calibrated before continuing

        yeeter = hardwareMap.get(DcMotor.class, "yeeter");
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftRear = hardwareMap.get(DcMotor.class, "left_rear");
        rightRear = hardwareMap.get(DcMotor.class, "right_rear");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        armServo = hardwareMap.get(Servo.class, "servo_arm");
        foundL = hardwareMap.get(Servo.class, "fl");
        foundR = hardwareMap.get(Servo.class, "fr");


        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);;

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        yeeter.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        int currLiftPos = armMotor.getCurrentPosition();
        armMotor.setPower(1);
        sleep(30);
        armMotor.setPower(0);
        foundationUP();
        armServo.setPosition(.15);
        runtime.reset();
        while (opModeIsActive()) {
            if (valLeft == 0) {
                telemetry.addData("pos", "left");
                telemetry.update();
                encoderArm1(830, 2120, 2120, 830, .55, 1.5, currLiftPos + 8, 0, .3);
                grabBlock();
                encoder(-230, -230, -230, -230, .8, 1.0);
                turn_to_heading(270, 0);
                sleep(50);
                encoders(3850, 1, 270);
                turn_to_heading1(0, -25, .9);
                encoder(350, 350, 350, 350, .8, 1.0);
                foundationDown();
                sleep(200);
                encoder(-800,-1900,-800,-1900,1,1.5);
                encoder(1200,-1200,1200,-1200,1,1.2);
                dropBlock();
                foundationUP();
                encoderArm1(-750, 750, 750, -750, .5, 1.0, currLiftPos +5, -300, .7);
                turn_to_heading(271, 0);
                encoderArm1(-4350, -4350, -4350, -4350, 1, 3.0, currLiftPos+5, -2000, .7);
                turn_to_heading(30, 0);
                encoder(0, 390, 0, 390, .55, .8);
                grabBlock();
                encoder(-700, -700, -700, -700, .8, .8);
                turnRight();
                turn_to_heading(270.5, -25);
                encoderArm(4500, 4500, 4500, 4500, 1, 3.0, 700, 2450, 1.0);
                // armLift(600,.8);
                dropBlock();
                turn_to_heading2(100, -25, currLiftPos, 1.0);
                yeeter.setPower(1);
                sleep(600);
                yeeter.setPower(0);
                armMotor.setTargetPosition(currLiftPos);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(.4);
                while (armMotor.isBusy() && opModeIsActive()) {
                }
                telemetry.addData("pos", armMotor.getCurrentPosition());
                telemetry.update();
                armMotor.setPower(0);
                sleep(10000);

                telemetry.update();
            } else if (valMid == 0) {
                telemetry.addData("pos", "center");
                telemetry.update();
                encoderArm1(1420, 1130, 1130, 1420, .55, 1.5, currLiftPos + 8, 0, .3);
                grabBlock();
                encoder(-210, -210, -210, -210, .8, 1.0);
                turn_to_heading(271, 0);
                sleep(50);
                encoders(3700, 1, 270);
                turn_to_heading1(0, -25, .9);
                encoder(350, 350, 350, 350, .8, 1.0);
                foundationDown();
                sleep(200);
                encoder(-800,-1900,-800,-1900,1,1.5);
                encoder(1300,-1300,1300,-1300,1,1.2);
                dropBlock();
                foundationUP();
                encoderArm1(-750, 750, 750, -750, .5, 1.0, currLiftPos +5, -300, .7);
                turn_to_heading(271, 0);
                encoderArm1(-4050, -4050, -4050, -4050, 1, 3.0, currLiftPos+5, -2000, .7);
                turn_to_heading(0, 0);
                encoder(390, 390, 390, 390, .55, .8);
                grabBlock();
                encoder(-500, -500, -500, -500, .8, .8);
                turnRight();
                turn_to_heading(270.5, -25);
                encoderArm(4400, 4400, 4400, 4450, 1, 3.0, 700, 2450, 1.0);
                // armLift(600,.8);
                dropBlock();
                turn_to_heading2(100, -25, currLiftPos, 1.0);
                yeeter.setPower(1);
                sleep(550);
                yeeter.setPower(0);
                armMotor.setTargetPosition(currLiftPos);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(.4);
                while (armMotor.isBusy() && opModeIsActive()) {
                }
                telemetry.addData("pos", armMotor.getCurrentPosition());
                telemetry.update();
                armMotor.setPower(0);
                sleep(10000);

                telemetry.update();
            } else if (valRight == 0) {
                telemetry.addData("pos", "right");
                telemetry.update();
                encoderArm1(1950, 730, 730, 1950, .55, 1.5, currLiftPos + 8, 0, .3);
                grabBlock();
                encoder(-320, -320, -320, -320, .8, 1.0);
                turn_to_heading(270, 0);
                sleep(50);
                encoders(3400, 1, 270);
                turn_to_heading1(0, -25, .9);
                encoder(350, 350, 350, 350, .8, 1.0);
                foundationDown();
                sleep(200);
                encoder(-900,-2000,-900,-2000,1,1.5);
                encoder(1200,-1200,1200,-1200,1,1.2);
                dropBlock();
                foundationUP();
                encoderArm1(-700, 700, 700, -700, .5, 1.0, currLiftPos +5, -300, .7);
                turn_to_heading(271, 0);
                encoderArm1(-3700, -3700, -3700, -3700, 1, 3.0, currLiftPos+5, -2000, .7);
                turn_to_heading(0, 0);
                encoder(390, 390, 390, 390, .55, .8);
                grabBlock();
                encoder(-650, -650, -650, -650, .8, .8);
                turnRight();
                turn_to_heading(269.5, -25);
                encoderArm(4350, 4350, 4350, 4350, 1, 3.0, 500, 2450, 1.0);
                // armLift(600,.8);
                armServo.setPosition(.15);
                sleep(50);
                turn_to_heading2(90, -25, currLiftPos, 1.0);
                yeeter.setPower(1);
                sleep(600);
                yeeter.setPower(0);
                armMotor.setTargetPosition(currLiftPos);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(.4);
                while (armMotor.isBusy() && opModeIsActive()) {
                }
                telemetry.addData("pos", armMotor.getCurrentPosition());
                telemetry.update();
                armMotor.setPower(0);
                sleep(10000);

                telemetry.update();
            }
        }
    }

    /*
     * An example image processing pipeline to be run upon receipt of each frame from the camera.
     * Note that the processFrame() method is called serially from the frame worker thread -
     * that is, a new camera frame will not come in while you're still processing a previous one.
     * In other words, the processFrame() method will never be called multiple times simultaneously.
     *
     * However, the rendering of your processed image to the viewport is done in parallel to the
     * frame worker thread. That is, the amount of time it takes to render the image to the
     * viewport does NOT impact the amount of frames per second that your pipeline can process.
     *
     * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
     * frame worker thread. This should not be a problem in the vast majority of cases. However,
     * if you're doing something weird where you do need it synchronized with your OpMode thread,
     * then you will need to account for that accordingly.
     */
    static class StageSwitchingPipeline extends OpenCvPipeline {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped() {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if (nextStageNum >= stages.length) {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input) {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int) (input.rows() * midPos[1]), (int) (input.cols() * midPos[0]));//gets value at circle
            valMid = (int) pixMid[0];

            double[] pixLeft = thresholdMat.get((int) (input.rows() * leftPos[1]), (int) (input.cols() * leftPos[0]));//gets value at circle
            valLeft = (int) pixLeft[0];

            double[] pixRight = thresholdMat.get((int) (input.rows() * rightPos[1]), (int) (input.cols() * rightPos[0]));//gets value at circle
            valRight = (int) pixRight[0];

            //create three points
            Point pointMid = new Point((int) (input.cols() * midPos[0]), (int) (input.rows() * midPos[1]));
            Point pointLeft = new Point((int) (input.cols() * leftPos[0]), (int) (input.rows() * leftPos[1]));
            Point pointRight = new Point((int) (input.cols() * rightPos[0]), (int) (input.rows() * rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid, 5, new Scalar(255, 0, 0), 1);//draws circle
            Imgproc.circle(all, pointLeft, 5, new Scalar(255, 0, 0), 1);//draws circle
            Imgproc.circle(all, pointRight, 5, new Scalar(255, 0, 0), 1);//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols() * (leftPos[0] - rectWidth / 2),
                            input.rows() * (leftPos[1] - rectHeight / 2)),
                    new Point(
                            input.cols() * (leftPos[0] + rectWidth / 2),
                            input.rows() * (leftPos[1] + rectHeight / 2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols() * (midPos[0] - rectWidth / 2),
                            input.rows() * (midPos[1] - rectHeight / 2)),
                    new Point(
                            input.cols() * (midPos[0] + rectWidth / 2),
                            input.rows() * (midPos[1] + rectHeight / 2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols() * (rightPos[0] - rectWidth / 2),
                            input.rows() * (rightPos[1] - rectHeight / 2)),
                    new Point(
                            input.cols() * (rightPos[0] + rectWidth / 2),
                            input.rows() * (rightPos[1] + rectHeight / 2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport) {
                case THRESHOLD: {
                    return thresholdMat;
                }

                case detection: {
                    return all;
                }

                case RAW_IMAGE: {
                    return input;
                }

                default: {
                    return input;
                }
            }

        }

    }
    public  void foundationUP()
    {
        foundL.setPosition(.6);
        foundR.setPosition(.2);
        sleep(300);
    }
    public  void foundationDown()
    {
        foundL.setPosition(.15);
        foundR.setPosition(.65);
        sleep(300);
    }

    public void encoder(int lf, int rf, int lr, int rr, double pow, Double sec) {
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
    public void encoderArm (int lf, int rf, int lr, int rr, double pow, Double sec, int tick, int when, Double armpow) {
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
            telemetry.addData("status", "busy");
            telemetry.update();
            if (rightRear.getCurrentPosition() > when && armMotor.getCurrentPosition() < tick) {
                armMotor.setPower(armpow);

            } else {
                armMotor.setPower(0);
            }
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        leftRear.setPower(0);
        armMotor.setPower(0);

    }
    public void encoderArm1  (int lf, int rf, int lr, int rr, double pow, Double sec, int tick, int when, Double armpow) {
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
            telemetry.addData("status", "busy");
            telemetry.update();
            if (rightRear.getCurrentPosition() > when && armMotor.getCurrentPosition() > tick) {
                armMotor.setPower(-armpow);

            } else {
                armMotor.setPower(0);
            }

        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        leftRear.setPower(0);
        armMotor.setPower(0);

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
    public void checkAngle(int firstA, Double power) {

        if (firstA-3.5 > (((imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)).firstAngle + 360) % 360))
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
            while (currAngle < firstA-.5 && opModeIsActive()) {
                leftFront.setPower(-power);
                leftRear.setPower(-power);
                rightRear.setPower(power);
                rightFront.setPower(power);
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
        else if (firstA +3.5 < (((imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)).firstAngle + 360) % 360))
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
            while (currAngle > firstA + .5 && opModeIsActive()) {
                leftFront.setPower(power);
                leftRear.setPower(power);
                rightRear.setPower(-power);
                rightFront.setPower(-power);
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
    public void armLift(int pos, Double pow)
    {


    }

    public void turn_to_heading(double target_heading, double speedModifier) {
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        boolean goRight;
        double currentHeading;
        double degreesToTurn;
        double wheelPower;
        double prevHeading = 0;
        ElapsedTime timeoutTimer = new ElapsedTime();

        currentHeading = getHeading();
        degreesToTurn = Math.abs(target_heading - currentHeading);

        goRight = target_heading > currentHeading;

        if (degreesToTurn > 180) {
            goRight = !goRight;
            degreesToTurn = 360 - degreesToTurn;
        }

        timeoutTimer.reset();
        prevHeading = currentHeading;
        while (degreesToTurn > .5 && opModeIsActive() && timeoutTimer.seconds() < 5) {  // 11/21 changed from .5 to .3

            if (speedModifier < 0) {
                wheelPower = (Math.pow((degreesToTurn + 25) / -speedModifier, 3) + 15) / 100;
            } else {
                if (speedModifier != 0) {
                    wheelPower = (Math.pow((degreesToTurn) / speedModifier, 4) + 35) / 100;
                } else {
                    wheelPower = (Math.pow((degreesToTurn) / 30, 4) + 15) / 100;
                }
            }

            if (goRight) {
                wheelPower = -wheelPower;
            }
            leftFront.setPower(wheelPower);
            leftRear.setPower(wheelPower);
            rightRear.setPower(-wheelPower);
            rightFront.setPower(-wheelPower);

            currentHeading = getHeading();

            degreesToTurn = Math.abs(target_heading - currentHeading);       // Calculate how far is remaining to turn

            goRight = target_heading > currentHeading;

            if (degreesToTurn > 180) {
                goRight = !goRight;
                degreesToTurn = 360 - degreesToTurn;
            }

            if (Math.abs(currentHeading - prevHeading) > 1) {  // if it has turned at least one degree
                timeoutTimer.reset();
                prevHeading = currentHeading;
            }

        }
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        rightFront.setPower(0);

    }

    public void turn_to_heading1(double target_heading, double speedModifier , Double armpow) {
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        boolean goRight;
        double currentHeading;
        double degreesToTurn;
        double wheelPower;
        double prevHeading = 0;
        ElapsedTime timeoutTimer = new ElapsedTime();

        currentHeading = getHeading();
        degreesToTurn = Math.abs(target_heading - currentHeading);

        goRight = target_heading > currentHeading;

        if (degreesToTurn > 180) {
            goRight = !goRight;
            degreesToTurn = 360 - degreesToTurn;
        }

        timeoutTimer.reset();
        prevHeading = currentHeading;
        while (degreesToTurn > .5 && opModeIsActive() && timeoutTimer.seconds() < 5) {  // 11/21 changed from .5 to .3

            if (speedModifier < 0) {
                wheelPower = (Math.pow((degreesToTurn + 25) / -speedModifier, 3) + 15) / 100;
            } else {
                if (speedModifier != 0) {
                    wheelPower = (Math.pow((degreesToTurn) / speedModifier, 4) + 35) / 100;
                } else {
                    wheelPower = (Math.pow((degreesToTurn) / 30, 4) + 15) / 100;
                }
            }

            if (goRight) {
                wheelPower = -wheelPower;
            }
            armMotor.setPower(armpow);

            leftFront.setPower(wheelPower);
            leftRear.setPower(wheelPower);
            rightRear.setPower(-wheelPower);
            rightFront.setPower(-wheelPower);

            currentHeading = getHeading();

            degreesToTurn = Math.abs(target_heading - currentHeading);       // Calculate how far is remaining to turn

            goRight = target_heading > currentHeading;

            if (degreesToTurn > 180) {
                goRight = !goRight;
                degreesToTurn = 360 - degreesToTurn;
            }

            if (Math.abs(currentHeading - prevHeading) > 1) {  // if it has turned at least one degree
                timeoutTimer.reset();
                prevHeading = currentHeading;
            }

        }
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        rightFront.setPower(0);
        armMotor.setPower(0);
    }


    public void turn_to_heading2(double target_heading, double speedModifier, int tick, Double armpow) {
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        boolean goRight;
        double currentHeading;
        double degreesToTurn;
        double wheelPower;
        double prevHeading = 0;
        ElapsedTime timeoutTimer = new ElapsedTime();

        currentHeading = getHeading();
        degreesToTurn = Math.abs(target_heading - currentHeading);

        goRight = target_heading > currentHeading;

        if (degreesToTurn > 180) {
            goRight = !goRight;
            degreesToTurn = 360 - degreesToTurn;
        }

        timeoutTimer.reset();
        prevHeading = currentHeading;
        while (degreesToTurn > .5 && opModeIsActive() && timeoutTimer.seconds() < 5) {  // 11/21 changed from .5 to .3

            if (speedModifier < 0) {
                wheelPower = (Math.pow((degreesToTurn + 25) / -speedModifier, 3) + 15) / 100;
            } else {
                if (speedModifier != 0) {
                    wheelPower = (Math.pow((degreesToTurn) / speedModifier, 4) + 35) / 100;
                } else {
                    degreesToTurn+=10;
                    wheelPower = (Math.pow((degreesToTurn) / 30, 4) + 15) / 100;
                }
            }

            if (goRight) {
                wheelPower = -wheelPower;
            }
            if(armMotor.getCurrentPosition() > tick + 5 ) {
                armMotor.setPower(-armpow);
            }
            else
            {
                armMotor.setPower(0);
            }

            leftFront.setPower(wheelPower);
            leftRear.setPower(wheelPower);
            rightRear.setPower(-wheelPower);
            rightFront.setPower(-wheelPower);

            currentHeading = getHeading();

            degreesToTurn = Math.abs(target_heading - currentHeading);       // Calculate how far is remaining to turn

            goRight = target_heading > currentHeading;

            if (degreesToTurn > 180) {
                goRight = !goRight;
                degreesToTurn = 360 - degreesToTurn;
            }

            if (Math.abs(currentHeading - prevHeading) > 1) {  // if it has turned at least one degree
                timeoutTimer.reset();
                prevHeading = currentHeading;
            }

        }
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        rightFront.setPower(0);
        armMotor.setPower(0);
    }
    public void encoders(int pos, double pow, int angle)
    {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));

        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (rightRear.getCurrentPosition() < pos && opModeIsActive()) {
            int error = pos - rightRear.getCurrentPosition();
            double counter = (error * (.1/pos));
            double power = (error * (pow / pos))+.5;
            leftFront.setPower(power);
            leftRear.setPower(power);
            rightRear.setPower(power+counter);
            rightFront.setPower(power + counter);
        }
        // checkAngle(angle, .1);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        rightFront.setPower(0);
    }
    public void encoders1(int pos, double pow, int angle)
    {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));

        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (rightRear.getCurrentPosition() < pos && opModeIsActive()) {
            int error = pos - rightRear.getCurrentPosition();
            double counter = (error * (.1/pos));
            double power = (error * (pow / pos))+.35;

            leftFront.setPower(power);
            leftRear.setPower(power);
            rightRear.setPower(power+counter);
            rightFront.setPower(power + counter);
        }
        // checkAngle(angle, .1);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        rightFront.setPower(0);
    }
    public void grabBlock()
    {
        armServo.setPosition(.45);
        sleep(200);
    }
    public void dropBlock()
    {
        armServo.setPosition(.15);
        sleep(200);
    }
    public void turnRight()
    {
        leftFront.setPower(1);
        leftRear.setPower(1);
        rightRear.setPower(-1);
        rightFront.setPower(-1);
        sleep(300);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        rightFront.setPower(0);
    }
    public void turnLeft()
    {
        leftFront.setPower(-1);
        leftRear.setPower(-1);
        rightRear.setPower(1);
        rightFront.setPower(1);
        sleep(300);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        rightFront.setPower(0);
    }
}