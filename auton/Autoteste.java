/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.PIDGyro.TurnPIDController;
import org.firstinspires.ftc.teamcode.PIJuncao.ContourPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;


@Autonomous
public class Autoteste extends LinearOpMode {

    public DcMotor RMF;    //Porta 0 - Motor Mecanum - Direita Frontal
    public DcMotor RMB;    //Porta 1 - Motor Mecanum - Direita Traseiro
    public DcMotor LMF;    //Porta 2 - Motor Mecanum - Esquerda Frontal
    public DcMotor LMB;    //Porta 3 - Motor Mecanum - Esquerda Traseiro

    public DcMotor MAT;     //Porta 0 - Motor Core Hex - Antebraço
    public DcMotor MSE;     //Porta 2 - Motor UltraPlanetary - Sistema de Elevação
    public DcMotor MRP;     //Porta 3 - Motor Core Hex - Rotação Prato

    public Servo servoGarra;    //Porta 0 - Direita
    public Servo servoGarra2;   //Porta 1 - Esquerda

    BNO055IMU imu;
    //    public DigitalChannel touch_MRP;
    public DistanceSensor distanceSensorF;
    public DistanceSensor distanceSensorB;

//    CVMaster cv = new CVMaster(this);

//======================================== Tag =====================================================

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1, 2, 3 from the 36h11 family
    int ID_TAG_OF_INTEREST = 18;
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    int id = 0;

    AprilTagDetection tagOfInterest = null;

//====================================== Junção ====================================================

    private OpenCvCamera webcam;

    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    private double CrLowerUpdate = 160;
    private double CbLowerUpdate = 100;
    private double CrUpperUpdate = 255;
    private double CbUpperUpdate = 255;

    public static double borderLeftX = 0.0;   //fraction of pixels from the left side of the cam to skip
    public static double borderRightX = 0.0;   //fraction of pixels from the right of the cam to skip
    public static double borderTopY = 0.0;   //fraction of pixels from the top of the cam to skip
    public static double borderBottomY = 0.0;   //fraction of pixels from the bottom of the cam to skip

    private double lowerruntime = 0;
    private double upperruntime = 0;

    // Yellow Range
    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 100.0, 0.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 170.0, 120.0);

//======================================= Robô =====================================================

    double PID_loc;
    double proporcional_loc, integral_loc, derivada_loc;
    double kP_loc = 0.02, kI_loc = 0, kD_loc = 0.002;

    double erro_loc, ultErro_loc;

    double distanceF, distanceB;

    static final double MAX_POS_D = 0.2;     // Maximum rotational position 0.5
    static final double MIN_POS_D = 0.0;     // Minimum rotational position    certo

    static final double MAX_POS_E = 0.1;     // Maximum rotational position
    static final double MIN_POS_E = 0.4;     // Minimum rotational position 0.5

    double position_D = MIN_POS_D; // Start at halfway position
    double position_E = MIN_POS_E; // Start at halfway position

    public double eixoX_juncao, eixoY_juncao;

    double getAbsoluteAngleVariable, firstAngRotation;
    boolean angBooleanInit = false;


    @Override


    public void runOpMode() {

        initHardware();

        //JuncaoInit();

//        idQRCode();

//        telemetry.addData("Eixo X:\t", cv.eixoX_juncao);
//        telemetry.addData("Eixo Y:\t", cv.eixoY_juncao);
//        telemetry.update();

        waitForStart();

        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */


//        getAbsoluteAngleVariable = getAbsoluteAngle();
//
//        statusGarra(false);
//
//        runToStation();
//
//        statusGarra(true);
//
//        turnPID(90);
//
//        setAllPower(0);

        runToJuncao();

//        searchingJunction();

        telemetry.addData("Eixo X:\t", eixoX_juncao);
        telemetry.addData("Eixo Y :\t", eixoY_juncao);

    }

    //Atalhos


    public void runToStation() {

        do {

            distanceF = distanceSensorF.getDistance(DistanceUnit.CM);

            erro_loc = -(11.5 - distanceF);

            proporcional_loc = erro_loc * kP_loc;

            setAllPower(proporcional_loc);

            telemetry.addData("Proporcional:\t", proporcional_loc);
            telemetry.addData("Distancia Frente:\t", distanceF);
            telemetry.addData("Erro_Loc:\t", erro_loc);
            telemetry.update();

        } while (erro_loc > 0);

        erro_loc = 0;
    }

    public void runToJuncao() {

//        setAllPower(-0.2);
//        sleep(500);

        final double kP_loc = 0.015;

//        do {
//
//            distanceB = distanceSensorB.getDistance(DistanceUnit.CM);
//
//            erro_loc = (60.0 - distanceB);
//
//            proporcional_loc = erro_loc * kP_loc;
//
//            setAllPower(proporcional_loc);
//
//        }while (erro_loc > 0);


        telemetry.addData("Eixo x Junção:\t",borderLeftX);
        telemetry.update();


    }

    public void statusGarra(boolean status) {

        if (status) {
            position_D = MAX_POS_D; // Start at halfway position
            position_E = MAX_POS_E; // Start at halfway position
        } else {
            position_D = MIN_POS_D; // Start at halfway position
            position_E = MIN_POS_E; // Start at halfway position
        }

        servoGarra.setPosition(position_D);              //Direita
        servoGarra2.setPosition(position_E);             //Esquerda
    }

    public void searchingJunction() {
        double motorPower = 0.2, erro = Math.abs(firstAngRotation - getAbsoluteAngle());

        do {
            TurnPIDController pid = new TurnPIDController(firstAngRotation, 0.01, 0, 0.003);
            telemetry.setMsTransmissionInterval(50);

            motorPower = 0.4;
            erro = Math.abs(firstAngRotation - getAbsoluteAngle());

            distanceF = distanceSensorB.getDistance(DistanceUnit.CM);

            if (erro > 0.8) {
                motorPower = (pid.update(getAbsoluteAngle()));
                setMotorPowerIndiv(motorPower, motorPower, -motorPower, -motorPower);
            } else {
                setMotorPowerIndiv(-motorPower, motorPower, motorPower, -motorPower);
            }

            telemetry.addData("Erro:\t", Math.abs(firstAngRotation - getAbsoluteAngle()));
            telemetry.addData("Distancia Frente:\t", distanceF);
            telemetry.addData("First Ang Rotation:\t", firstAngRotation);
            telemetry.update();
            telemetry.addData("Erro:\t", Math.abs(firstAngRotation - getAbsoluteAngle()));

        } while (distanceF > 10);


        while (1 > 0) {

            distanceF = distanceSensorB.getDistance(DistanceUnit.CM);

            telemetry.addLine("FIM");
            telemetry.addData("Distancia Frente:\t", distanceF);
            telemetry.addData("First Ang Rotation:\t", firstAngRotation);
            telemetry.update();

            setAllPower(0);
        }
    }

    //Gyro

    public double getAbsoluteAngle() {
        return imu.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES
        ).firstAngle;
    }

    public void turnPID(double degrees) {

        firstAngRotation = degrees + getAbsoluteAngleVariable;

        turnToPID(degrees + getAbsoluteAngleVariable);
    }

    void turnToPID(double targetAngle) {
        TurnPIDController pid = new TurnPIDController(targetAngle, 0.01, 0, 0.003);
        telemetry.setMsTransmissionInterval(50);
        // Checking lastSlope to make sure that it's not oscillating when it quits
        while (Math.abs(targetAngle - getAbsoluteAngle()) > 0.5 || pid.getLastSlope() > 0.75) {
            double motorPower = pid.update(getAbsoluteAngle());

            setMotorPowerIndiv(motorPower, motorPower, -motorPower, -motorPower);

//            telemetry.addData("Current Angle", getAbsoluteAngle());
//            telemetry.addData("Target Angle", targetAngle);
//            telemetry.addData("Slope", pid.getLastSlope());
//            telemetry.addData("Power", motorPower);

            telemetry.addData("Erro Normal", targetAngle - getAbsoluteAngle());
            telemetry.addData("Erro Abs", (Math.abs(targetAngle - getAbsoluteAngle())));
            telemetry.update();

            if ((Math.abs(targetAngle - getAbsoluteAngle())) == 360 && angBooleanInit) break;
            if ((Math.abs(targetAngle - getAbsoluteAngle())) == 360) {
                angBooleanInit = true;
            }
        }
        setAllPower(0);
    }

    //Hardware Pushbot

    public void initHardware() {

        setInitMotors();

        setInitSensor();

        setDiretion();

        setStatus();

        setEncoders();

//        resetEncoder(-1);

        setAllPower(0);     //Definindo força 0 para todos os motores

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

//        touch_MRP.setMode(DigitalChannel.Mode.INPUT);
//        touch_ATB.setMode(DigitalChannel.Mode.INPUT);

    }

    public void setInitMotors() {//  INNICIALIZAÇÃO DOS MOTORES E SERVOS
        LMF = hardwareMap.get(DcMotor.class, "LMF");
        LMB = hardwareMap.get(DcMotor.class, "LMB");
        RMF = hardwareMap.get(DcMotor.class, "RMF");
        RMB = hardwareMap.get(DcMotor.class, "RMB");

        MAT = hardwareMap.get(DcMotor.class, "MAT");
        MSE = hardwareMap.get(DcMotor.class, "MSE");
        MRP = hardwareMap.get(DcMotor.class, "MRP");

        servoGarra = hardwareMap.get(Servo.class, "servoGarra");
        servoGarra2 = hardwareMap.get(Servo.class, "servoGarra2");
    }

    public void setInitSensor() {
        distanceSensorF = hardwareMap.get(DistanceSensor.class, "distanceF");
        distanceSensorB = hardwareMap.get(DistanceSensor.class, "distanceB");
//        touch_MRP = hardwareMap.get(DigitalChannel.class,"touch_MRP");
//        touch_ATB = hardwareMap.get(DigitalChannel.class, "touch_ATB");

    }

    public void setDiretion() {//    DIREÇÃO DOS MOTORES E SERVOS
        LMF.setDirection(DcMotorSimple.Direction.REVERSE);      //REVERSE - Direção Reversa
        LMB.setDirection(DcMotorSimple.Direction.REVERSE);
        RMF.setDirection(DcMotorSimple.Direction.FORWARD);      //FORWARD - Direção Normal
        RMB.setDirection(DcMotorSimple.Direction.FORWARD);

        MAT.setDirection(DcMotorSimple.Direction.FORWARD);
        MSE.setDirection(DcMotorSimple.Direction.REVERSE);
        MRP.setDirection(DcMotorSimple.Direction.FORWARD);

        servoGarra.setDirection(Servo.Direction.FORWARD);
        servoGarra2.setDirection(Servo.Direction.FORWARD);
    }

    public void setStatus() {//      STATUS DOS MOTORES E SERVOS
        LMF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RMF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LMB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RMB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        MAT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MSE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MRP.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setEncoders() {//    STATUS DOS ENCODERS
        LMF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);//     INUTILIZANDO
        RMB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RMF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RMB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        ATB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);//       UTILIZANDO
////        PRT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        LIN.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setAllPower(double p) {     //Metodo para definir a mesma força para todos os motores
        setMotorPowerIndiv(p, p, p, p);
    }

//    public void giroEixo(double p){
//
//        if(p >= 0){
//            setMotorPowerIndiv(-p,-p,p,p);
//        }else{
//            setMotorPowerIndiv(p,p,-p,-p);
//        }
//
//    }

    public void setMotorPowerIndiv(double rF, double rB, double lF, double lB) {
        RMF.setPower(rF);
        RMB.setPower(rB);
        LMF.setPower(lF);
        LMB.setPower(lB);
    }

//Processamento de Imagem:

    //-  Cone:
    public void idQRCode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
//            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if (tagOfInterest == null || tagOfInterest.id == LEFT) {
            //trajetoria
        } else if (tagOfInterest.id == MIDDLE) {
            //trajetoria
        } else {
            //trajetoria
        }


    }

    void tagToTelemetry(AprilTagDetection detection) {
        id = detection.id;
//        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
//        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
//        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
//        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
//        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    ////////////////////////////////////////////////////////////Junction Auto////////////////////////////////////////////////////////////////////////////////////////////

        public class ContourPipeline extends OpenCvPipeline {
            Scalar HOT_PINK = new Scalar(196, 23, 112);

            // Use this picture for you own color https://github.com/PinkToTheFuture/OpenCV_FreightFrenzy_2021-2022/blob/main/YCbCr.jpeg
            // Note that the Cr and Cb values range between 0-255. this means that the origin of the coordinate system is (128,128)

            // Volatile because accessed by OpMode without sync
            public volatile boolean error = false;
            public volatile Exception debug;

            private double borderLeftX;     //fraction of pixels from the left side of the cam to skip
            private double borderRightX;    //fraction of pixels from the right of the cam to skip
            private double borderTopY;      //fraction of pixels from the top of the cam to skip
            private double borderBottomY;   //fraction of pixels from the bottom of the cam to skip

            private int CAMERA_WIDTH;
            private int CAMERA_HEIGHT;

            private int loopCounter = 0;
            private int pLoopCounter = 0;

            private final Mat mat = new Mat();
            private final Mat processed = new Mat();

            private Rect maxRect = new Rect(600, 1, 1, 1);

            private double maxArea = 0;
            private boolean first = false;

            private final Object sync = new Object();

            public ContourPipeline(double borderLeftX, double borderRightX, double borderTopY, double borderBottomY) {
                this.borderLeftX = borderLeftX;
                this.borderRightX = borderRightX;
                this.borderTopY = borderTopY;
                this.borderBottomY = borderBottomY;
            }

            public void configureScalarLower(double y, double cr, double cb) {
                scalarLowerYCrCb = new Scalar(y, cr, cb);
            }

            public void configureScalarUpper(double y, double cr, double cb) {
                scalarUpperYCrCb = new Scalar(y, cr, cb);
            }

            public void configureScalarLower(int y, int cr, int cb) {
                scalarLowerYCrCb = new Scalar(y, cr, cb);
            }

            public void configureScalarUpper(int y, int cr, int cb) {
                scalarUpperYCrCb = new Scalar(y, cr, cb);
            }

            public void configureBorders(double borderLeftX, double borderRightX, double borderTopY, double borderBottomY) {
                this.borderLeftX = borderLeftX;
                this.borderRightX = borderRightX;
                this.borderTopY = borderTopY;
                this.borderBottomY = borderBottomY;
            }


            @Override
            public Mat processFrame(Mat input) {
                CAMERA_WIDTH = input.width();
                CAMERA_HEIGHT = input.height();
                try {
                    // Process Image
                    Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb);
                    Core.inRange(mat, scalarLowerYCrCb, scalarUpperYCrCb, processed);
                    // Remove Noise
                    Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_OPEN, new Mat());
                    Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_CLOSE, new Mat());
                    // GaussianBlur
                    Imgproc.GaussianBlur(processed, processed, new Size(5.0, 15.0), 0.00);
                    // Find Contours
                    List<MatOfPoint> contours = new ArrayList<>();
                    Imgproc.findContours(processed, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

                    // Draw Contours
                    Imgproc.drawContours(input, contours, -1, new Scalar(255, 0, 0));

                    // Lock this up to prevent errors when outside threads access the max rect property.
                    synchronized (sync) {
                        // Loop Through Contours
                        for (MatOfPoint contour : contours) {
                            Point[] contourArray = contour.toArray();

                            // Bound Rectangle if Contour is Large Enough
                            if (contourArray.length >= 15) {
                                MatOfPoint2f areaPoints = new MatOfPoint2f(contourArray);
                                Rect rect = Imgproc.boundingRect(areaPoints);

                                if (rect.area() > maxArea
                                        && rect.x + (rect.width / 2.0) > (borderLeftX * CAMERA_WIDTH)
                                        && rect.x + (rect.width / 2.0) < CAMERA_WIDTH - (borderRightX * CAMERA_WIDTH)
                                        && rect.y + (rect.height / 2.0) > (borderTopY * CAMERA_HEIGHT)
                                        && rect.y + (rect.height / 2.0) < CAMERA_HEIGHT - (borderBottomY * CAMERA_HEIGHT)

                                        || loopCounter - pLoopCounter > 6
                                        && rect.x + (rect.width / 2.0) > (borderLeftX * CAMERA_WIDTH)
                                        && rect.x + (rect.width / 2.0) < CAMERA_WIDTH - (borderRightX * CAMERA_WIDTH)
                                        && rect.y + (rect.height / 2.0) > (borderTopY * CAMERA_HEIGHT)
                                        && rect.y + (rect.height / 2.0) < CAMERA_HEIGHT - (borderBottomY * CAMERA_HEIGHT)
                                ) {
                                    maxArea = rect.area();
                                    maxRect = rect;
                                    pLoopCounter++;
                                    loopCounter = pLoopCounter;
                                    first = true;
                                } else if (loopCounter - pLoopCounter > 10) {
                                    maxArea = new Rect().area();
                                    maxRect = new Rect();
                                }

                                areaPoints.release();
                            }
                            contour.release();
                        }
                        if (contours.isEmpty()) {
                            maxRect = new Rect(600, 1, 1, 1);
                        }
                    }
                    // Draw Rectangles If Area Is At Least 500
                    if (first && maxRect.area() > 500) {
                        Imgproc.rectangle(input, maxRect, new Scalar(0, 255, 0), 2);
                    }
                    // Draw Borders
                    Imgproc.rectangle(input, new Rect(
                            (int) (borderLeftX * CAMERA_WIDTH),
                            (int) (borderTopY * CAMERA_HEIGHT),
                            (int) (CAMERA_WIDTH - (borderRightX * CAMERA_WIDTH) - (borderLeftX * CAMERA_WIDTH)),
                            (int) (CAMERA_HEIGHT - (borderBottomY * CAMERA_HEIGHT) - (borderTopY * CAMERA_HEIGHT))
                    ), HOT_PINK, 2);

                    // Display Data
                    Imgproc.putText(input, "Area: " + getRectArea() + " Midpoint: " + getRectMidpointXY().x + " , " + getRectMidpointXY().y, new Point(5, CAMERA_HEIGHT - 5), 0, 0.6, new Scalar(255, 255, 255), 2);
                    loopCounter++;
                } catch (Exception e) {
                    debug = e;
                    error = true;
                }
                return input;
            }

            public int getRectHeight() {
                synchronized (sync) {
                    return maxRect.height;
                }
            }

            public int getRectWidth() {
                synchronized (sync) {
                    return maxRect.width;
                }
            }

            public int getRectX() {
                synchronized (sync) {
                    return maxRect.x;
                }
            }

            public int getRectY() {
                synchronized (sync) {
                    return maxRect.y;
                }
            }

            public double getRectMidpointX() {
                synchronized (sync) {
                    return getRectX() + (getRectWidth() / 2.0);
                }
            }

            public double getRectMidpointY() {
                synchronized (sync) {
                    return getRectY() + (getRectHeight() / 2.0);
                }
            }

            public Point getRectMidpointXY() {
                synchronized (sync) {
                    return new Point(getRectMidpointX(), getRectMidpointY());
                }
            }

            public double getAspectRatio() {
                synchronized (sync) {
                    return getRectArea() / (CAMERA_HEIGHT * CAMERA_WIDTH);
                }
            }

            public double getRectArea() {
                synchronized (sync) {
                    return maxRect.area();
                }
            }
        }
    }
