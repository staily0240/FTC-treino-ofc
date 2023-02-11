package org.firstinspires.ftc.teamcode.Oficial;

import  com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class OnlyMainCode extends LinearOpMode {

    public DcMotor  RMF;    //Porta 0 - Motor Mecanum - Direita Dianteiro   Odometria Vertical
    public DcMotor  RMB;    //Porta 1 - Motor Mecanum - Direita Traseiro    Odometria Horizontal
    public DcMotor  LMF;    //Porta 2 - Motor Mecanum - Esquerda Dianteiro
    public DcMotor  LMB;    //Porta 3 - Motor Mecanum - Esquerda Traseiro

    public DcMotor MAT;     //Porta 0 - Motor Core Hex - Antebraço
    public DcMotor MSE;     //Porta 2 - Motor UltraPlanetary - Sistema de Elevação
    public DcMotor MRP;     //Porta 3 - Motor Core Hex - Rotação Prato

    public Servo servoGarra;    //Porta 0 - Direita
    public Servo servoGarra2;   //Porta 1 - Esquerda

    BNO055IMU imu;
    public DigitalChannel touch_MRP;
    public DistanceSensor distanceSensorF;
    public DistanceSensor distanceSensorB;

    double[] powersLoc, powersSist;

//Variáveis - Locomoção

    double speedLoc = 0, confiSpeedLoc = 0;
    boolean boolConfigSpeedLoc = true;
    double switchAngle = 0;

    double y;
    double x;

    double x2;

    double d;

    double angulorad;
    double angulo;

    double angulofinalE;
    double angulorealE = 0;

    final double PI = Math.PI;

    double fMD = 0;
    double fME = 0;

    double forcax = 0;
    double forcay = 0;

    int typeLoc = 0;
    boolean boolTypeLoc = true;

    double forceRF;
    double forceRB;
    double forceLF;
    double forceLB;

//Variaveis - Sistema

    double y_left;
    double y_right;

    //Sistema de Elevação
    double currentPosition_MSE, controleEncoder_MSE, targetPosition_MSE, controleForce_MSE, max_MSE = -7000;
    boolean runMaxPosition_MSE = false, controleMaxPosition_MSE = true, controleMaxPosition_MSE2 = true;

    //Prato
    double currentPosition_PRT, targetPosition_PRT = 0, zeroEncoder_PRT = 0, alvo_PRT = 0;

    double controleManual_PRT;

    double PID_PRT;
    double proporcional_PRT, integral_PRT, derivada_PRT;
    double kP_PRT = 0.00065, kI_PRT = 0.00085, kD_PRT = 0.0002;

    double erro_PRT, ultErro_PRT;

    //Antebraço
    double speedAtb=  0, confiSpeedAtb = 1, analogForceATB;
    boolean boolConfigSpeedAtb = true;

    double currentPosition_ATB, ultCurrentPosition_ATB, erroATB2, targetPosition_ATB, controleEncoder_ATB, max_ATB = 1950, min_ATB = -300;
    boolean runMaxPosition_ATB = false, controleMaxPosition_ATB = true, controleMaxPosition_ATB2 = true;

    double PID_ATB;
    double proporcional_ATB, integral_ATB, derivada_ATB;
    double kP_ATB = 0.0008, kI_ATB = 0.00085, kD_ATB = 0;

    double erro_ATB, ultErro_ATB;

    //Force
    double forceATB = 0;
    double forcePRT = 0;
    double forceLIN = 0;

    //Garra
    static final double MAX_POS_D     =  0.2;     // Maximum rotational position 0.5
    static final double MIN_POS_D     =  0.0;     // Minimum rotational position    certo

    static final double MAX_POS_E     =  0.1;     // Maximum rotational position
    static final double MIN_POS_E     =  0.4;     // Minimum rotational position 0.5

    double  position_D = MIN_POS_D; // Start at halfway position
    double  position_E = MIN_POS_E; // Start at halfway position

    double timer;
    double timer2;

    @Override
    public void runOpMode() {

        resetRuntime();

        initHardware();

        controleEncoder_ATB = MAT.getCurrentPosition();
        controleEncoder_MSE = MSE.getCurrentPosition();

        waitForStart();

        while (opModeIsActive()) {

            powersLoc = runlocomocao();
            powersSist = runsistemas();

            RMF.setPower(powersLoc[0]);
            RMB.setPower(powersLoc[1]);
            LMF.setPower(powersLoc[2]);
            LMB.setPower(powersLoc[3]);

            MAT.setPower(powersSist[0]);
            MSE.setPower(powersSist[1]);
            MRP.setPower(powersSist[2]);

            Prints();

        }
    }


//Locomoção

    public double[] runlocomocao(){

        y = -gamepad1.left_stick_y;
        x = gamepad1.left_stick_x;

        x2 = gamepad1.right_stick_x;

        d = Math.hypot(x,y);

        angulorad = (d!=0)?Math.asin(y/d):0;
        angulo = Math.toDegrees(angulorad);

        angulorad = (d!=0)?Math.asin(y/d):0;
        angulo = Math.toDegrees(angulorad);

        configSpeed();

        switchAngle = gamepad1.dpad_down ? getAngle() : switchAngle;
        switchAngle = (typeLoc%2 != 0) ? getAngle() : switchAngle;

        getAngJoyL();

        movAng();

        formMovAng();

        selectTypeLoc();

        setPowerLoc();

        return new double[]{forceRF,forceRB,forceLF,forceLB};
    }

    public void configSpeed(){

        if((gamepad1.left_stick_button || gamepad1.right_stick_button) && boolConfigSpeedLoc){
            if(gamepad1.left_stick_button){
                confiSpeedLoc++;
            }
            if(gamepad1.right_stick_button){
                confiSpeedLoc--;
            }
            boolConfigSpeedLoc = false;
        }
        if(!gamepad1.left_stick_button && !gamepad1.right_stick_button && !boolConfigSpeedLoc){
            boolConfigSpeedLoc = true;
        }

        confiSpeedLoc = (confiSpeedLoc > 2) ? 0 : (confiSpeedLoc < 0) ? 2 : confiSpeedLoc;

        confiSpeedLoc = gamepad1.b ? 0 : gamepad1.a ? 1 : gamepad1.x ? 2 : confiSpeedLoc;

        speedLoc = (confiSpeedLoc == 0) ? 1 : (confiSpeedLoc == 1) ? 0.5 : (confiSpeedLoc == 2) ? 0.25 : speedLoc;

    }

    public void getAngJoyL(){//                  ÂNGULO DO JOY DIREITO

        //1º QUADRANTE
        if((angulo >= 0) && (angulo <= 90) && (x >= 0)) {
            angulofinalE = angulo;
        }
        //2º QUADRANTE
        else if ((angulo >= 0) && (angulo <= 90) && (x <= 0)) {
            angulofinalE = 180-angulo;
        }
        //3º QUADRANTE
        else if ((angulo <= 0) && (angulo >= -90) && (x <= 0)) {
            angulofinalE = -angulo-180;
        }
        //4º QUADRANTE
        else if ((angulo <= 0) && (angulo >= -90) && (x >= 0)) {
            angulofinalE = angulo;
        }

    }

    public void movAng(){//                   DEFINIÇÃO FORÇAS POR QUADRANTES
        double anguloRobo;
        // MOVIMENTAÇÃO POR ÂNGULO

        anguloRobo = getAngle();
        //Analógico Esquerdo
        angulorealE =  angulofinalE - anguloRobo + switchAngle;
        forcay = Math.sin(Math.toRadians(angulorealE)) * d;
        forcax = Math.cos(Math.toRadians(angulorealE)) * d;

    }

    public void formMovAng(){//              FORMULAS DOS EIXOS

        //1º QUADRANTE FORÇA

        angulorad = (d!=0)?Math.asin(forcay/d):0;

        if((forcax>= 0) && (forcay>=0)) {
            fME = d;
            fMD = (((4 / PI) * angulorad) - 1) * d;

        }
        //2º QUADRANTE FORÇA
        else if ((forcax< 0) && (forcay>=0)) {
            fME = (((4 / PI) * angulorad) - 1) * d;
            fMD = d;
        }
        //3º QUADRANTE FORÇA
        else if ((forcax< 0) && (forcay<0)) {
            fME = -d;
            fMD = (((4 / PI) * angulorad) + 1) * d ;
        }
        //4º QUADRANTE FORÇA
        else if ((forcax>= 0) && (forcay<0)) {
            fME = (((4 / PI) * angulorad) + 1) * d;
            fMD = -d;
        }

    }

    public void selectTypeLoc(){

        if(gamepad1.dpad_up && boolTypeLoc){
            typeLoc++;
            boolTypeLoc = false;
        }
        if(!gamepad1.dpad_up && !boolTypeLoc){boolTypeLoc = true;}

    }

    public void setPowerLoc(){//                                  DEFINIR A FORÇA DOS MOTORES
//        if(typeLoc%2 == 0) {
            forceRF = ((fMD - x2) * speedLoc);
            forceRB = ((fME - x2) * speedLoc);
            forceLF = ((fME + x2) * speedLoc);
            forceLB = ((fMD + x2) * speedLoc);
//        }else{
//            forceRF = ((y - x - x2) * speedLoc);
//            forceRB = ((y + x - x2) * speedLoc);
//            forceLF = ((y + x + x2) * speedLoc);
//            forceLB = ((y - x + x2) * speedLoc);
//
//        }
    }

//Sistemas

    public double[] runsistemas(){

        timer = getRuntime() - timer2;
        y_left = -gamepad2.left_stick_y;
        y_right = -gamepad2.right_stick_y;
        currentPosition_PRT = MRP.getCurrentPosition();
        currentPosition_MSE = MSE.getCurrentPosition();
        currentPosition_ATB = MAT.getCurrentPosition();

        currentPosition_ATB = Math.abs(currentPosition_ATB) -  Math.abs(controleEncoder_ATB);
//        currentPosition_MSE = Math.abs(currentPosition_MSE) -  Math.abs(controleEncoder_MSE);

        statusATB();

        configSpeed_MRP();

        setServoGarra();

        setTarget_MRP();

//        setTarget_MSE();

        erro_ATB = targetPosition_ATB - currentPosition_ATB;
        erro_PRT = targetPosition_PRT - currentPosition_PRT;

        setPID_ATB();

        setPID_PRT();

        limiteSistElevacao();

        limiteAntebraco();

        ultErro_ATB = erro_ATB;
        ultErro_PRT = erro_PRT;

        setPowerSist();

        timer2 = getRuntime();


        return new double[]{forceATB, forceLIN, forcePRT};
    }

    private void statusATB() {
//        if(forceATB == 0){
//            MAT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        }else{
//            MAT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        }
    }

    public void setTarget_MRP(){

        alvo_PRT =
            (gamepad2.dpad_up) ? 0 :

            (gamepad2.dpad_right) ? 2000 :

            (gamepad2.dpad_left) ? -2000 :

            alvo_PRT;

        alvo_PRT =
            ((currentPosition_PRT > 0) && (gamepad2.dpad_down)) ? 4000 :

            ((currentPosition_PRT < 0) && (gamepad2.dpad_down)) ? -4000 :

            alvo_PRT;

        alvo_PRT =
            (gamepad2.dpad_right && gamepad2.dpad_up) ? 1000 :
            (gamepad2.dpad_right && gamepad2.dpad_down) ? 3000 :

            (gamepad2.dpad_left && gamepad2.dpad_up) ? -1000 :
            (gamepad2.dpad_left && gamepad2.dpad_down) ? -3000 :

            alvo_PRT;

        resetEncoder("MRP");

        targetPosition_PRT = alvo_PRT < 0 ? alvo_PRT - zeroEncoder_PRT : alvo_PRT + zeroEncoder_PRT;

        if (gamepad2.right_trigger > 0.1 || gamepad2.left_trigger > 0.1){
            targetPosition_PRT = currentPosition_PRT;
        }

    }

    public void configSpeed_MRP(){

        if(gamepad2.left_stick_button && boolConfigSpeedAtb){
            confiSpeedAtb++;
            boolConfigSpeedAtb = false;
        }
        if(!gamepad2.left_stick_button && !boolConfigSpeedAtb){
            boolConfigSpeedAtb = true;
        }

        confiSpeedAtb = (confiSpeedAtb > 2) ? 0 : (confiSpeedAtb < 0) ? 2 : confiSpeedAtb;

        confiSpeedAtb = gamepad2.b ? 0 : gamepad2.a ? 1 : gamepad2.x ? 2 : confiSpeedAtb;

        speedAtb = (confiSpeedAtb == 0) ? 1 : (confiSpeedAtb == 1) ? 0.5 : (confiSpeedAtb == 2) ? 0.25 : speedAtb;

    }

    public void setPID_PRT(){

        proporcional_PRT = erro_PRT * kP_PRT;
        integral_PRT += erro_PRT * kI_PRT * timer;
        derivada_PRT = 0;

        if(Math.abs(erro_PRT) < 600) {// Erro - Não Passou

            derivada_PRT = (erro_PRT - ultErro_PRT) / timer * kD_PRT;

        }

        derivada_PRT = Math.abs(derivada_PRT) > 0.2 ? (derivada_PRT > 0 ? 0.2 : - 0.2) : derivada_PRT;

        if(gamepad2.dpad_up || gamepad2.dpad_right || gamepad2.dpad_left || gamepad2.dpad_down){
            integral_PRT = 0;
        }

        PID_PRT = proporcional_PRT + integral_PRT + derivada_PRT;

        PID_PRT = Math.abs(PID_PRT) < 0.1 ? 0 : PID_PRT;

        controleManual_PRT = ((gamepad2.right_trigger - gamepad2.left_trigger) * 0.35);

    }

    public void limiteAntebraco(){

        erroATB2 = Math.abs(ultCurrentPosition_ATB) - Math.abs(currentPosition_ATB);

        min_ATB = (currentPosition_ATB < 0 && erroATB2 < 10) ? currentPosition_ATB : min_ATB;

        analogForceATB = y_right;

        if(gamepad2.right_stick_button){
            max_ATB = currentPosition_ATB;
        }
        else if((currentPosition_ATB < min_ATB && analogForceATB < 0) || (currentPosition_ATB > max_ATB && analogForceATB > 0)){
            analogForceATB = 0;
            PID_ATB = 0;
        }

        ultCurrentPosition_ATB = currentPosition_ATB;
    }

    public void setPID_ATB(){

        if(gamepad2.y && controleMaxPosition_ATB){
            if(runMaxPosition_ATB){runMaxPosition_ATB = false;}else{runMaxPosition_ATB = true;}
            controleMaxPosition_ATB = false;
        }
        if(!gamepad2.y && !controleMaxPosition_ATB){ controleMaxPosition_ATB = true;}

        targetPosition_ATB = runMaxPosition_ATB ? max_ATB : currentPosition_ATB;

        if(!runMaxPosition_ATB ){
            if(analogForceATB < 0.1 && controleMaxPosition_ATB2){
                targetPosition_ATB = currentPosition_ATB;
                controleMaxPosition_ATB2 = false;
            }
            if(analogForceATB >= 0.1 && !controleMaxPosition_ATB2){
                controleMaxPosition_ATB2 = true;
            }
        }

        proporcional_ATB = erro_ATB * kP_ATB;
        integral_ATB += erro_ATB * kI_ATB;
        derivada_ATB = (erro_ATB - ultErro_ATB) * kD_ATB;

        PID_ATB = runMaxPosition_ATB ? (proporcional_ATB + integral_ATB + derivada_ATB) : 0;

    }

    public void limiteSistElevacao(){

        max_MSE = gamepad2.left_stick_button ? currentPosition_MSE : max_MSE;

        controleForce_MSE = -y_left;

        if(gamepad2.y && controleMaxPosition_MSE){
            if(runMaxPosition_MSE){runMaxPosition_MSE = false;}else{runMaxPosition_MSE = true;}
            controleMaxPosition_MSE = false;
        }
        if(!gamepad2.y && !controleMaxPosition_MSE){ controleMaxPosition_MSE = true;}

        if(runMaxPosition_MSE){
            controleForce_MSE = (Math.abs(currentPosition_MSE) < Math.abs(max_MSE)) ? -1 : controleForce_MSE;
        }
        if(!runMaxPosition_MSE && gamepad2.y){
            controleForce_MSE = (Math.abs(currentPosition_MSE) >50) ? 1 : controleForce_MSE;
        }

        if(((currentPosition_MSE >= 0 && y_left < 0) || (currentPosition_MSE <= max_MSE && y_left > 0)) && (!gamepad2.left_stick_button)){
            controleForce_MSE = 0;
        }

    }

    public void setServoGarra(){

        if(gamepad2.right_bumper){
            position_D = MAX_POS_D;
            position_E = MAX_POS_E;
        }

        if(gamepad2.left_bumper){
            position_D = MIN_POS_D;
            position_E = MIN_POS_E;
        }

        servoGarra.setPosition(position_D);              //Direita
        servoGarra2.setPosition(position_E);             //Esquerda
    }

    public void setPowerSist(){
        forcePRT = PID_PRT + controleManual_PRT;
        forceLIN = controleForce_MSE;
        forceATB = ((analogForceATB * speedAtb) + PID_ATB); // PID_ATB
    }

//Sensor's

    public double getAngle(){//     RECEBER O ÂNGULO REFERENCIAL
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return orientation.firstAngle;
    }

    public boolean getTouch(int touch){
        switch (touch){
            case 0:
//                return !(touch_ATB.getState());
            case 3:
                return !(touch_MRP.getState());
        }
        return false;
    }

    public void resetEncoder(String motor){

        switch (motor){
            case "MAT":
                break;
            case "MSE":
                break;
            case "MRP":
                if(getTouch(3)){
                    zeroEncoder_PRT = currentPosition_PRT;
                }
                break;
            default:
        }
    }

//Hardware Pushbot

    public void initHardware() {

        setInitMotors();

        setInitSensor();

        setDiretion();

        setStatus();

        setEncoders();



        setAllPower(0);     //Definindo força 0 para todos os motores

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void setInitMotors(){//  INNICIALIZAÇÃO DOS MOTORES E SERVOS
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

    public void setInitSensor(){
        distanceSensorF = hardwareMap.get(DistanceSensor.class, "distanceF");
        distanceSensorB = hardwareMap.get(DistanceSensor.class, "distanceB");
        touch_MRP = hardwareMap.get(DigitalChannel.class,"touch_MRP");

    }

    public void setDiretion(){//    DIREÇÃO DOS MOTORES E SERVOS
        LMF.setDirection(DcMotorSimple.Direction.REVERSE);      //REVERSE - Direção Reversa
        LMB.setDirection(DcMotorSimple.Direction.REVERSE);
        RMF.setDirection(DcMotorSimple.Direction.FORWARD);      //FORWARD - Direção Normal
        RMB.setDirection(DcMotorSimple.Direction.FORWARD);

        MAT.setDirection(DcMotorSimple.Direction.REVERSE);
        MSE.setDirection(DcMotorSimple.Direction.FORWARD);
        MRP.setDirection(DcMotorSimple.Direction.FORWARD);

        servoGarra.setDirection(Servo.Direction.FORWARD);
        servoGarra2.setDirection(Servo.Direction.FORWARD);
    }

    public void setStatus(){//      STATUS DOS MOTORES E SERVOS
        LMF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RMF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LMB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RMB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        MAT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MSE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MRP.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setEncoders(){//    STATUS DOS ENCODERS
        RMF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);//     INUTILIZANDO
        RMB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LMF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LMB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setAllPower(double p){     //Metodo para definir a mesma força para todos os motores
        setMotorPowerIndiv(p,p,p,p,p,p,p);
    }

    public void setMotorPowerIndiv(double rF, double rB, double lF,  double lB, double atb, double lin, double prt){
        RMF.setPower(rF);
        RMB.setPower(rB);
        LMF.setPower(lF);
        LMB.setPower(lB);

        MAT.setPower(atb);
        MSE.setPower(lin);
        MRP.setPower(prt);
    }

//Print's

    public void Prints(){

        telemetry.addLine("\n-------------------------------------------------\n");

        telemetry.addLine("Locomoção:");//                                    Locomoção
        telemetry.addData("Type Loc", typeLoc%2);
        telemetry.addData("Speed Control", confiSpeedLoc);

        telemetry.addLine("Sensor:");//                                    Sensor's
        telemetry.addData("Touch:\t", getTouch(3));
        telemetry.addData("Distance Frente:\t", distanceSensorF.getDistance(DistanceUnit.CM));
        telemetry.addData("Distance Tras:\t", distanceSensorB.getDistance(DistanceUnit.CM));
        telemetry.addData("Odometria Vertical", LMF.getCurrentPosition());
        telemetry.addData("Odometria Horizontal", LMB.getCurrentPosition());

        telemetry.addLine("\n-------------------------------------------------\n");

        telemetry.addLine("Servo:");//                                     Force
        telemetry.addData("Garra:\t", servoGarra.getPosition());
        telemetry.addData("Garra2:\t", servoGarra2.getPosition());

        telemetry.addLine("\n-------------------------------------------------\n");

        telemetry.addLine("Motor:");//                                     Force
        telemetry.addData("Antebraço:\t", MAT.getPower());
        telemetry.addData("Linear:\t", MSE.getPower());
        telemetry.addData("Prato:\t", MRP.getPower());

        telemetry.addLine("\n-------------------------------------------------\n");

        telemetry.addLine("Encoder - Prato:");//                           Encoder's
        telemetry.addData("Atual:\t", currentPosition_PRT);
        telemetry.addData("Alvo:\t", targetPosition_PRT);
        telemetry.addData("Erro:\t", erro_PRT);
        telemetry.addData("Reset Alvo:\t", zeroEncoder_PRT);


        telemetry.addData("\nPID:\t", PID_PRT);
        telemetry.addData("Proporcional:\t", proporcional_PRT);
        telemetry.addData("Integral:\t", integral_PRT);
        telemetry.addData("Derivada:\t", derivada_PRT);

        telemetry.addLine("\n-------------------------------------------------\n");

        telemetry.addLine("Encoder - Antebraço:");
        telemetry.addData("Atual robo:\t", MAT.getCurrentPosition());
        telemetry.addData("Atual nosso:\t", currentPosition_ATB);
        telemetry.addData("Alvo:\t", targetPosition_ATB);
        telemetry.addData("Erro:\t", erro_ATB);

        telemetry.addData("\nMax:\t", max_ATB);
        telemetry.addData("Min:\t", min_ATB);
        telemetry.addData("Button:\t", gamepad2.left_stick_button);
        telemetry.addData("Erro:\t", erroATB2);


        telemetry.addData("\nPID:\t", PID_ATB);
        telemetry.addData("Proporcional:\t", proporcional_ATB);
        telemetry.addData("Integral:\t", integral_ATB);
        telemetry.addData("Derivada:\t", derivada_ATB);

        telemetry.addLine("\n-------------------------------------------------\n");

        telemetry.addLine("Encoder - Sistema de Elevação:");
        telemetry.addData("Atual:\t", currentPosition_MSE);

        telemetry.update();

    }
}
