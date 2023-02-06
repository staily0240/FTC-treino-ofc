package org.firstinspires.ftc.teamcode.SemiOficial;

public class Sistemas {

    double y_left;
    double y_right;

    double speedLin;

    double currentPosition_PRT, targetPosition_PRT;

    double PID_PRT;
    double proporcional_PRT, integral_PRT, derivada_PRT;
    double kP_PRT = 0.00045, kI_PRT = 0.00045, kD_PRT = 0.002;

    double erro_PRT, ultErro_PRT;


    double currentPosition_ATB, targetPosition_ATB;

    double PID_ATB;
    double proporcional_ATB, integral_ATB, derivada_ATB;
    double kP_ATB = 0, kI_ATB = 0, kD_ATB = 0;

    double erro_ATB, ultErro_ATB;

    double forceATB;
    double forcePRT;
    double forceLIN;

    public Sistemas(
            double y_left, double y_right, double speedLin,
            double currentPosition_PRT, double targetPosition_PRT,
            double currentPosition_ATB, double targetPosition_ATB
    ){
        this.y_left = y_left;
        this.y_right = y_right;
        this.speedLin = speedLin;
        this.currentPosition_PRT = currentPosition_PRT;
        this.targetPosition_PRT = targetPosition_PRT;
    }

    public double[] runsistemas(){

        erro_ATB = targetPosition_ATB - currentPosition_ATB;
        erro_PRT = targetPosition_PRT - currentPosition_PRT;

        setPID_ATB();

        setPID_PRT();

        ultErro_ATB = erro_ATB;
        ultErro_PRT = erro_PRT;

        setPower();

        double powers[]={forceATB, forceLIN, forcePRT, PID_ATB, erro_ATB, proporcional_ATB, integral_ATB, derivada_ATB};
        return powers;
    }

    public void setPID_PRT(){

        proporcional_PRT = erro_PRT * kP_PRT;
        integral_PRT = 0;
        derivada_PRT = 0;

        if(targetPosition_PRT < 0) { // - 2000

            if(erro_PRT > -200) {// Erro - Não Passou

                integral_PRT += erro_PRT * kI_PRT;

            }

            if(erro_PRT > -150 && erro_PRT < 100) {// Erro - Não Passou

                derivada_PRT = (erro_PRT - ultErro_PRT) * kD_PRT;

            }

        }else{ // + 2000

            if(erro_PRT < 200) {// Erro - Não Passou

                integral_PRT += erro_PRT * kI_PRT;

            }

            if(erro_PRT < 150 && erro_PRT > -100) {// Erro - Não Passou

                derivada_PRT = (erro_PRT - ultErro_PRT) * kD_PRT;

            }

        }

        PID_PRT = proporcional_PRT + integral_PRT + derivada_PRT;

    }

    public void setPID_ATB(){

        proporcional_ATB = erro_ATB * kP_ATB;
        integral_ATB += erro_ATB * kI_ATB;
        derivada_ATB = (erro_ATB - ultErro_ATB) * kD_ATB;

        PID_ATB = proporcional_ATB + integral_ATB + derivada_ATB;

    }

    public void setPower(){
        forcePRT = PID_PRT;
        forceLIN = y_right * speedLin;
        forceATB = y_left * speedLin;
    }

}
