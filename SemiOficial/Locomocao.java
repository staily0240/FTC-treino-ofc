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

package org.firstinspires.ftc.teamcode.SemiOficial;

public class Locomocao{

    final double PI = Math.PI;

    double y;
    double x;
    double x2;

    double d;

    boolean povDown;

    double speed;

    double angulofinalE;
    double angulorealE;

    double angulorad;
    double angulo;

    double anguloRobo;

    double switchAngle;

    double fMD = 0;
    double fME = 0;

    double forcax = 0;
    double forcay = 0;

    double forceRF;
    double forceRB;
    double forceLF;
    double forceLB;

    public Locomocao(double y, double x, double x2, double angle, double speed, boolean povDown){
        this.y = y;
        this.x = x;
        this.x2 = x2;
        this.povDown = povDown;
        this.anguloRobo = angle;
        this.speed = speed;
    }

        public double[] runlocomocao(){

            d = Math.hypot(x, y);

            angulorad = (d != 0) ? Math.asin(y / d) : 0;
            angulo = Math.toDegrees(angulorad);

            //  Control Speed

            switchAngle = povDown ? anguloRobo : switchAngle;

            getAngJoyL();

            formMovAng();

            movAng();

            setPower();

            double powers[] = {forceRF,forceRB,forceLF,forceLB};
            return powers;
        }

        public void setPower(){//                                  DEFINIR A FORÇA DOS MOTORES
            forceRF = ((fMD-x2) * speed);
            forceRB = ((fME-x2) * speed);
            forceLF = ((fME+x2) * speed);
            forceLB = ((fMD+x2) * speed);
        }

        public void movAng(){//                   DEFINIÇÃO FORÇAS POR QUADRANTES
            angulorad = (d!=0)?Math.asin(forcay/d):0;

            //1º QUADRANTE FORÇA
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

        public void formMovAng(){//              FORMULAS DOS EIXOS

            //Analógico Esquerdo
            angulorealE =  angulofinalE - anguloRobo + switchAngle;
            forcay = Math.sin(Math.toRadians(angulorealE)) * d;
            forcax = Math.cos(Math.toRadians(angulorealE)) * d;

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

    }