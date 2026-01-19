package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "AUTO ODO POD X Y (SEM IMU)", group = "CORTEX")
public class odo1 extends LinearOpMode {

    DcMotor FL0; // POD eixo Y
    DcMotor FR1;
    DcMotor BL2; // POD eixo X
    DcMotor BR3;

    static final int TOLERANCIA_X = 40;
    static final int TARGET_Y = 5000;

    static final double TICKS_POR_GRAU = 18.0; // 🔧 CALIBRAR
    static final double GIRO_POWER = 0.5;

    @Override
    public void runOpMode() {

        FL0 = hardwareMap.get(DcMotor.class, "FL0");
        FR1 = hardwareMap.get(DcMotor.class, "FR1");
        BL2 = hardwareMap.get(DcMotor.class, "BL2");
        BR3 = hardwareMap.get(DcMotor.class, "BR3");

        FL0.setDirection(DcMotor.Direction.FORWARD);
        BL2.setDirection(DcMotor.Direction.FORWARD);
        FR1.setDirection(DcMotor.Direction.REVERSE);
        BR3.setDirection(DcMotor.Direction.REVERSE);

        FR1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FL0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Pronto - Odometria pura (sem IMU)");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        int yInicial = FL0.getCurrentPosition();
        int xInicial = BL2.getCurrentPosition();

        // =============================
        // 1️⃣ ANDAR PARA TRÁS
        // =============================
        while (opModeIsActive() &&
                Math.abs(FL0.getCurrentPosition() - yInicial) < TARGET_Y) {

            FL0.setPower(-0.8);
            FR1.setPower(-0.8);
            BL2.setPower(-0.8);
            BR3.setPower(-0.8);
        }

        stopMotors();
        sleep(200);

        // =============================
        // 2️⃣ CORRIGIR X
        // =============================
        int erroX = BL2.getCurrentPosition() - xInicial;

        while (opModeIsActive() && Math.abs(erroX) > TOLERANCIA_X) {

            double direcao = erroX > 0 ? -0.4 : 0.4;

            FL0.setPower(direcao);
            FR1.setPower(-direcao);
            BL2.setPower(-direcao);
            BR3.setPower(direcao);

            erroX = BL2.getCurrentPosition() - xInicial;
        }

        stopMotors();
        sleep(300);

        // =============================
        // 3️⃣ GIRAR 15° PARA DIREITA (SEM IMU)
        // =============================
        girarDireitaOdo(15);

        stopMotors();
        telemetry.addLine("Autônomo finalizado");
        telemetry.update();
    }

    // =============================
    // 🔄 GIRO POR ODOMETRIA
    // =============================
    private void girarDireitaOdo(double graus) {

        int startY = FL0.getCurrentPosition();
        int startX = BL2.getCurrentPosition();

        int targetTicks = (int) (graus * TICKS_POR_GRAU);

        while (opModeIsActive()) {

            int deltaY = Math.abs(FL0.getCurrentPosition() - startY);
            int deltaX = Math.abs(BL2.getCurrentPosition() - startX);

            int media = (deltaY + deltaX) / 2;

            if (media >= targetTicks) break;

            // 🔄 giro horário (direita)
            FL0.setPower( GIRO_POWER);
            BL2.setPower(-GIRO_POWER);

            FR1.setPower(-GIRO_POWER);
            BR3.setPower( GIRO_POWER);
        }

        stopMotors();
        sleep(200);
    }

    private void stopMotors() {
        FL0.setPower(0);
        FR1.setPower(0);
        BL2.setPower(0);
        BR3.setPower(0);
    }
}
