package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Leitura Encoder Pods 0 e 2", group = "Teste")
public class OD extends LinearOpMode {

    // ===== MOTORES =====
    DcMotor FL0; // POD 0
    DcMotor BL2; // POD 2

    int pos0Inicial;
    int pos2Inicial;

    @Override
    public void runOpMode() {

        // Pods conectados nos ports dos motores
        FL0 = hardwareMap.get(DcMotor.class, "FL0");
        BL2 = hardwareMap.get(DcMotor.class, "BL2");

        // Nunca movimentar
        FL0.setPower(0);
        BL2.setPower(0);

        // Reset só para zerar leitura
        FL0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Encoder apenas para leitura
        FL0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Pronto - Empurre o robô com a mão");
        telemetry.update();

        waitForStart();

        pos0Inicial = FL0.getCurrentPosition();
        pos2Inicial = BL2.getCurrentPosition();

        while (opModeIsActive()) {

            int delta0 = FL0.getCurrentPosition() - pos0Inicial;
            int delta2 = BL2.getCurrentPosition() - pos2Inicial;

            int mediaTicks = (delta0 + delta2) / 2;

            telemetry.addData("Pod 0 (FL0) ticks", delta0);
            telemetry.addData("Pod 2 (BL2) ticks", delta2);
            telemetry.addData("Ticks médios", mediaTicks);

            if (mediaTicks > 0)
                telemetry.addLine("Sentido: FRENTE");
            else if (mediaTicks < 0)
                telemetry.addLine("Sentido: TRÁS");
            else
                telemetry.addLine("Sentido: PARADO");

            telemetry.update();
        }
    }
}
