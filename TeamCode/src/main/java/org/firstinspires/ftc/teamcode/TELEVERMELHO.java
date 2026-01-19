package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;

@TeleOp(name = "TELEOP FINAL - CORTEX VERMELHO", group = "CORTEX")
public class TELEVERMELHO extends LinearOpMode {

    // ===== DRIVE =====
    DcMotor frontLeft, frontRight, backLeft, backRight;

    // ===== CTVL =====
    DcMotorEx ctvl;
    DcMotorEx ctvl2;

    boolean ctvlToggle = false;
    boolean rtLast = false;

    // ===== SHOOTERS =====
    DcMotorEx shooterL, shooterR;

    // ===== RPM ALVOS =====
    final double RPM_A = 1440;
    final double RPM_B = 1230;
    final double RPM_X = 1065;
    final double RPM_Y = 1690;

    boolean shooterOn = false;
    double targetRPM = 0;

    // ===== PID =====
    double powerL = 0, powerR = 0;
    double iL = 0, iR = 0;
    double lastErrL = 0, lastErrR = 0;
    long lastTime;

    final double kP = 0.0007;
    final double kI = 0.0000006;
    final double kD = 0.0003;

    final double POWER_MAX = 1.0;
    final double POWER_MIN = 0.0;

    boolean aLast = false, bLast = false, xLast = false, yLast = false;

    @Override
    public void runOpMode() {

        // ===== DRIVE =====
        frontLeft  = hardwareMap.get(DcMotor.class, "FL0");
        frontRight = hardwareMap.get(DcMotor.class, "FR1");
        backLeft   = hardwareMap.get(DcMotor.class, "BL2");
        backRight  = hardwareMap.get(DcMotor.class, "BR3");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // ===== CTVL =====
        ctvl = hardwareMap.get(DcMotorEx.class, "CTVL");
        ctvl.setDirection(DcMotorSimple.Direction.FORWARD);
        ctvl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ctvl2 = hardwareMap.get(DcMotorEx.class, "CTVL2");
        ctvl2.setDirection(DcMotorSimple.Direction.REVERSE);
        ctvl2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ctvl2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ===== SHOOTERS =====
        shooterL = hardwareMap.get(DcMotorEx.class, "shooterL");
        shooterR = hardwareMap.get(DcMotorEx.class, "shooterR");

        shooterL.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterR.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();
        lastTime = System.currentTimeMillis();

        while (opModeIsActive()) {

            // ===== DRIVE =====
            double drive  = -gamepad1.left_stick_y;
            double turn   =  gamepad1.right_stick_x;
            double strafe = 0;

            if (gamepad1.left_bumper)       strafe = -1.0;
            else if (gamepad1.right_bumper) strafe =  1.0;

            frontLeft.setPower(drive + strafe + turn);
            frontRight.setPower(drive - strafe - turn);
            backLeft.setPower(drive - strafe + turn);
            backRight.setPower(drive + strafe - turn);

            // ===== CTVL (TOGGLE) =====
            boolean rt = gamepad1.right_trigger > 0.3;
            if (rt && !rtLast) ctvlToggle = !ctvlToggle;
            rtLast = rt;

            if (gamepad1.left_trigger > 0.3)
                ctvl.setPower(-1.0);
            else if (ctvlToggle)
                ctvl.setPower(1.0);
            else
                ctvl.setPower(0.0);

            // ===== CTVL2 (RT GAMEPAD2) =====
            if (gamepad2.right_trigger > 0.3)
                ctvl2.setPower(-1.0);
            else
                ctvl2.setPower(0.0);

            // ===== SHOOTER BOTÕES =====
            if (gamepad2.a && !aLast) toggleShooter(RPM_A);
            if (gamepad2.b && !bLast) toggleShooter(RPM_B);
            if (gamepad2.x && !xLast) toggleShooter(RPM_X);
            if (gamepad2.y && !yLast) toggleShooter(RPM_Y);

            aLast = gamepad2.a;
            bLast = gamepad2.b;
            xLast = gamepad2.x;
            yLast = gamepad2.y;

            // ===== PID SHOOTER =====
            if (shooterOn) {
                pidShooter();
                shooterL.setPower(powerL);
                shooterR.setPower(powerR);
            } else {
                shooterL.setPower(0);
                shooterR.setPower(0);
            }

            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("RPM L", shooterL.getVelocity());
            telemetry.addData("RPM R", shooterR.getVelocity());
            telemetry.update();
        }
    }

    // ===== PID =====
    void pidShooter() {
        long now = System.currentTimeMillis();
        double dt = (now - lastTime) / 1000.0;
        if (dt <= 0) dt = 0.01;
        lastTime = now;

        double errL = targetRPM - Math.abs(shooterL.getVelocity());
        double errR = targetRPM - Math.abs(shooterR.getVelocity());

        iL += errL * dt;
        iR += errR * dt;

        double dL = (errL - lastErrL) / dt;
        double dR = (errR - lastErrR) / dt;

        lastErrL = errL;
        lastErrR = errR;

        powerL = Math.max(POWER_MIN,
                Math.min(POWER_MAX, powerL + kP * errL + kI * iL + kD * dL));

        powerR = Math.max(POWER_MIN,
                Math.min(POWER_MAX, powerR + kP * errR + kI * iR + kD * dR));
    }

    // ===== TOGGLE SHOOTER =====
    void toggleShooter(double rpm) {
        shooterOn = !(shooterOn && targetRPM == rpm);
        targetRPM = shooterOn ? rpm : 0;

        powerL = powerR = 0.35;
        iL = iR = 0;
        lastErrL = lastErrR = 0;
        lastTime = System.currentTimeMillis();
    }
}