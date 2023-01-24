package org.firstinspires.ftc.teamcode.opmodes.testing

import org.firstinspires.ftc.teamcode.core.Robot
import org.firstinspires.ftc.teamcode.components.Mecanum
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.acmerobotics.dashboard.config.Config
import kotlinx.coroutines.CoroutineScope
import kotlin.math.abs
import kotlin.math.sign

@TeleOp(group = "testing")
@Config
class TurretTest : LinearOpMode(){

  companion object{
    @JvmField var ticksPerDegree = 4.26666
    @JvmField var tolerance = 1.0
    @JvmField var maxPower = 0.6
    @JvmField var p = 0.01
    @JvmField var s = 0.2
    @JvmField var d = 0.0
    @JvmField var targetPos = 0.0
  }

  override fun runOpMode(){
    val slides = hardwareMap.dcMotor.get("slides")
    slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
    slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)

    slides.setDirection(DcMotorSimple.Direction.REVERSE)

    val turret = hardwareMap.dcMotor.get("turret")
    turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
    turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)

    turret.setDirection(DcMotorSimple.Direction.REVERSE)

    val extension = hardwareMap.servo.get("extension")
    extension.setPosition(0.7)
    
    waitForStart()

    var lastTime = System.nanoTime() * 1e-9
    var lastError = 0.0

    while (opModeIsActive()){
      val rawSlides = slides.getCurrentPosition()
      val time = System.nanoTime() * 1e-9
      val deltaTime = time - lastTime

      lastTime = time

      val posSlides = rawSlides.toDouble() / SlidesTest.maxticks

      val errorSlides = 0.5 - posSlides

      val raw = turret.getCurrentPosition()
      val pos = raw / ticksPerDegree

      val error = targetPos - pos
      val derivative = (lastError - error) / deltaTime
      lastError = error

      val powerSlides = errorSlides * SlidesTest.p + if (targetPos > 0.05) SlidesTest.gravity else 0.0
      val power = error * p + if(abs(error) < tolerance) 0.0 else (sign(error) * s) + derivative * d
      
      slides.setPower(powerSlides)
      turret.setPower(power.coerceIn(-maxPower, maxPower))

      telemetry.addData("rawPos", raw)
      telemetry.addData("pos", pos)
      telemetry.addData("error", error)
      telemetry.update()
    }
  }
}
