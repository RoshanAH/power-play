package org.firstinspires.ftc.teamcode.opmodes.testing

import org.firstinspires.ftc.teamcode.core.Robot
import org.firstinspires.ftc.teamcode.components.Mecanum
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.acmerobotics.dashboard.config.Config
import kotlinx.coroutines.CoroutineScope

@TeleOp(group = "testing")
@Config
class SlidesTest : LinearOpMode(){

  companion object{
    @JvmField var maxticks = 1100
    @JvmField var p = 30.0
    @JvmField var d = 0.0
    @JvmField var gravity = 0.17
    @JvmField var targetPos = 0.0
  }

  override fun runOpMode(){
    val slides = hardwareMap.dcMotor.get("slides")
    slides.setDirection(DcMotorSimple.Direction.REVERSE)
    
    waitForStart()

    var lastTime = System.nanoTime() * 1e-9
    var lastError = 0.0

    while (opModeIsActive()){
      val raw = slides.getCurrentPosition()
      val time = System.nanoTime() * 1e-9
      val deltaTime = time - lastTime
      lastTime = time
      val pos = (raw).toDouble() / maxticks

      val error = targetPos - pos
      lastError = error

      val derivative = (lastError - error) / deltaTime

      val power = error * p + derivative * d + if (targetPos > 0.05) gravity else 0.0
      
      slides.setPower(power)

      telemetry.addData("rawPos", raw)
      telemetry.addData("pos", pos)
      telemetry.update()
    }
  }
}
