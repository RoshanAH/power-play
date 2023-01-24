package org.firstinspires.ftc.teamcode.opmodes.testing

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.HardwareMap
import com.acmerobotics.dashboard.config.Config
import kotlinx.coroutines.CoroutineScope
import org.firstinspires.ftc.teamcode.components.Mecanum
import org.firstinspires.ftc.teamcode.core.BaseOpmode
import org.firstinspires.ftc.teamcode.core.Robot

@TeleOp(group="testing")
@Config
class DrivetrainEncoderTest : BaseOpmode() {

  companion object{
      @JvmField var ticksPerInch = 30.9861111
  }

  private lateinit var dt: Mecanum

  override fun setRobot() =
      object : Robot() {
        override fun mapHardware(map: HardwareMap) {
          dt = Mecanum(hardwareMap, "fl", "fr", "bl", "br")
          addComponents(dt)
        }
      }

  override fun onInit(scope: CoroutineScope){

  }

  override fun onStart(scope: CoroutineScope) {}

  override fun onUpdate(scope: CoroutineScope) {
    dt.ticksPerInch = ticksPerInch
    dt.pos.apply{ 
      telemetry.addData("fl", fl)
      telemetry.addData("fr", fr)
      telemetry.addData("bl", bl)
      telemetry.addData("br", br)
    }
    telemetry.update()
  }
}
