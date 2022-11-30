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
class DrivetrainTuner : BaseOpmode() {

  companion object{
      @JvmField var ticksPerInch = 1.0
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
    telemetry.addData("fl", dt.pos.fl)
    telemetry.addData("fr", dt.pos.fr)
    telemetry.addData("bl", dt.pos.bl)
    telemetry.addData("br", dt.pos.br)
    telemetry.update()
  }
}
