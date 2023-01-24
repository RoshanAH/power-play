package org.firstinspires.ftc.teamcode.opmodes.testing

import org.firstinspires.ftc.teamcode.core.BaseOpmode
import org.firstinspires.ftc.teamcode.core.Robot
import org.firstinspires.ftc.teamcode.components.Mecanum
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.CoroutineScope

@TeleOp(group = "testing")
class DriveTest : BaseOpmode(){
  lateinit var dt: Mecanum
  override fun setRobot() = object : Robot(){
    override fun mapHardware(map: HardwareMap) {
      dt = Mecanum(map, "fl", "fr", "bl", "br", { 12.0 } ) 
      addComponents(dt)
    }
  }

  override fun onUpdate(scope: CoroutineScope){
    dt.drive(gamepad1)
  }
}
