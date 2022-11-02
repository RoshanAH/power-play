package org.firstinspires.ftc.teamcode.opmodes.testing

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.HardwareMap
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import org.firstinspires.ftc.teamcode.components.Slides
import org.firstinspires.ftc.teamcode.core.BaseOpmode
import org.firstinspires.ftc.teamcode.core.Robot

@TeleOp(group = "testing")
@Config
class SlideEncoderTest : BaseOpmode() {

  companion object{
    @JvmField var maxticks = 1125
    @JvmField var p = 10.0
    @JvmField var d = 0.02
    @JvmField var gravity = 0.1

    @JvmField var open = 0.5
    @JvmField var close = 0.5
    @JvmField var clawPos = open
    @JvmField var targetPosition = 0.0
  }

  lateinit var slides: Slides

  override fun setRobot() =
      object : Robot() {
        override fun mapHardware(map: HardwareMap) {
            slides = Slides(map, "left", "right", "claw")
            addComponents(slides)
          }
      }

  override fun onInit() {
    //        FTCDashboard is the web browser app that we use to tweak and see values from out computer
        val dash = FtcDashboard.getInstance();
//        This diverts any incoming telemetry to both the driver station's telemetry and FTCDashboard's telemetry
        telemetry = MultipleTelemetry(telemetry, dash.getTelemetry());
  }

  override fun onUpdate(){
    slides.maxticks = maxticks 
    slides.p = p 
    slides.d = d 
    slides.gravity = gravity 

    slides.open = open 
    slides.close = close 
    slides.close()
    slides.targetPosition = targetPosition 
    
    telemetry.addData("slideTicks", slides.position * slides.maxticks)
    telemetry.addData("slidePos", slides.position)
    telemetry.update()
  }
}
