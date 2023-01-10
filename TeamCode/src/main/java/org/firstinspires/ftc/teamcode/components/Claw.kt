package org.firstinspires.ftc.teamcode.components

import com.qualcomm.robotcore.hardware.HardwareMap
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.delay
import org.firstinspires.ftc.teamcode.core.Component

 class Claw (map : HardwareMap, claw: String, horizontal : String)  : Component {
     var open = 0.5
     var close = 0.5
     var clawPos = open
         private set

     fun open(){
         clawPos = open
     }

     fun close(){
         clawPos = close
     }

    override fun init(scope: CoroutineScope) {
        TODO("Not yet implemented")
    }

    override fun start(scope: CoroutineScope) {
        TODO("Not yet implemented")
    }

     override fun update(scope: CoroutineScope) {
         TODO("Not yet implemented")
     }
 }