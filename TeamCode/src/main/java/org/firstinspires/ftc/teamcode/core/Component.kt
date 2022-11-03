package org.firstinspires.ftc.teamcode.core;

import kotlinx.coroutines.CoroutineScope

interface Component {
    fun init(scope: CoroutineScope)
    fun start(scope: CoroutineScope)
    fun update(scope: CoroutineScope)
}
