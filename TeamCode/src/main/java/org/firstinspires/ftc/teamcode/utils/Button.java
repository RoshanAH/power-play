package org.firstinspires.ftc.teamcode.utils;

public class Button {

    private boolean inter;
    private boolean toggle;

    public Button(){
        inter = false;
    }

    public boolean get(boolean statement){

        if(statement && !inter){
            inter = true;
            toggle = !toggle;
            return true;
        } else if(!statement){
            inter = false;
            return false;
        }

        return false;
    }

    public boolean toggle(){
        return toggle;
    }

}
