package org.firstinspires.ftc.teamcode.subsystems;

class points
{
    points(int RPM,float pitch_big_t,float pitch_small_t)
    {
        targetRPM = RPM;
        pitch_big = pitch_big_t;
        pitch_small = pitch_small_t;
    }
    int targetRPM;
    float pitch_big;
    float pitch_small;
}
public class teleopTable {
    points redrighttable[][] = {
            {},{},{}
    };
    points redlefttable[][] = {
            {},{},{}
    };
    points bulerighttable[][] = {
            {},{},{}
    };
    points bluelefttable[][] = {
            {},{},{}
    };
}
