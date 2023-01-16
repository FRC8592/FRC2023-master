package frc.robot;

public enum Color {
    RED (128, 0, 0),
    GREEN (0, 128, 0),
    BLUE (0, 80, 133),
    YELLOW (255, 255, 0),
    PURPLE (138,43,226),
    ORANGE (243, 50, 0),
    OFF (0, 0, 0);

    public final int red;
    public final int blue;
    public final int green;

    Color(int red, int green, int blue){

        this.red = red;
        this.green = green;
        this.blue = blue;
    }

}
