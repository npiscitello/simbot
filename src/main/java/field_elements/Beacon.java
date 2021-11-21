package field_elements;

public class Beacon {

    public enum Color {
        RED,
        GREEN,
        BLUE,
        YELLOW,
        UNKNOWN
    }

    private double _distance = -1;
    private double _direction = 0;
    private Color _req_color = Color.UNKNOWN;
    private Color _cur_color = Color.UNKNOWN;

    // direction in degrees from robot straight ahead, positive = clockwise
    public Beacon(double distance, double direction, Color color) {
        _req_color = color;
    }

    public double   getDistance() { return _distance; };
    public double   getDirection() { return _direction; };
    public Color    getRequiredColor() { return _req_color; }
    public Color    getCurrentColor() { return _cur_color; }

    public void     setDistance( Double distance) { _distance = distance; };
    public void     setDirection( Double direction) { _direction = direction; };
    public void     setCurrentColor( Color color ) { _cur_color = color; }

    public boolean isColorSet() {
        return _cur_color == _req_color;
    }
}
