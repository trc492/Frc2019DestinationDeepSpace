package raspivision;

/**
 * Represents a pair of retroreflective tapes.
 */
class TargetData
{
    public int x, y, w, h;

    public TargetData(int x, int y, int w, int h)
    {
        this.x = x;
        this.y = y;
        this.w = w;
        this.h = h;
    }

    @Override
    public boolean equals(Object o)
    {
        if (!(o instanceof TargetData))
        {
            return false;
        }
        TargetData data = (TargetData) o;
        return data.x == x && data.y == y && data.w == w && data.h == h;
    }
}
