namespace UnitySensors.Interface.Std
{
    /// <summary>
    /// An integer reading, typically a monotonically increasing event counter.
    /// A counter survives a sampling rate lower than the rate of the events it
    /// counts, which a boolean state cannot: consumers take the difference between
    /// two readings instead of watching for an edge.
    /// </summary>
    public interface IIntStateInterface
    {
        public int state { get; }
    }
}
