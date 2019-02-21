namespace FunctionalKSP

module Units =

    open Microsoft.FSharp.Data.UnitSystems.SI

    /// Meter
    [<Measure>]
    type m = UnitSymbols.m

    /// Seconds
    [<Measure>]
    type s = UnitSymbols.s

    /// Kilogram
    [<Measure>]
    type kg = UnitSymbols.kg

    /// Newtown
    [<Measure>]
    type N = UnitSymbols.N