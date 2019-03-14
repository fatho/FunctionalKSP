namespace ANewDawn.Math

module Util =
    open ANewDawn.Units

    let inline clamp<[<Measure>] 'u> (lo: float<'u>) (hi: float<'u>) (v: float<'u>): float<'u> = min hi (max lo v)

    let inline square<[<Measure>] 'u> (x: float<'u>): float<'u^2> = x * x
    let inline cube<[<Measure>] 'u> (x: float<'u>): float<'u^3> = x * x * x

    let inline cbrt<[<Measure>] 'u> (x: float<'u^3>): float<'u> = LanguagePrimitives.FloatWithMeasure ((float x) ** (1./3.))

    let pi = System.Math.PI

    let inline acosh x = log (x + sqrt (x * x - 1.))
    
    let inline asinh x = log (x + sqrt (x * x + 1.))

    let inline sinRad (angle: float<rad>) = sin (angle / 1.<rad>)

    let inline cosRad (angle: float<rad>) = cos (angle / 1.<rad>)

    let inline sinDeg (angle: float<deg>) = sinRad (angle / degPerRad)

    let inline cosDeg (angle: float<deg>) = cosRad (angle / degPerRad)