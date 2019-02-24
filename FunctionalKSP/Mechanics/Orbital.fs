module FunctionalKSP.Mechanics.Orbital

open FunctionalKSP
open Units

/// Vis-viva equation (https://en.wikipedia.org/wiki/Vis-viva_equation), solved for v^2.
/// Computes v^2 = GM (2/r - 1/a) where r is the current distance from the bodies center and a the semi-major=axis of the orbit.
let visViva (mu: float<m^3/s^2>) (r: float<m>) (sma: float<m>) =
    mu * (2. / r - 1. / sma)

/// Compute the orbital velocity on an elliptical orbit with semi-major axis `sma` around
/// a body with a gravitational parameter `mu` at the current distance `r` from the bodies center.
/// Computes v^2 = GM (2/r - 1/a) where r is the current distance from the bodies center and a the semi-major=axis of the orbit.
let orbitalVelocity  (mu: float<m^3/s^2>) (r: float<m>) (sma: float<m>) =
    sqrt (visViva mu r sma)