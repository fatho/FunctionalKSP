module FunctionalKSP.Units

open Microsoft.FSharp.Data.UnitSystems.SI
open System

// GENERAL UNITS

/// Meter
[<Measure>]
type m = UnitSymbols.m

/// Seconds
[<Measure>]
type s = UnitSymbols.s

/// Frequency
[<Measure>]
type Hz = UnitSymbols.Hz

/// Kilogram
[<Measure>]
type kg = UnitSymbols.kg

/// Newtown
[<Measure>]
type N = UnitSymbols.N

/// Degrees
[<Measure>]
type deg

/// Radians
[<Measure>]
type rad

/// Degrees per radian
let degPerRad = 180.<deg/rad> / Math.PI
    
// KSP SPECIFIC UNITS

/// Electric Charge
[<Measure>]
type EC
    
/// Fuel units (no specific equivalent in real world)
[<Measure>]
type fuel


/// Convert a float to float32 while keeping the unit
let inline float32U (x: float<'u>) : float32<'u> = LanguagePrimitives.Float32WithMeasure (float32 x)

/// Convert a float to float32 while keeping the unit
let inline floatU (x: float32<'u>) : float<'u> = LanguagePrimitives.FloatWithMeasure (float x)

type Double with
    member f.As<[<Measure>] 'u>(): float<'u> = LanguagePrimitives.FloatWithMeasure f
        
type Single with
    member f.As<[<Measure>] 'u>(): float32<'u> = LanguagePrimitives.Float32WithMeasure f