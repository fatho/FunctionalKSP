namespace FunctionalKSP.Control

open FunctionalKSP
open Units
open System

/// PID controller based on the very-well working implementation in kOS
/// (https://github.com/KSP-KOS/KOS/blob/develop/src/kOS.Safe/Encapsulation/PIDLoop.cs)
type PidLoop<[<Measure>] 'i, [<Measure>] 'o>(kp: float<'o/'i>, ki: float<'o/('i*s)>, kd: float<'o*s/'i>, ?minOutput: float<'o>, ?maxOutput: float<'o>) =
    let mutable unwinding = false

    member val Kp = kp with get, set
    member val Ki = ki with get, set
    member val Kd = kd with get, set
    member val Setpoint: float<'i> = 0.0<_> with get, set
    member val Error: float<'i> = 0.0<_> with get, set
    member val ErrorSum: float<'i s> = 0.0<_> with get, set
    member val LastSampleTime: float<s> = Double.MaxValue.As<s>() with get, set
    member val LastInput: float<'i> = 0.0<_> with get, set
    member val MinOutput = defaultArg minOutput (Double.MinValue.As<'o>()) with get, set
    member val MaxOutput = defaultArg maxOutput (Double.MaxValue.As<'o>()) with get, set
    member val Output: float<'o> = 0.<_> with get, set
    member val ExtraUnwind = false with get, set
    
    member val PTerm: float<'o> = 0.0<_> with get, set
    member val DTerm: float<'o> = 0.0<_> with get, set
    member val ITerm: float<'o> = 0.0<_> with get, set
    
    member this.ResetI(): unit =
        this.ErrorSum <- 0.<_>
        this.ITerm <- 0.<_>
        this.LastSampleTime <- Double.MaxValue.As<s>()

    member this.Update(sampleTime: float<s>, input : float<'i>): float<'o> =
        let error = this.Setpoint - input
        let pTerm = error * this.Kp

        let mutable dTerm, iTerm = 0.<_>, 0.<_>

        if sampleTime > this.LastSampleTime then
            let dt = sampleTime - this.LastSampleTime
            if this.Ki <> 0.<_> then
                if this.ExtraUnwind then
                    if sign error <> sign this.ErrorSum then
                        if not unwinding then
                            this.Ki <- this.Ki * 2.
                            unwinding <- true
                    elif unwinding then
                        this.Ki <- this.Ki / 2.
                        unwinding <- false
                iTerm <- this.ITerm + error * dt * this.Ki

            let changeRate = (input - this.LastInput) / dt
            if this.Kd <> 0.<_> then
                dTerm <- -changeRate * this.Kd
        else
            dTerm <- this.DTerm
            iTerm <- this.ITerm

        this.Output <- pTerm + iTerm + dTerm

        if this.Output > this.MaxOutput then
            this.Output <- this.MaxOutput
            if this.Ki <> 0.<_> && this.LastSampleTime < sampleTime then
                iTerm <- this.Output - min (pTerm + dTerm) this.MaxOutput
        elif this.Output < this.MinOutput then
            this.Output <- this.MinOutput
            if this.Ki <> 0.<_> && this.LastSampleTime < sampleTime then
                iTerm <- this.Output - min (pTerm + dTerm) this.MinOutput

        this.LastSampleTime <- sampleTime
        this.LastInput <- input
        this.Error <- error
        this.ITerm <- iTerm
        this.DTerm <- dTerm
        this.PTerm <- pTerm
        this.ErrorSum <- if this.Ki <> 0.<_> then iTerm / this.Ki else 0.<_>
        this.Output

    member this.Update(sampleTime: float<s>, input : float<'i>, setpoint: float<'i>, maxOutput: float<'o>): float<'o> =
        this.Setpoint <- setpoint
        this.MinOutput <- -maxOutput
        this.MaxOutput <- maxOutput
        this.Update(sampleTime, input)