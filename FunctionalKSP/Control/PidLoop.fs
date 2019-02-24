namespace FunctionalKSP.Control

open FunctionalKSP
open Units

type PidLoop<[<Measure>] 'i, [<Measure>] 'o>(kp: float<'o/'i>, ki: float<'o/('i*s)>, kd: float<'o*s/'i>) =
     let mutable error: float<'i> = 0.0<_>
     let mutable errorSum: float<'i * s> = 0.0<_>
     let mutable output: float<'o> = 0.<_>
     let mutable lastInput: float<'i> = 0.0<_>

     member val Kp = kp with get, set
     member val Ki = ki with get, set
     member val Kd = kd with get, set

     member val Setpoint: float<'i> = 0.0<_> with get, set

     member this.Update(dt: float<s>, input : float<'i>): float<'o> =
            assert (dt > 0.0<s>)

            error <- this.Setpoint - input
            errorSum <- errorSum + error * dt
            let change = (input - lastInput) / dt

            let pTerm = this.Kp * error
            let iTerm = this.Ki * errorSum
            let dTerm = this.Kd * change

            lastInput <- input
            output <- pTerm + iTerm + dTerm
            output