module ANewDawn.Math.Lambert

(*

This implementation of a solver for the Lambert problem has been taken from 
https://github.com/rodyo/FEX-Lambert/blob/35edc8078b82d3b2ab5ae2bb4118b76afb07fb7a/lambert.m
and translated to F#

Copyright (c) 2018, Rody Oldenhuis
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of this project.

*)


open ANewDawn.Units
open ANewDawn.Math
open ANewDawn.Math.Util
open System

type LambertResult =
    | Solution of (vec3<m/s> * vec3<m/s>) //  * float<m> * float<m>
    | NoSolution
    | Failed

let lambert (p1: vec3<m>) (p2: vec3<m>) (flightTime: float<s>) (numOrbits: int) (muC: float<m^3/s^2>): LambertResult =
    // initial values
    let debug = false

    let tol = 1e-18
    let mutable bad = false

    // work with non-dimensional units
    let r1 = Vec3.mag p1
    let r1vec = p1 / r1
    let r2vec = p2 / r1
    let V = sqrt (muC / r1)
    let T = r1 / V


    // relevant geometry parameters (non dimensional)
    let mr2vec = Vec3.mag r2vec
        

    // decide whether to use the left or right branch (for multi-revolution
    // problems), and the long- or short way
    let leftbranch = sign numOrbits
    let longway = sign flightTime
    let m = abs numOrbits
    let tf = abs flightTime / T
    
    // make 100% sure it's in (-1 <= dth <= +1)
    let dth =
        let dth = acos (clamp -1. 1. (Vec3.dot r1vec r2vec / mr2vec));
        if longway < 0 then 2. * pi - dth else dth
    
    if debug then printfn "V = %.1f    r1 = %.1f   mr2 = %.1f   dth = %.1f" V r1 mr2vec (dth * degPerRad)

    // derived quantities
    let c      = sqrt (1. + square mr2vec - 2. * mr2vec * cos dth) // non-dimensional chord
    let s      = (1. + mr2vec + c) / 2.                     // non-dimensional semi-perimeter
    let a_min  = s / 2.                                    // minimum energy ellipse semi major axis
    let Lambda = sqrt mr2vec * cos (dth/2.) / s            // lambda parameter (from BATTIN's book)
    let crossprd = Vec3.cross r1vec r2vec                 // non-dimensional normal vectors
    let mcr      = Vec3.mag crossprd                      // magnitues thereof
    let nrmunit  = crossprd / mcr                         // unit vector thereof
    
    if debug then printfn "c = %.3f  s = %.3f  a_min = %.3f  Lambda = %.3f mcr = %.3f" c s a_min Lambda mcr

    // Initial values
    // ---------------------------------------------------------

    let logt = log tf // avoid re-computing the same value

    let inn1, inn2 =
        if m = 0 then
            // single revolution (1 solution)
            -0.5233, +0.5233
        elif leftbranch < 0 then
            // multiple revolutions, left branch
            -0.5234, -0.223
        else
            // multiple revolutions, right branch
            +0.7234, +0.5234

    let mutable x1, x2 =
        if m = 0 then
            log (1. + inn1), log(1. + inn2)
        else
            tan (inn1*pi/2.), tan (inn2*pi/2.)


    // since (inn1, inn2) < 0, initial estimate is always ellipse
    let xx = [| inn1; inn2 |]
    let aa = [| a_min / (1. - square inn1); a_min / (1. - square inn2) |]
    
    let bbeta =
        let bbetaF a = float longway * 2. * asin (sqrt ((s-c)/2./a));
        Array.map bbetaF aa

    // make 100.4% sure it's in (-1 <= xx <= +1)
    let aalfa = Array.map (fun x -> 2. * acos (clamp -1. 1. x)) xx

    // evaluate the time of flight via Lagrange expression
    //let y12  =
    //    let tmp = Array.map2 (fun a b -> (a - sin a) - (b - sin b) + 2. * pi * float m) aalfa bbeta
    //    Array.map2 (fun aa tmp -> aa * sqrt aa * tmp) aa tmp
    let y12 = [| aa.[0] * sqrt (aa.[0]) * ((aalfa.[0] - sin(aalfa.[0])) - (bbeta.[0]-sin(bbeta.[0])) + 2. * pi * float m);
                 aa.[1] * sqrt (aa.[1]) * ((aalfa.[1] - sin(aalfa.[1])) - (bbeta.[1]-sin(bbeta.[1])) + 2. * pi * float m) |]

    // initial estimates for y
    let mutable y1, y2 =
        if m = 0 then
            log(y12.[0]) - logt,  log(y12.[1]) - logt
        else
            y12.[0] - float tf, y12.[1] - float tf
    
    // Solve for x
    // ---------------------------------------------------------

    // Newton-Raphson iterations
    // NOTE - the number of iterations will go to infinity in case
    // m > 0  and there is no solution. Start the other routine in
    // that case
    let mutable err = Double.PositiveInfinity
    let mutable iterations = 0
    let mutable xnew = 0.

    while err > tol && not bad do
        iterations <- iterations + 1
        // new x
        xnew <- (x1*y2 - y1*x2) / (y2-y1)

        // copy-pasted code (for performance)
        let x = 
            if m = 0 then
                exp xnew - 1.
            else
                atan xnew * 2. / pi

        let a = a_min / (1. - x * x)
        let alfa, beta =
            if x < 1. then // ellipse
                if debug then printfn "ellipse"
                // make 100.4% sure it's in (-1 <= xx <= +1)
                2. * acos (clamp -1. 1. x), float longway * 2. * asin (sqrt((s-c)/2./a));
            else // hyperbola
                if debug then printfn "hyperbola"
                2. * acosh x, float longway * 2. * asinh (sqrt((s-c)/(-2.*a)))

        // evaluate the time of flight via Lagrange expression
        let tof =
            if a > 0. then
                a*sqrt(a)*((alfa - sin(alfa)) - (beta-sin(beta)) + 2.*pi*float m)
            else
                -a*sqrt(-a)*((sinh(alfa) - alfa) - (sinh(beta) - beta))

        if debug then printfn "alfa = %.1f beta = %.1f tof = %.1f a = %.1f" (alfa * degPerRad) (beta * degPerRad) tof a

        // new value of y
        let ynew =
            if m = 0 then
                log(tof) - logt
            else
                tof - float tf

        // save previous and current values for the next iterarion
        // (prevents getting stuck between two values)
        x1 <- x2;  x2 <- xnew;
        y1 <- y2;  y2 <- ynew;
        // update error
        err <- abs (x1 - xnew)

        // escape clause
        if iterations > 30 then
            bad <- true


    // If the Newton-Raphson scheme failed, try to solve the problem
    // with the other Lambert targeter.
    if bad then
        // NOTE: use the original, UN-normalized quantities
        //[V1, V2, extremal_distances, exitflag] = ...
        //    lambert_LancasterBlanchard(r1vec*r1, r2vec*r1, longway*tf*T, leftbranch*m, muC);
        //return
        Failed
    else
        // convert converged value of x
        let x =
            if m = 0 then
                exp xnew - 1.
            else
                atan xnew * 2. / pi

        //The solution has been evaluated in terms of log(x+1) or tan(x*pi/2), we
        //now need the conic. As for transfer angles near to pi the Lagrange-
        //coefficients technique goes singular (dg approaches a zero/zero that is
        //numerically bad) we here use a different technique for those cases. When
        //the transfer angle is exactly equal to pi, then the ih unit vector is not
        //determined. The remaining equations, though, are still valid.

        // Solution for the semi-major axis
        let a = a_min/(1.-x * x)

        // Calculate psi
        let eta2, eta =
            if x < 1. then // ellipse
                let beta = float longway * 2. * asin (sqrt ((s-c) / 2. / a))
                // make 100.4% sure it's in (-1 <= xx <= +1)
                let alfa = 2. * acos (clamp -1. 1. x)
                let psi  = (alfa-beta)/2.
                let eta2 = 2. * a * square (sin(psi)) / s
                let eta  = sqrt(eta2)
                eta2, eta
            else       // hyperbola
                let beta = float longway * 2. * asinh (sqrt((c-s)/2./a))
                let alfa = 2. * acosh(x);
                let psi  = (alfa-beta)/2.;
                let eta2 = -2.*a*square (sinh(psi)) / s;
                let eta  = sqrt(eta2);
                eta2, eta

        // unit of the normalized normal vector
        let ih =
            if mcr > tol then
                float longway * nrmunit
            // Fix singularity case:
            elif Vec3.angle Vec3.unitX r1vec < tol * 1.<rad> then
                float longway * Vec3.norm (Vec3.cross r1vec Vec3.unitZ)
            else
                float longway * Vec3.norm (Vec3.cross r1vec Vec3.unitX)


        // unit vector for normalized [r2vec]
        let r2n = r2vec/mr2vec;

        // cross-products
        let crsprd1 = Vec3.cross ih r1vec
        let crsprd2 = Vec3.cross ih r2vec

        if debug then printfn "ih = %O cr1 = %O cr2 = %O" ih crsprd1 crsprd2
        
        if debug then printfn "eta = %.3f   eta2 = %.3f   x = %.3f" eta eta2 x
        
        // radial and tangential directions for departure velocity
        let Vr1 = 1. / eta / sqrt a_min * (2. * Lambda * a_min - Lambda - x*eta)
        let Vt1 = sqrt (mr2vec/a_min/eta2 * square (sin(dth/2.)))

        // radial and tangential directions for arrival velocity
        let Vt2 = Vt1/mr2vec
        let Vr2 = (Vt1 - Vt2) / tan(dth/2.) - Vr1;

        // terminal velocities
        let V1 = (Vr1*r1vec + Vt1*crsprd1)*V;
        let V2 = (Vr2*r2n + Vt2*crsprd2)*V;

        Solution (V1, V2)
        //// exitflag
        //exitflag = 1; // (success)

        //// also compute minimum distance to central body
        //// NOTE: use un-transformed vectors again!
        //// extremal_distances = ...
        ////    minmax_distances(r1vec*r1, r1, r2vec*r1, mr2vec*r1, dth, a*r1, V1, V2, m, muC);

        //Failed
