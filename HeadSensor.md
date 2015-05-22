<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->
**Table of Contents**  *generated with [DocToc](http://doctoc.herokuapp.com/)*

- [1. General Info](#1-general-info)
- [2. Hardware](#2-hardware)
- [3. Wiring](#3-wiring)
- [4. Setup](#4-setup)
- [5. Software](#5-software)
- [6. Debouncing Algorithm](#6-debouncing-algorithm)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

# 1. General Info
The page documents how to make a head sensor for da Vinci Research Kit.  The goal is to detect the presence of the operator at the master console using a proximity sensor mounted on the stereo display.

# 2. Hardware

 * 1 Digital Distance Sensor 10cm
   * http://www.pololu.com/product/1134
 * 20 Molex pin connectors (Digikey Part No. WM2510-ND) 
   * NOTE: we only need 6 pin connectors
 * 12-feet S-Video Cable 
    * just get one that is long enough
    * any 3 wire cable would work, we just happened to have an old S-Video cable handy 
 * 1 3-pin straight connector (for the connection to the dVRK controller)
 * 1 3-pin right angle connector (for the connection on the sensor side)

# 3. Wiring

| Sensor | Cable | Controller (J18)     |
|--------|-------|----------------------|
| VIN    | Red   | Pin 8 (VCC-CON-A 5V) |
| GND    | White | Pin 6 (GND)          |
| OUT    |Yellow | Pin 7 (HOME4)        |

Notes:
* J18 is a 15-pin connector labelled DOF 4 on the back of the dVRK controller
* Please connect head sensor and foot pedal on same controller box

# 4. Setup

* Setup option 1: base

  ![](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/head/dvrk-head-sensor-base.jpg)

* Setup option 2: side

  ![](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/head/dvrk-head-sensor-side.jpg)  

* Connection to controller box

  ![](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/head/dvrk-head-sensor-controller.jpg)

# 5. Software
 * Rerun MATLAB XML config generator to make sure the digital input is renamed "HEAD"
 * Update JSON config file to set the presence sensor


# 6. Debouncing Algorithm

```matlab
% Head Sensor: Debouncing Algorithm
% Zihan Chen
% 2015-05-22 

clc; clear; close all;

x0 = 0;
u = ones(1, 1000);
x = zeros(1, 1000);
v = zeros(1, 1000);

% 0 -> 1
u(1:100) = zeros(1,100);
u(101:120) = (rand(1,20)>0.8);
u(121:140) = (rand(1,20)>0.6);
u(141:160) = (rand(1,20)>0.4);
u(161:180) = (rand(1,20)>0.2);

% 1 -> 0
u(201:500) = ones(1,300);
u(501:520) = (rand(1,20)<0.8);
u(521:540) = (rand(1,20)<0.6);
u(541:560) = (rand(1,20)<0.4);
u(561:580) = (rand(1,20)<0.2);
u(581:600) = (rand(1,20)<0.0);
u(601:1000) = zeros(1,400);

w = 0.98;  % weighting factor
low = 0.2; high = 0.8;
x(1) = x0;
for i = 2:length(u)
  % compute confidence
  x(i) = w * x(i-1) + (1-w) * u(i);  
  
  % hysteresis 
  if ((v(i-1) == 1) && (x(i) < low)) 
    v(i) = 0;
  elseif ((v(i-1) == 0) && (x(i) > high))
    v(i) = 1;
  else
    v(i) = v(i-1);
  end  
end

figure;
subplot(3,1,1); plot(u); 
title('Sensor Input'); axis([0 1000 -0.1 1.1]);

subplot(3,1,2); plot(x); 
title('Confidence'); axis([0 1000 -0.1 1.1]);

subplot(3,1,3); plot(v); 
title('Present'); axis([0 1000 -0.1 1.1]);
```

**Sample Result:**  
![](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/head/dvrk-head-sensor-algorithm.jpg)