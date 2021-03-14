# FunFutureFan
Here is the code for my "Fan of the Future" project, my science fair project for 2020-2021.

## Prototype

![Image of the prototypes](https://github.com/SamskrithRaghav/FunFutureFan/blob/main/img/Fan%20of%20the%20Future%20Prototype.png)

### REQUIREMENTS
- ESP32-CAM
- Arduino Uno
- Arduino IDE

### ABSTRACT
As global warming worsens, people need a way to stay cool while conserving energy. People, especially in poor countries, employ fans to stay cool in an increasingly warmer climate. However, most fans are either fixed or have a predetermined angle of oscillation. This often results in wasted energy, because the fan either does not cover everyone in the room or wastes energy by blowing air to the wall. The goal of my project was to create a relatively inexpensive oscillating table fan which uses artificial intelligence to detect people’s faces and adjust the angle of oscillation duly, saving energy and time. The first prototype of this invention uses an Arduino Mega 2560 Circuit Board, an ESP32-CAM with embedded artificial intelligence, a servo motor for oscillation, a DC motor to simulate the actual fan, and four 1.5 V batteries to power the servo and the DC motor. Some additional features which would help optimize the fan are: a sensor to activate the servo and DC motor only when someone is in the room, a temperature sensor which controls the DC motor/fan rotation speed, and a more powerful AI camera to help optimize the code. The fan worked mostly as intended; it could detect faces and oscillate the fan between them. However, it couldn’t detect more than two faces at a time, and some hardware limitations prolonged the code and made the processing time much slower. Further iterations of the fan must be made in order to truly achieve the original objective.
