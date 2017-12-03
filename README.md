# nogasm
Software and hardware for Arduino-based orgasm prediction / detection ... on protoboard!

The nogasm is an amazing toy.  I wanted to make something easier to hack and more accessible to the homebuilder.  Fortunately headcrabned was kind enough to open source the whole thing.

I ported it the Arduino Uno, and hacked around on perfboard until I had it working.  It works great and only requires a handful of readily available through-hole components.

Some aspects have been simplified compared to the original electronics:
* The DIP switches have been eliminated.  Options are configured in the source instead.

### Vibrator
The vibrator is made from a Mabuchi RS-555PH motor, with an eccentric mass pressed onto the shaft. These can be purchased pre-assembled, or built from the motor and some .88" to 1" diameter cylindrical brass stock.
The vibrator case is compatible with attachments designed for the Hitachi Magic Wand. The motor is press-fit or glued into the two halves of the case.

This work is licensed under a Creative Commons Attribution-NonCommercial 4.0 International Licence, available at http://creativecommons.org/licenses/by-nc/4.0/.
