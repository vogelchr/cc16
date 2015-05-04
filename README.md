# cc16
Linux driver for **Wiener** **CC16/PC16** **CAMAC** **controller/ISA-card**.

CAMAC is an ancient standard for data acquisition in (mostly) nuclear
or high energy physics from the 1970s.
See: http://en.wikipedia.org/wiki/Computer_Automated_Measurement_and_Control

A CAMAC data acquisition system consists of one or more 19" *Crates*, which
are metal boxes into which the individual I/O cards are plugged in. This
Crate will also provide power to your cards. The rightmost card in this
CAMAC crate is the *Crate* *Controller*. This *Crate* *Controller* will
either have a CPU on its own or will provide connection to the controlling
computer.

CC16 is the Crate Controller manufactured by W-IE-NE-R (Wiener)
Plein & Baus GmbH up to around the year 2000, and PC16 the corresponding
ISA card to plug into a PC of roughly the same vintage.

Documentation can (as of 2015) still be found on the Wiener Website:
  http://file.wiener-d.com/documentation/CC16-PC16-PC16/
  
This kernel module acts as a very thin wrapper around the hardware interface
of the PC16 card to post "NAF" commands ("NAF" are the addresses used in
the CAMAC system) to the cards to trigger actions and read back data.

If you still have no idea what this all means, then this software most likely
will not be useful for you :-).

Written mostly in 2007 by Christian Vogel <vogelchr@vogel.cx>, then a
PhD-Student at the University of Erlangen Physics Institute, Department II (PI2).
