# feather_cgms
This project uses an Adafruit Feather NRF52 as a CGMS.  
This board connects to a CC2500 via SPI, and picks up data from a dexcom G4.  
The data is converted into a Glucose reading (or something) and relayed on to a MI Band 2.  You need a separate app (on your phone) to determine a Slope / Intercept value to feed to this app.
All relevant information should be available via the CGMS service, through the isig, slope, intercept and battery characteristics.
Your app should read the ISIG, determine a Slope and Intercept value throug the algo of your choice, then write it back through the exposed characteristics.
