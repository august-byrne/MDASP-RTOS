
# MDASP - RTOS

This is the embedded part of the MDASP (modular digital audio signal processor) project. The project is built on top of the Espressif IDF libraries. The software provides all of the I2S, audio processing, and bluetooth functionality of the project on the MDASP board's side. A seperate project named MDASP - Android provides control of this board through an android app.

## How to Use

### Hardware Required

* A MDASP board
* A UART programmer with power and programming wires.

### Configure the Project

```
idf.py menuconfig
```

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.
