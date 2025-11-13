# Data Link Protocol

## Work overview

* Implement a data link layer protocol, according to the specification provided in [this](./guide-lab1.pdf) guide
    * this protocol implements `transmitter` and `receiver` functionality to transfer a file stored on a computer hard disk between computers connected through a RS-232 serial cable

* Develop a simple transmitter and receiver data transfer application to test the protocol, according to the specification provided
    * the application to be developed uses/invokes the functions implemented by the data link layer protocol
        * the data link layer protocol thus offers/exposes an API to the upper layer

* Development environment
    * PC running LINUX
    * Programming language – C
    * Serial port RS-232 (asynchronous communication)

## Goal and General Functionality

* Goal of the Data Link Layer Protocol
    * Provide reliable communication between two systems connected by a communication medium – in this case, a serial cable
* Generic functions of data link protocols
    * Framing
        * Packaging and synchronisation/delimitation
    * Connection establishment and termination
    * Frame numbering
    * Acknowledgement
    * Error control (Stop-and-Wait)
    * Flow control

## INSTRUCTIONS FOR SERIAL PORT PROTOCOL

### Project Structure

- `bin/`: Compiled binaries.
- `src/`: Source code for the implementation of the link-layer and application layer protocols. Students should edit these files to implement the project.
- `include/`: Header files of the link-layer and application layer protocols. These files must not be changed.
- `cable/`: Virtual cable program to help test the serial port. This file must not be changed.
- `main.c`: Main file. This file must not be changed.
- `Makefile`: Makefile to build the project and run the application.
- `penguin.gif`: Example file to be sent through the serial port.

### Instructions to Run the Project

1. Edit the source code in the `src/` directory.
2. Compile the application and the virtual cable program using the provided `Makefile`.
3. Run the virtual cable program (either by running the executable manually or using the Makefile target):

    ```bash
    ./bin/cable_app
    ```
    ```bash
    make run_cable
    ```
4. Test the protocol without cable disconnections and noise
	* Run the receiver (either by running the executable manually or using the Makefile target):
    ```bash
    ./bin/main /dev/ttyS11 rx penguin-received.gif
    ```
    ```bash
    make run_rx
    ```

	* Run the transmitter (either by running the executable manually or using the Makefile target):
    ```bash
    ./bin/main /dev/ttyS10 tx penguin.gif
    ```
    ```bash
    make run_tx
    ```

	* Check if the file received matches the file sent, using the diff Linux command or using the Makefile target:
    ```bash
    diff -s penguin.gif penguin-received.gif
    ```
    ```bash
    make check_files
    ```

5. Test the protocol with cable disconnections and noise
	* Run receiver and transmitter again
	* Quickly move to the cable program console and press 0 for unplugging the cable, 2 to add noise, and 1 to normal
	* Check if the file received matches the file sent, even with cable disconnections or with noise

### Report 

You can find the final report (in Portuguese) [here](./report.pdf).