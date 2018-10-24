# Computer_Systems_Design
The final project was completed using an Anvyl Spartan 6 FPGA board. The final project was carried out by two other students and myself and the outline provided by Dr. Katkoori and the University of South Florida as seen on this page. The skeleton was provided and the remaining Verilog and PicoBlaze Assembly Code was the responsability of the students.

CDA 4203/4203L Spring 2018
Computer System Design -- Final Project
Prototyping of an Audio Message Recorder
Demo Deadline: Wednesday, 2nd May 2018
This project carries 25% of the lecture grade and 50% of the lab grade. Start early!
Assigned Monday, 2nd April 2018

Overview

In this project, you will prototype an audio recorder/player using the ANVYL FPGA Board.   The player must be implemented with picoBlaze soft microcontroller as the main controller.  The design should have the standard features of an audio recorder, namely, ability to record an audio message, play/pause/delete an audio message selected by the user from the audio library.  The user interface is through the Serial Terminal, push buttons, LEDS, and the dip switches.

  
Design Requirements

The following are the design requirements:
R1) On start up, the system must show a welcome message and then display a menu
1)	Play a message
2)	Record a message
3)	Delete a message
4)	Delete all messages
5)	Volume control

R2) For options 1 and 3 above, the system should display the audio library and the user should be able to scroll up or down in the list and then select it to play or delete.

R3) When the memory is full, the system should display a “MEMORY FULL” message.

R4) While playing an audio message, the user be able to pause/resume and skip the message.

R5) While a message is being played or recorded the user should be able to interact with the system through the menu.  In other words, the picoBlaze should not be tied up with playing/recording the message.

R6) The total record time should be at least 4 minutes.

R7) The recorder should be able to record/store atleast 5 messages of variable duration.


Project Team

You are allowed to work with at most two students in the class.  A reasonable work distribution is along these lines – serial terminal interfacing, Codec interfacing, and User Control and Memory Interfacing.  Each project member’s grade will be decided based on their contribution.

Deliverables and Demo

•	Project Demo: Your team must give a demo during the exam week on Wednesday, 2nd May.  A signup sheet will be posted on TAs’ office door.  The demo will be for 30 minutes and all team members must be present during the demo. 
•	Project Report: A project report template will be posted on Canvas.  The project report is due by 5 pm Friday, 4th May.
•	Project Code: You should upload an archive of your project code along with the project report on Canvas.

Grade Distribution

The grade distribution is based on (a) your report; and (b) the working features of your design:
•	Message Recording
o	(4 pts) Can record a message
•	Message Playback
o	(3 pts) Plays a message
o	(2 pts) Pause and continue
o	(2 pts) Delete a message
o	(1 pt)  Delete all messages
o	(1 pt)  Skip a message
•	User Interface
o	(2 pts) Scroll up/down 
o	(3 pts) Can interact while system is playing/recording a message
o	(2 pts) Volume control with level indicator
•	System Messages
o	(2 pts) Welcome message – display for atleast 5 seconds or until user presses a button
o	(2 pts) System memory full
•	Total Record time is at least 4 minutes
o	(3 pts) Constraint met
•	Final Report
o	(3 pts) Report with design details

Extra Credit (6 pts): For the above regular credit, you will implement the user interface with Serial Terminal (PuTTY or similar interface) display, push-buttons, LEDs, and dip switches. For the extra credit, you will also implement the same user interface using touch screen. All messages (welcome message, memory full message) and commands (play, record, pause, delete, skip, volume control) must be displayed and accepted respectively, via touch screen.  

