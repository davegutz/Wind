# Wind
Lab Project
  Verify controllability of turbine system designed elsewhere.
  
# Installation Instructions
1.  Install Link Shell Extension  http://download.cnet.com/Link-Shell-Extension-64-bit/3000-2248_4-75213087.html
        It will make you restart explorer.    
    Link Shell Manual:  http://www.howtogeek.com/howto/16226/complete-guide-to-symbolic-links-symlinks-on-windows-or-linux/
2.  Install Arduino:  Get Arduino .zip
        i.  https://www.arduino.cc/en/Main/Software    get the Windows ZIP file for non-admin install
        ii. Unzip into your “My Documents”
            Ref:  https://www.arduino.cc/en/Guide/PortableIDE  Summary follows so shouldn’t have to go here
3.  Browse to Arduino-1.6.x, the root.   Create a new directory called “portable” alongside the others
4.  Launch the Arduino executable “arduino.exe” found in the root.  This is the IDE.
    a.  Create/Save shortcuts to taskbar and Start
    b.  IDE-File - preferences - uncheck "Show verbose output during compilation" - set Compiler warnings = none.
        Be sure to click OK even if made no changes (do not press “X”).
    c.  Close the IDE.   The defaults are now set.
5.  Browse to root/portable/sketchbook.   Create symbolic link to source
    a.  right-click on potESC in  <root>\portable\sketchbook and “pick link source”
    b.  right-click in sketchbook.  Drop-as Symbolic link.
6.  Open the IDE.   File-open-browse to potWind.ino.   Click it open.
    Close all other IDE windows so IDE remembers your default choice.
     Reference: https://www.arduino.cc/en/Reference/HomePage
        In brief:  Ctrl-r to compile.   Try it now with potWind.    Ask for help if this does not work.
                   Ctrl-u to compile/upload.   Arduino needs to be connected and USB free
7.  Install CoolTerm USB monitor
    http://freeware.the-meiers.org/CoolTerm_Win.zip
    Copy CoolTerm_0.stc in this folder to the desktop for easy starting of USB
8.  Habitually calibrate your ESC:
        - Open loop on board
        - Set POT to max
        - Power the ESC.   Wait for beep
        - Set POT to min.   Wait for two beeps
    Calibration may last 100 runs or it may last 1 run.   Motion of the POT
    can undo calibration if you're not hearing all the beeps.  
    Beware of this.   It's the most common problem.

# FAQ
1.  Coolterm “Port not found.”  
    Rescan ports in Coolterm-Connection-Options
    Restart Coolterm if necessary
    Turn off Arduino IDE USB monitor
2.  Suddenly bad stability.   Recalibrate ESC.
3.  No response.  Check for loose wires on board.
4.  Poor response.   Check for reversed wires on ESC to 3-phase. 
    Confirm direction of airflow into inlet of gas generator.


# Expected Results
Set "#define VECTOR" in potWind.ino
Recompile and upload
Select open or close loop mode as desired on board
Start CoolTerm data save
Press pushbutton on board to begin defined vector test
Stop CoolTerm data save when done
Compare to Saves/v<your kit #>_2017


