Copy all files within the directory of the executable Matlab.
You can upload new simulations: just save a new csv file in the same folder.

You have to download with your smartphone the applications "IMU+GPS Sensor Stream"
and then you have to activate the following sensors: magnetometer, gyroscopes and
accelerometer. You have to save the file as csv into folder and rename into editor
filename with the same name of the saved file csv, like the examples.
 
IMPORTANT: The version used is Matlab R2010a in which the function
igrf11 is not present. It has since been downloaded from the internet
a package for IGRF, called IGRF, and provided into zip package.
To incorporate this feature into Matlab's tools you perform the following steps:
1. copy the folder IGRF
2. enter the installation folder of matlab and look for the sub-folder
"toolbox".
3. copy the folder IGRF inside the toolbox 
4. now look for Matlab's toolbar "Set path" .
5. then browse to the folder IGRF
6. click on Add with subfolders
7. click on Save
Now you can use the IGRF

Each time you run the program, you are asked if you want make the animation.
If the answer is yes, you will be saved all frames of the animation in the 
subfolder smartphone. With these frames you can create a movie manually using
the program "Windows Live Movie Maker."

In file_calibration.txt are saved each time the parameters bias and scale factors,
possibly to be used for subsequent executions of the program.

smartphone.wrk is instead the model of the phone recreated in 3D vrworld
application of matlab .

Finally, the package contains functions used (lowercase)
and the main to start ( ACC_MAG_SENSORS )
-------------------------------------------------- ------------------------
Sapienza University of Rome
Course Navigation Systems
A.A.2013 - 2014
-------------------------------------------------- ------------------------
Ennio Condoleo (1303549)
enniocon@hotmail.it