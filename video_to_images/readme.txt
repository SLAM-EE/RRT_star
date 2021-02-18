***********************************************************************
To convert video file into image files:
	1. Create a folder "images"
	2. Run the command: "ffmpeg -i <video file> -vf fps=<fps value> ./images/%d.png"
***********************************************************************
To rename the image files:
	1. Run the command: "python rename.py"
***********************************************************************
To generate "times.txt":
	1. Run the command: ./times
	2. Enter the number of image files
***********************************************************************
Use the file "logitech.yaml" and edit the parameters for "FPS", "FAST Threshold" and "Number of Features" as required

Use the file "mono_test.cc" for monocular with custom dataset and run "build.sh" once

Executable "mono_test" is also included
***********************************************************************
