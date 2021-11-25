import subprocess
python2_command = "python2 call_robot.py --command 1 2 3 4 --filename image.png --topic /hires/image_raw/compressed"
process = subprocess.Popen(python2_command.split(), stdout=subprocess.PIPE)
output, error = process.communicate()
