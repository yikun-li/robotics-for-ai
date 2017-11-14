import sys
import mad
import pygame
import subprocess

filename = sys.argv[1]
# print "now attempting to play"
pygame.init()
while True:
	try:
		pygame.mixer.music.load(filename)
		break
	except IOError as e:
		print e
		pass
pygame.mixer.music.play()
# wait until playback is finished
clock = pygame.time.Clock()
while pygame.mixer.music.get_busy():
        clock.tick(30)
subprocess.Popen("rm -f " + filename, shell=True)