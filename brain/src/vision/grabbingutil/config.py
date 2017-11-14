config = {
	# region of interest of the captured images. Used by both the tablemodule
	# as the grabbingmodule. All calculations (segmentization, detection)
	# should support and adhere this ROI. As far as I know, they do.
	# If you make the ROI smaller, make sure the left_arm_point and
	# right_arm_point are still within the ROI.
    "roi" : (0, 0, 630, 380),
	
	# Default image used by the TiltFilter to compensate for the skew of the
	# kinect camera. If this is set to the string "source", it will use the
	# first image it obtains from the camera. If a path (absolute, or relative
	# to the *module.py file) is given, it will load that image on start-up.
	# If None, filtering is disabled.
    "gradient" : "home",

	# Points used by the grabbingmodule (and via that by the objectdetector)
	# to guess where the arms are. If an object is seen at one of these two
	# points it is marked as the left or right arm.
	# Keep in mind that these points have to be in the ROI. 
    "left_arm_point" : (230, 340),
    "right_arm_point" : (420, 340)
}

