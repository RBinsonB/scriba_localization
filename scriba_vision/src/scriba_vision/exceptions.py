#!/usr/bin/env python

class FeatureMatchingError(Exception):
	def __init__(self, *args):
		if args:
			self.message = args[0]
		else:
			self.message = None

	def __str__(self):
		if self.message:
			return "feature matching : {0}".format(self.message)
		else:
			return "feature matching error"

class ImageLocalizationError(Exception):
	def __init__(self, *args):
		if args:
			self.message = args[0]
		else:
			self.message = None

	def __str__(self):
		if self.message:
			return "Image Localization: {0}".format(self.message)
		else:
			return "Image Localization"